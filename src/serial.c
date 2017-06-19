/*
Copyright (c) 2015, Intel Corporation. All rights reserved.
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice,
*this list of conditions and the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice,
*this list of conditions and the following disclaimer in the documentation
*and/or other materials provided with the distribution.
*
*3. Neither the name of the copyright holder nor the names of its contributors
*may be used to endorse or promote products derived from this software without
*specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
*AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*POSSIBILITY OF SUCH DAMAGE.
*/
#include<stdio.h>
#include<stdlib.h>
#include <stddef.h>
#include <fcntl.h>
#include <time.h>
#include "client.h"


#define FILE_TEST
//#define TESTFILE	"/dev/ttyS0"
#define TESTFILE	"testfile2"

int sfd=0;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_ss = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond=PTHREAD_COND_INITIALIZER;
static int ThreadsExit=1;
pthread_t p_tid[2];


#define BUFFER_SIZE 100

#define HEAD_BYTE1_VALUE	0xa5
#define HEAD_BYTE2_VALUE	0x7f
#define TAIL_BYTE1_VALUE	0x7f
#define TAIL_BYTE2_VALUE	0x5A

/*including checksum tail1 and tail2*/
#define MSG_TAIL_SIZE	3

typedef enum cmd_type_s{
    CMD_TYPE_CHECK=0x0,
    CMD_TYPE_TEST=0x1,
    CMD_TYPE_PAIR=0x2,
    CMD_TYPE_PAIR_RSP=0x3,
    CMD_TYPE_PAIR_BCK=0x4,
    CMD_TYPE_PAIR_BCK_RSP=0x5,
}cmd_type_t;

/*serial response message,except for checksum,tail 2 bytes*/
typedef struct serial_rsp_msg{
    unsigned char head1;
    unsigned char head2;
    unsigned char ret_num1;
    unsigned char ret_num2;
    unsigned char floorid;
    unsigned char roomid;
    unsigned char cmd_id;
    unsigned char data_len;
    /*these 3 bytes should be parsed in last*/
    unsigned char checksum;
    unsigned char tail1;
    unsigned char tail2;

    unsigned char data[0];
}serial_rsp_msg_t;

enum header_send_index_e{
    INDEX_SEND_HEAD1=0,
    INDEX_SEND_HEAD2=1,
    INDEX_SEND_FLOORID=2,
    INDEX_SEND_ROOMID=3,
    INDEX_SEND_CMDID=4,
    INDEX_SEND_DATA_LEN=5,
    INDEX_SEND_HEADER_SIZE=6,
};

enum header_recv_index_e{
    INDEX_RECV_HEAD1=0,
    INDEX_RECV_HEAD2=1,
    INDEX_RECV_RETNUM1=2,
    INDEX_RECV_RETNUM2=3,
    INDEX_RECV_FLOORID=4,
    INDEX_RECV_ROOMID=5,
    INDEX_RECV_CMDID=6,
    INDEX_RECV_DATA_LEN=7,
    INDEX_RECV_HEADER_SIZE=8,
};


typedef struct serial_resending_msg_s{
    int rewrite;
    int buf_len;
    unsigned char buf[100];
    struct list_head list;
}serial_resending_msg_t;

LIST_HEAD(sending_list);


static serial_resending_msg_t *get_first_node(struct list_head *head)
{
    if( (head->next != NULL) && ( !list_empty(head) ) ){
	return list_entry(head->next,struct serial_resending_msg_s,list);
    }
    return NULL;
}

static void rm_first_node(struct list_head *head)
{
    if( head->next != NULL ){
	struct serial_resending_msg_s *node = list_entry(head->next,struct serial_resending_msg_s,list);
	node->rewrite=0;
	node->buf_len=0;
	list_del_init(head->next);
	free(node);
    }
}

void print_list()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&sending_list){
	struct serial_resending_msg_s *node = list_entry(plist,struct serial_resending_msg_s,list);
	sbc_print("List entry:p=%p, buf_len=%d,rewrite=%d\n",node, node->buf_len,node->rewrite);
    }
}

/*msg protocal
    char head1;
    char head2;
    char floorid;
    char roomid;
    char cmd_id;
    char data_len;
    char data[0];
    char checksum;
    char tail1;
    char tail2;
*/
unsigned char *send_serial_msg(char floorid,\
			char roomid, \
			cmd_type_t cmd_id, \
			char *data,  \
			unsigned int data_len)
{
    int i=0,write_size;

    pthread_mutex_lock(&mutex);
    //serial_gpio_config(true);

    unsigned int buf_len=data_len+INDEX_SEND_HEADER_SIZE+MSG_TAIL_SIZE;

    /*It must use the unsigned char,or >128 will error*/
    unsigned char *buf=(unsigned char *)malloc(buf_len+2);
    buf[INDEX_SEND_HEAD1]=HEAD_BYTE1_VALUE;
    buf[INDEX_SEND_HEAD2]=HEAD_BYTE2_VALUE;
    buf[INDEX_SEND_FLOORID]=floorid;
    buf[INDEX_SEND_ROOMID]=roomid;
    buf[INDEX_SEND_CMDID]=cmd_id;
    buf[INDEX_SEND_DATA_LEN]=data_len;

    unsigned char checksum=floorid+roomid+cmd_id+data_len;
    if(data_len>0 && data){
	memcpy(buf+INDEX_SEND_HEADER_SIZE,data,data_len);
	for(i=0;i<data_len;i++){
	    checksum=checksum+buf[INDEX_SEND_HEADER_SIZE+i];
	}
    }
    checksum=~(checksum&0xff)+0x01;

    int tmp_len=INDEX_SEND_HEADER_SIZE+data_len;
    buf[tmp_len]=checksum;
    buf[tmp_len+1]=TAIL_BYTE1_VALUE;
    buf[tmp_len+2]=TAIL_BYTE2_VALUE;

    for(i=0;i<buf_len;i++){
	blue_print("buf[%d]=0x%x\n",i,buf[i]);
    }

    /*back up the sending msg into list*/
    serial_resending_msg_t *sending_msg=malloc(sizeof(serial_resending_msg_t));
    sending_msg->buf_len=buf_len;
    memcpy(sending_msg->buf,buf,buf_len);
    sending_msg->rewrite=0;
    list_add(&sending_msg->list,&sending_list);
    //blue_print("buf_len=%d,sending_msg->len=%d,sending_msg=%p\n",buf_len,sending_msg->buf_len,sending_msg);

    print_list();

    write_size=write(sfd,buf,buf_len);
    write(sfd,buf,buf_len);
    if(write_size<1){
	blue_print("write msg error,write_size=%d\n",write_size);
	goto __exit;
    }

    blue_print("First write success\n");

__exit:
    //serial_gpio_config(false);
    pthread_mutex_unlock(&mutex);
    return buf;
}

serial_rsp_msg_t *serial_parse_msg(unsigned char *buf,int buf_len)
{
    if(buf_len < INDEX_RECV_HEADER_SIZE){
	blue_print("The buf length is too short\n");
	return 0;
    }

    int data_len=buf[INDEX_RECV_DATA_LEN];
    int i=0;

    if( data_len > buf_len ){
	blue_print("The buf length is too long,data_len=%d,buf_len=%d\n",data_len,buf_len);
	goto __resend;
    }

    /*checking the msg that was sent by us*/
    if( buf_len == (data_len+INDEX_SEND_HEADER_SIZE+MSG_TAIL_SIZE)){
	goto __resend;
    }

    serial_rsp_msg_t *data=malloc(sizeof(serial_rsp_msg_t)+data_len);
    data->head1=buf[0];
    data->head2=buf[1];
    data->ret_num1=buf[2];
    data->ret_num2=buf[3];

    if(data->head1!=HEAD_BYTE1_VALUE ||data->head2!=HEAD_BYTE2_VALUE ){
	blue_print("Head Bytes are error,0x%x,0x%x\n",data->head1,data->head2);
	goto __resend;
    }
    if(data->ret_num1!=0x88||data->ret_num2!=0x88){
	blue_print("ret nums are error,0x%x,0x%x\n",data->ret_num1,data->ret_num2);
	goto __resend;
    }

    data->floorid=buf[4];
    data->roomid=buf[5];
    data->cmd_id=buf[6];
    data->data_len=buf[7];
    data->checksum=buf[8+data_len];
    data->tail1=buf[9+data_len];
    data->tail2=buf[10+data_len];
    if(data->tail1!=TAIL_BYTE1_VALUE||data->tail2!=TAIL_BYTE2_VALUE){
	blue_print("tails are error,0x%x,0x%x\n",data->tail1,data->tail2);
	goto __resend;
    }
    if(data_len>0){
	memcpy(data->data,&buf[8],data_len);
    }

    blue_print("serial_rsp_msg: \nfloorid=0x%x\nroom_id=0x%x\ncmd_id=0x%x\ndata_len=%d\n",\
	    data->floorid,data->roomid,data->cmd_id,data->data_len);
    for(i=0;i<data_len;i++){
	blue_print("serial_rsp data[%d]=%x\n",i,data->data[i]);
    }
    return data;
__resend:
    /*if the msg that was not sent by us,resend*/
    if( !list_empty(&sending_list) ){
	serial_resending_msg_t  *msg=get_first_node(&sending_list);
	if((buf_len >= msg->buf_len) &&  !memcmp(msg->buf,buf,buf_len)){
	    rm_first_node(&sending_list);
	    blue_print("send success,no need to resend\n");
	}else{
	    msg->rewrite++;
	    blue_print("not mine msg,send again\n");
	    pthread_cond_signal(&cond);
	}
    }
    return NULL;
}

int serial_gpio_config(char flag)
{
    int fd,bytes_write;

    /*TODO*/
    fd = open("/sys/XXXXX", O_RDWR);
    if(fd==-1){
	blue_print("Open sys error\n");
	return -1;
    }
    if(flag)
	bytes_write=write(fd,"1",1);
    else
	bytes_write=write(fd,"0",1);
    if(bytes_write < 1){
	blue_print("write flag error,bytes_write=%d\n",bytes_write);
	return -1;
    }
    return 0;
}

void *serial_resend(void *addr)
{
    blue_print("Therad init\n");
    while(1)
    {
	pthread_mutex_lock(&mutex_ss);
	pthread_cond_wait(&cond, &mutex_ss);

	srand(time(0));
	int sleep_time=(int)(rand()%500+1);
	usleep(sleep_time*1000);
	serial_resending_msg_t  *msg=get_first_node(&sending_list);
	if(msg->buf_len > BUFFER_SIZE){
	    blue_print("get node error buf_len=%d\n",msg->buf_len);
	    break;
	}
	sbc_color_print(RED,"rewbuf_len=%d,rewrite=%d\n",msg->buf_len,msg->rewrite);

	if(msg->rewrite < 4){
	    int write_size=write(sfd,msg->buf,msg->buf_len);
	    if(write_size>INDEX_SEND_HEADER_SIZE){
		blue_print("resend success,rewrite=%d\n",msg->rewrite);
	    }else
		blue_print("resend msg error,write_size=%d,rewrite=%d\n",write_size,msg->rewrite);
	}

	pthread_mutex_unlock(&mutex_ss);
    }
    return 0;

}

void serial_process_recv_msg()
{
    /*TODO*/
    return ;
}

void *serial_recv(void *addr)
{
    int bytes_read;
    unsigned char buffer[BUFFER_SIZE];
    sleep(2);
    blue_print("Therad init\n");
    int rsfd = open(TESTFILE, O_RDWR|O_CREAT);
    lseek(rsfd, 0, SEEK_SET);

    while(1){
#ifdef FILE_TEST
	bytes_read = read(rsfd,buffer,BUFFER_SIZE);
#else
	bytes_read = read(sfd,buffer,BUFFER_SIZE);
#endif
	if ((bytes_read == -1) && (errno != EINTR)) break;
	else if (bytes_read > 0) {
	    blue_print("serial msg:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
	    serial_rsp_msg_t *rsp_msg = serial_parse_msg(buffer,bytes_read);
	    serial_process_recv_msg(rsp_msg);
	    free(rsp_msg);
	}
    }
    close(rsfd);
    blue_print("Exception,Shut Down\n");
    sleep(1);
    return 0;
}

int open_serial_port(char *dev)
{
    int fd = -1;
    char *pDev[]={"/dev/ttyS0","/dev/ttyS1"};

    if(dev)
	fd = open(dev,O_RDWR|O_NOCTTY|O_NDELAY);
    else
	fd = open(pDev[0],O_RDWR|O_NOCTTY|O_NDELAY);
    if( fd<0 )
    {
	perror("Can't Open Serial Port !");
	return (-1);
    }

    /*reset the serial port as wait status*/
    if( fcntl(fd,F_SETFL,0)<0 ){
	printf("fcntl failed !\n");
	return (-1);
    }else{
	printf("fcntl OK = %d !\n",fcntl(fd,F_SETFL,0));
    }

    /*check the fd is a serial port or not*/
    if( !isatty(STDIN_FILENO) ){
	printf("Standard input isn't a terminal device !\n");
	return (-1);
    }else{
	printf("It's a serial terminal device!\n");
    }

    return fd;
}

int set_serial_port(int fd,int iBaudRate,int iDataSize,char cParity,int iStopBit)
{
    int iResult = 0;
    struct termios oldtio,newtio;

    iResult = tcgetattr(fd,&oldtio);
    if( iResult ){
        perror("Can't get old terminal description !");
        return (-1);
    }

    bzero(&newtio,sizeof(newtio));
    /*ECHO:enable the echo redispaly*/
    newtio.c_cflag |= CLOCAL | CREAD | ECHO;

    switch( iBaudRate )
    {
	case 2400:
	    cfsetispeed(&newtio,B2400);
	    cfsetospeed(&newtio,B2400);
	    break;
	case 4800:
	    cfsetispeed(&newtio,B4800);
	    cfsetospeed(&newtio,B4800);
	    break;
	case 9600:
	    cfsetispeed(&newtio,B9600);
	    cfsetospeed(&newtio,B9600);
	    break;
	case 19200:
	    cfsetispeed(&newtio,B19200);
	    cfsetospeed(&newtio,B19200);
	    break;
	case 38400:
	    cfsetispeed(&newtio,B38400);
	    cfsetospeed(&newtio,B38400);
	    break;
	case 57600:
	    cfsetispeed(&newtio,B57600);
	    cfsetospeed(&newtio,B57600);
	    break;
	case 115200:
	    cfsetispeed(&newtio,B115200);
	    cfsetospeed(&newtio,B115200);
	    break;
	case 460800:
	    cfsetispeed(&newtio,B460800);
	    cfsetospeed(&newtio,B460800);
	    break;
	default  :
	    /*perror("Don't exist iBaudRate !");*/
	    printf("Don't exist iBaudRate %d !\n",iBaudRate);
	    return (-1);
    }

    newtio.c_cflag &= (~CSIZE);
    switch( iDataSize )
    {
	case    7:
	    newtio.c_cflag |= CS7;
	    break;
	case    8:
	    newtio.c_cflag |= CS8;
	    break;
	default:
	    /*perror("Don't exist iDataSize !");*/
	    printf("Don't exist iDataSize %d !\n",iDataSize);
	    return (-1);
    }

    switch( cParity )
    {
	case    'N':                    /*无校验*/
	    newtio.c_cflag &= (~PARENB);
	    break;
	case    'O':                    /*奇校验*/
	    newtio.c_cflag |= PARENB;
	    newtio.c_cflag |= PARODD;
	    newtio.c_iflag |= (INPCK | ISTRIP);
	    break;
	case    'E':                    /*偶校验*/
	    newtio.c_cflag |= PARENB;
	    newtio.c_cflag &= (~PARODD);
	    newtio.c_iflag |= (INPCK | ISTRIP);
	    break;
	default:
	    /*perror("Don't exist cParity  !");*/
	    printf("Don't exist cParity %c !\n",cParity);
	    return (-1);
    }

    switch( iStopBit )
    {
	case    1:
	    newtio.c_cflag &= (~CSTOPB);
	    break;
	case    2:
	    newtio.c_cflag |= CSTOPB;
	    break;
	default:
	    /*perror("Don't exist iStopBit !");*/
	    printf("Don't exist iStopBit %d !\n",iStopBit);
	    return (-1);
    }

    newtio.c_cc[VTIME] = 0; /*setiing wait time*/
    newtio.c_cc[VMIN] = 0;  /*setting minor char*/
    tcflush(fd,TCIFLUSH);       /*refresh input queue (TCIOFLUSH:flush input and output)*/
    iResult = tcsetattr(fd,TCSANOW,&newtio);    /*enable new config*/

    if( iResult )
    {
	perror("Set new terminal description error !");
	return (-1);
    }

    printf("set serial port success !\n");

    return 0;
}

int serial_init(void)
{
    int err=0;
#ifdef FILE_TEST
    sfd = open(TESTFILE, O_RDWR|O_CREAT);
    lseek(sfd, 0, SEEK_SET);
#else
    sfd = open_serial_port("/dev/ttyS0");
    set_serial_port(sfd,115200,8,'N',1);
#endif
    if(sfd==-1){
	blue_print("Open dev error\n");
	return 0;
    }

    usleep(100);
    /*build the serial resend system*/
    err = pthread_create(&p_tid[1], NULL, &serial_resend,  (void*)&ThreadsExit);
    if (err != 0)
    {
	blue_print("\ncan't create serial resend thread :[%s]", strerror(err));
	return 1;
    }
    usleep(100);
    /*build the serial  recv system*/
    err = pthread_create(&p_tid[0], NULL, &serial_recv,  (void*)&ThreadsExit);
    if (err != 0)
    {
	blue_print("\ncan't create serial recv thread :[%s]", strerror(err));
	return 1;
    }

    /*test*/
    send_serial_msg(3,24,0, NULL, 0);
    sleep(4);
    send_serial_msg(3,25,0, NULL, 0);

    return 0;
}

int serial_close(void)
{
    close(sfd);
    return 0;
}
