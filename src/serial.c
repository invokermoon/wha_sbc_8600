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
#include "serial.h"

//#define TESTFILE	"/dev/ttyS0"
#define SERIAL_PORT	"/dev/ttyO1"

int sfd=0;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_ss = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_parse = PTHREAD_MUTEX_INITIALIZER;


static pthread_cond_t cond=PTHREAD_COND_INITIALIZER;
static int ThreadsExit=1;
pthread_t p_tid[2];

LIST_HEAD(serial_send_list);
LIST_HEAD(serial_recv_list);

enum which_bytes_first_e {
    BYTES_NONE_FIRST=0,
    BYTES_HEAD_FIRST=1,
    BYTES_TAIL_FIRST=2,
};

static serial_send_node_t *get_first_send_node(struct list_head *head)
{
    if( (head->next != NULL) && ( !list_empty(head) ) ){
	return list_entry(head->next,struct serial_send_node_s,list);
    }
    return NULL;
}

static void rm_first_send_node(struct list_head *head)
{
    if( head->next != NULL ){
	struct serial_send_node_s *node = list_entry(head->next,struct serial_send_node_s,list);
	node->rewrite=0;
	node->buf_len=0;
	list_del_init(head->next);
	free(node);
    }
}

void __attribute__((unused)) print_send_list()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&serial_send_list){
	struct serial_send_node_s *node = list_entry(plist,struct serial_send_node_s,list);
	sbc_color_print(RED,"List entry:p=%p, buf_len=%d,rewrite=%d\n",node, node->buf_len,node->rewrite);
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
unsigned char *serial_send_message(char floorid,\
			char roomid, \
			serial_cmd_type_t cmd_id, \
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
    //TODO
    checksum=0;

    int tmp_len=INDEX_SEND_HEADER_SIZE+data_len;
    buf[tmp_len]=checksum;
    buf[tmp_len+1]=TAIL_BYTE1_VALUE;
    buf[tmp_len+2]=TAIL_BYTE2_VALUE;

    for(i=0;i<buf_len;i++){
	serial_print("buf[%d]=0x%x\n",i,buf[i]);
    }

    /*back up the sending msg into list*/
    serial_send_node_t *sending_msg=malloc(sizeof(serial_send_node_t));
    sending_msg->buf_len=buf_len;
    memcpy(sending_msg->buf,buf,buf_len);
    sending_msg->rewrite=0;
    list_add(&sending_msg->list,&serial_send_list);

    sbc_print(DEBUG_DBG,"buf_len=%d,sending_msg->len=%d,sending_msg=%p\n",buf_len,sending_msg->buf_len,sending_msg);

    print_send_list();

    write_size=write(sfd,buf,buf_len);

    if(write_size<1){
	serial_print("write msg error,write_size=%d\n",write_size);
	goto __exit;
    }

    serial_print("First write success,write_size=%d\n",write_size);

__exit:
    //serial_gpio_config(false);
    pthread_mutex_unlock(&mutex);
    return buf;
}

serial_recv_message_t *serial_parse_one_message(unsigned char *buf,int buf_len)
{
    int i=0;
    if(buf_len < INDEX_RECV_HEADER_SIZE){
	serial_print("The buf length is too short\n");
	return 0;
    }

#if 1
    printf(RED"[%s]",__func__);
    for(i=0;i<buf_len;i++){
	printf(BLUE"bytes[%d]=%x ",i,buf[i]);
    }
    printf(NONE"\n");
#endif

    int data_len=buf[INDEX_RECV_DATA_LEN];

    if( data_len > buf_len ){
	serial_print("Length is error,goto resend data_len=%d,buf_len=%d\n",data_len,buf_len);
	goto __resend;
    }

    /*checking the msg that was sent by us*/
    if( buf_len == (data_len+INDEX_SEND_HEADER_SIZE+MSG_TAIL_SIZE)){
	goto __resend;
    }

    serial_recv_message_t *data=malloc(sizeof(serial_recv_message_t)+data_len);
    data->head1=buf[0];
    data->head2=buf[1];
    data->ret_num1=buf[2];
    data->ret_num2=buf[3];

    if(data->head1!=HEAD_BYTE1_VALUE ||data->head2!=HEAD_BYTE2_VALUE ){
	serial_print("Head Bytes are error,goto resend 0x%x,0x%x\n",data->head1,data->head2);
	goto __resend;
    }
    if(data->ret_num1!=0x88||data->ret_num2!=0x88){
	serial_print("ret nums are error,goto resend 0x%x,0x%x\n",data->ret_num1,data->ret_num2);
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
	serial_print("tails are error,goto resend 0x%x,0x%x\n",data->tail1,data->tail2);
	goto __resend;
    }
    if(data_len>0){
	memcpy(data->data,&buf[8],data_len);
    }

    sbc_print(DEBUG_DBG,"serial_rsp_msg: \nfloorid=0x%x\nroom_id=0x%x\ncmd_id=0x%x\ndata_len=%d\n",\
	    data->floorid,data->roomid,data->cmd_id,data->data_len);
    for(i=0;i<data_len;i++){
	serial_print("serial_rsp data[%d]=%x\n",i,data->data[i]);
    }
    return data;
__resend:
    /*if the msg that was not sent by us,resend*/
    if( !list_empty(&serial_send_list) ){
	serial_send_node_t  *msg=get_first_send_node(&serial_send_list);
	if((buf_len >= msg->buf_len) &&  !memcmp(msg->buf,buf,buf_len)){
	    rm_first_send_node(&serial_send_list);
	    serial_print("[Resend]send success,no need to resend\n");
	}else{
	    msg->rewrite++;
	    serial_print("[Resend]not mine msg,send again\n");
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
	serial_print("Open sys error\n");
	return -1;
    }
    if(flag)
	bytes_write=write(fd,"1",1);
    else
	bytes_write=write(fd,"0",1);
    if(bytes_write < 1){
	serial_print("write flag error,bytes_write=%d\n",bytes_write);
	return -1;
    }
    return 0;
}

void *serial_resend(void *addr)
{
    serial_print("Therad init\n");
    while(1)
    {
	pthread_mutex_lock(&mutex_ss);
	pthread_cond_wait(&cond, &mutex_ss);

	srand(time(0));
	int sleep_time=(int)(rand()%500+1);
	usleep(sleep_time*1000);
	serial_send_node_t  *msg=get_first_send_node(&serial_send_list);
	if(msg->buf_len > BUFFER_SIZE){
	    serial_print("get node error buf_len=%d\n",msg->buf_len);
	    break;
	}
	serial_print("rewbuf_len=%d,rewrite=%d\n",msg->buf_len,msg->rewrite);

	if(msg->rewrite < 4){
	    int write_size=write(sfd,msg->buf,msg->buf_len);
	    if(write_size>INDEX_SEND_HEADER_SIZE){
		serial_print("resend success,rewrite=%d\n",msg->rewrite);
	    }else
		serial_print("resend msg error,write_size=%d,rewrite=%d\n",write_size,msg->rewrite);
	}

	pthread_mutex_unlock(&mutex_ss);
    }
    return 0;

}

void serial_process_recv_msg(serial_recv_message_t *rsp_msg)
{
    /*TODO*/
    if (rsp_msg->cmd_id == 3){
	char room[6];
	sprintf(room,"%02d%02d|",rsp_msg->floorid,rsp_msg->roomid);
	handler->funcs.cs_pair(room,strlen(room));
    }

    return ;
}

static struct serial_recv_node_s *find_invalid_node()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&serial_recv_list){
	struct serial_recv_node_s *node = list_entry(plist,struct serial_recv_node_s,list);
	//serial_print("Read list =%s\n",node->one_msg);
	if(node && node->valid==0){
	    return node;
	}
    }
    return NULL;
}

static void __attribute__((unused)) print_recv_list()
{
    struct list_head *plist,*pnode;
    int i=0;
    serial_print("Print List\n");
    list_for_each_safe(plist,pnode,&serial_recv_list){
	struct serial_recv_node_s *node = list_entry(plist,struct serial_recv_node_s,list);
	//serial_print("line=%d,plist=%p,node->list=%p,plist->prev=%p,plist->next=%p\n",__LINE__,plist,&node->list,plist->prev,plist->next);
	if(node){
	    serial_print("valid=%d,valid_len=%d:",node->valid,node->valid_len);
	    for(i=0;i<node->valid_len;i++) printf("[%d]=0x%x ",i,node->one_msg[i]);
	    printf("\n");
	}
    }
}

unsigned char *find_headbytes_pointer(unsigned char *buf,int buf_len)
{
    unsigned char *hpos=NULL;
    int i=0;
    for(i=0;i<buf_len-1;i++){
	if( (buf[i] == HEAD_BYTE1_VALUE) && (buf[i+1] == HEAD_BYTE2_VALUE) ){
	    hpos = &buf[i];
	    return hpos;
	}
    }
    return NULL;
}

unsigned char *find_tailbytes_pointer(unsigned char *buf,int buf_len)
{
    unsigned char *tpos=NULL;
    int i=0;
    for(i=0;i<buf_len-1;i++){
	if( (buf[i] == TAIL_BYTE1_VALUE ) && (buf[i+1] == TAIL_BYTE2_VALUE) ){
	    tpos = &buf[i];
	    return tpos;
	}
    }
    return NULL;
}

static enum which_bytes_first_e head_tail_which_first(unsigned char *buf,int buf_len)
{
    int i=0;
    for(i=0;i<buf_len-1;i++){
	if( (buf[i] == HEAD_BYTE1_VALUE) && (buf[i+1] == HEAD_BYTE2_VALUE) ){
	    return BYTES_HEAD_FIRST;
	}
	if( (buf[i] == TAIL_BYTE1_VALUE ) && (buf[i+1] == TAIL_BYTE2_VALUE) ){
	    return BYTES_TAIL_FIRST;
	}
    }
    return BYTES_NONE_FIRST;
}


int serial_parse_messages(unsigned char *buf,int unparsed_buf_len)
{
    pthread_mutex_lock(&mutex_parse);

    unsigned char *spos=NULL;
    unsigned char *hpos=NULL;
    unsigned char *tpos=NULL;
    int used_len = 0;

__parse:
    spos=buf;
    hpos=find_headbytes_pointer(buf,unparsed_buf_len);
    tpos=find_tailbytes_pointer(buf,unparsed_buf_len);
    enum which_bytes_first_e which_1st=head_tail_which_first(buf,unparsed_buf_len);

    struct serial_recv_node_s *invaild_node=find_invalid_node();

    //serial_print("hpos=0x%p,tpos=0x%p,spos=0x%p,unparsed_buf_len=%d\n",hpos,tpos,spos,unparsed_buf_len);
    if( which_1st == BYTES_HEAD_FIRST  ){
	if(invaild_node){
	    /*remove old rubish data*/
	    list_del_init(&(invaild_node->list));
	}
	struct serial_recv_node_s *p=malloc(sizeof(serial_recv_node_t));
	memset(p,0,sizeof(serial_recv_node_t));
	if( tpos ){
	    used_len=tpos-hpos+2;
	    memcpy(p->one_msg,hpos,used_len);
	    p->valid=1;
	    p->valid_len=used_len;
	    list_add(&(p->list), &serial_recv_list);

	    if( unparsed_buf_len > used_len ){
		buf=tpos+2;
		unparsed_buf_len -=used_len;
		goto __parse;
	    }
	}else{
	    int useless_size=hpos-spos;
	    used_len=unparsed_buf_len-useless_size;
	    memcpy(p->one_msg,hpos,used_len);
	    p->valid=0;
	    p->valid_len=used_len;
	    list_add(&(p->list), &serial_recv_list);
	}
    }else if( which_1st == BYTES_NONE_FIRST){
	if(invaild_node){
	    if( ( invaild_node->valid_len + unparsed_buf_len) > BUFLEN ){
		serial_print("message len is too long, discard it\n");
		list_del_init(&(invaild_node->list));
		return 0;
	    }
	    used_len = unparsed_buf_len;
	    memcpy(&(invaild_node->one_msg[invaild_node->valid_len]),spos,used_len);
	    invaild_node->valid=0;
	    invaild_node->valid_len+=used_len;
	}else{
	    /*Do nothing,just ignore it*/
	}

    }else if( which_1st == BYTES_TAIL_FIRST ){
	used_len = tpos-spos+2;
	if(invaild_node){
	    memcpy(&(invaild_node->one_msg[invaild_node->valid_len]),spos,used_len);
	    invaild_node->valid_len+=used_len;
	    invaild_node->valid=1;
	}
	/*transfer to other situation*/
	if( unparsed_buf_len > used_len ){
	    buf=tpos+2;
	    unparsed_buf_len-=used_len;
	    goto __parse;
	}

    }

    //print_recv_list();

    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&serial_recv_list){
	struct serial_recv_node_s *node = list_entry(plist,struct serial_recv_node_s,list);
	//serial_print("Read list =%s,valid=%d\n",node->one_msg,node->valid);
	if(node && node->valid){
	    serial_recv_message_t *rsp_msg = serial_parse_one_message(node->one_msg,node->valid_len);
	    if(rsp_msg){
		serial_process_recv_msg(rsp_msg);
		free(rsp_msg);
	    }
	    list_del_init(plist);
	    free(node);
	}
    }

    //print_recv_list();

    pthread_mutex_unlock(&mutex_parse);
    return 0;
}


void *serial_recv(void *addr)
{
    int bytes_read;
    int i=0;
    unsigned char buffer[BUFFER_SIZE];
    sleep(2);
    serial_print("Therad init\n");
    int rsfd = open(SERIAL_PORT, O_RDWR|O_CREAT);
    lseek(rsfd, 0, SEEK_SET);

    while(1){
#ifdef FILE_TEST
	bytes_read = read(rsfd,buffer,BUFFER_SIZE);
#else
	bytes_read = read(sfd,buffer,BUFFER_SIZE);
#endif
	if ((bytes_read == -1) && (errno != EINTR)){
	    serial_print("Reading none,break\n");
	    break;
	}else if (bytes_read > 0) {
	    for(i=0;i<bytes_read;i++) serial_print("serial msg:bytes_read[%d]=0x%x\n",i,buffer[i]);
	    serial_parse_messages(buffer,bytes_read);
	}
    }
    close(rsfd);
    serial_print("Exception,Shut Down\n");
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
    if( fd<0 ){
	serial_print("Can't Open Serial Port !\n");
	return (-1);
    }else
	serial_print("Open serial port %s succuss\n",dev);

    /*reset the serial port as wait status*/
    if( fcntl(fd,F_SETFL,0)<0 ){
	serial_print("fcntl failed !\n");
	return (-1);
    }else{
	serial_print("fcntl OK = %d !\n",fcntl(fd,F_SETFL,0));
    }

    /*check the fd is a serial port or not*/
    if( !isatty(STDIN_FILENO) ){
	serial_print("Standard input isn't a terminal device !\n");
	return (-1);
    }else{
	serial_print("It's a serial terminal device!\n");
    }

    return fd;
}

int set_serial_port(int fd,int iBaudRate,int iDataSize,char cParity,int iStopBit)
{
    int iResult = 0;
    struct termios oldtio,newtio;

    iResult = tcgetattr(fd,&oldtio);
    if( iResult ){
        serial_print("Can't get old terminal description !\n");
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
	    serial_print("Don't exist iBaudRate %d !\n",iBaudRate);
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
	    serial_print("Don't exist iDataSize %d !\n",iDataSize);
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
	    serial_print("Don't exist cParity %c !\n",cParity);
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
	    serial_print("Don't exist iStopBit %d !\n",iStopBit);
	    return (-1);
    }

    newtio.c_cc[VTIME] = 0; /*setiing wait time*/
    newtio.c_cc[VMIN] = 0;  /*setting minor char*/
    tcflush(fd,TCIFLUSH);       /*refresh input queue (TCIOFLUSH:flush input and output)*/
    iResult = tcsetattr(fd,TCSANOW,&newtio);    /*enable new config*/

    if( iResult )
    {
	serial_print("Set new terminal description error !");
	return (-1);
    }

    serial_print("set serial port success !\n");
    return 0;
}

int serial_init(void)
{
    int err=0;
#ifdef FILE_TEST
    sfd = open(SERIAL_PORT, O_RDWR|O_CREAT);
    lseek(sfd, 0, SEEK_SET);
    serial_print("FILETEST OPEN...\n");
#else
    sfd = open_serial_port(SERIAL_PORT);
    //set_serial_port(sfd,115200,8,'N',1);
    set_serial_port(sfd,9600,8,'N',1);
#endif
    if(sfd==-1){
	serial_print("Open dev error\n");
	return 0;
    }

    usleep(100);
    /*build the serial resend system*/
    err = pthread_create(&p_tid[1], NULL, &serial_resend,  (void*)&ThreadsExit);
    if (err != 0)
    {
	serial_print("\ncan't create serial resend thread :[%s]", strerror(err));
	return 1;
    }
    usleep(100);
    /*build the serial  recv system*/
    err = pthread_create(&p_tid[0], NULL, &serial_recv,  (void*)&ThreadsExit);
    if (err != 0)
    {
	serial_print("\ncan't create serial recv thread :[%s]", strerror(err));
	return 1;
    }
    return 0;
}

int serial_close(void)
{
    close(sfd);
    return 0;
}
