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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <termios.h>
#include "cJSON.h"
#include "client.h"

int ThreadsExit=1;
pthread_t p_tid[3];

/*Global list head*/
LIST_HEAD(message_list);


typedef void (*sighandler_t)(int);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

void sig_pipe(int signo);

void sig_pipe(int signo)
{
    sbc_print("catch a signal...\n");
    sleep(3);
    if(signo == SIGTSTP){
	close(sockfd);
    }
    exit(-1);
}

char *system_timestamp()
{
    struct tm *t;
    time_t tt;
    static char time_buf[20];
    memset(time_buf,0,20);
    time(&tt);
    t = localtime(&tt);

    sprintf(time_buf,"%4d/%02d/%02d %02d:%02d:%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    return time_buf;
}

char *make_send_msg(char *itype,void *data, unsigned int data_len)
{
    pthread_mutex_lock(&mutex);
    int i =0 ;
    char slen[6];
    char iitype[4]={0};
    if(data==NULL||data==0){
	sbc_print("data is null,make msg fail\n");
	return NULL;
    }

    /*ignore the '\0' in the end*/
    if( ((char*)data)[data_len-1] == 0 ){
	data_len-=1;
    }

    unsigned len = data_len + sizeof(struct msg_header) + strlen(MSG_TAIL_STRING);

    msg_header_t *header=malloc(len+1);
    memset(header,0,len+1);

    memcpy(iitype,itype,2);
    memcpy(iitype+2,"|",1);
    sbc_print("data_len=%d,iitype=%s\n",data_len,iitype);

    sprintf(slen,"%04d|",len);

    memcpy(header->head,MSG_HEADER_STRING,sizeof(header->head));
    memcpy(header->id,MSG_ID,sizeof(header->id));
    memcpy(header->itype,iitype,sizeof(header->itype));
    memcpy(header->version,VERSION,sizeof(header->version));
    memcpy(header->own,"1111111111|",sizeof(header->own));
    memcpy(header->device,DEVICE,sizeof(header->device));
    memcpy(header->length,slen,sizeof(header->length));

    /*cpy data*/
    if((data!=NULL)&&(data_len!=0)){
	memcpy(header->data,(char*)data,data_len);
    }

    /*check if there are '\0' in the msg*/
    for(i=0;i<len-1;i++){
	if( ((char*)header)[i] == 0 ){
	    ((char*)header)[i] = '$';
	}
    }

    /*add the tail mask*/
    memcpy(header->data+data_len,MSG_TAIL_STRING,strlen(MSG_TAIL_STRING));

    sbc_print("Complete msg:\n%s\nlen=%d\n",(char *)header,len);
    pthread_mutex_unlock(&mutex);
    return (char*)header;
}

int send_message(char *itype,void *data, unsigned int data_len)
{
    char *msg=(char *)make_send_msg(itype,data,data_len);

    unsigned int wsize=strlen(msg);
    socklen_t len = send(sockfd,msg,wsize,0);

    free(msg);
    if(len > 0)
	sbc_print("Send success：%d\n",len);
    else{
	sbc_print("Send fail!\n");
	sleep(1);
	return -1;
    }
    return 0;
}

int process_device_status(sc_dev_status_t *data)
{
    return 0;

}

/*use | instead of \0*/
void add_strings_splitsymbol(char *buf,unsigned int buf_len)
{
    int i=0;
    if(buf){
	/*ignore the last char*/
	for(i=0;i<(buf_len-3);i++){
	    if( buf[i]== 0 ) buf[i]='|';
	}
    }
}

/*use \0 instead of |*/
void remove_strings_splitsymbol(char *buf,unsigned int buf_len)
{
    int i=0;
    if(buf){
	/*ignore the last char*/
	for(i=0;i<buf_len;i++){
	    if( buf[i]=='|') buf[i]=0;
	    else if( buf[i]=='<') buf[i]=0;
	    else if( buf[i]=='>') buf[i]=0;
	}
    }
}

int send_response(char *msg_buf,unsigned int buf_len, char *status,char *error)
{
    msg_header_t *buf=(msg_header_t *)msg_buf;
    unsigned int error_len=strlen(error);
    unsigned int len=0;

    char slen[8];
    if(error){
	error_len=strlen(error);
    }

    /*ensure that the symbol is vaild*/
    memcpy(buf->head,MSG_HEADER_STRING,sizeof(buf->head));
    add_strings_splitsymbol((char*)buf,buf_len);

    char *end_p=strstr((char*)buf,">>");
    if( end_p ){
	/*If this buf have the ">>" */
	len=buf_len+strlen("|")+sizeof(msg_rsp_t)+error_len;
	/*remove the end symbol to connect others strings*/
	memset(end_p,0,2);
    }else
	/*If this buf doesn't have the >>*/
	len=buf_len+strlen("|")+sizeof(msg_rsp_t)+error_len+strlen(MSG_TAIL_STRING);

    sprintf(slen,"%04d|",len);

    char buff[len+2];
    memset(buff,0,len+2);
    if(buf==NULL){
	sbc_print("buf is NULL,error\n");
	return 0;
    }

    memcpy(buff,buf,buf_len);
    /*change the length*/
    char *len_p=strstr(buff,"|");
    if(len_p){
	len_p=strstr(len_p+1,"|");
	if(len_p)
	    memcpy(len_p+1,slen,4);
    }

    strcat(buff,"|");
    strcat(buff,status);
    strcat(buff,"|");
    strcat(buff,error);
    strcat(buff,MSG_TAIL_STRING);
    sbc_print("rsp msg is: \n%s\nlen=%d\n",buff,len);
    send(sockfd,buff,len,0);
    return 0;
}

void print_header_struct(msg_header_t *header)
{
    sbc_color_print(DARY_GRAY,"Id is :%s\n",header->id);
    sbc_color_print(DARY_GRAY,"version is :%s\n",header->version);
    sbc_color_print(DARY_GRAY,"len is :%s\n",header->length);
    sbc_color_print(DARY_GRAY,"itype is :%s\n",header->itype);
    sbc_color_print(DARY_GRAY,"own is :%s\n",header->own);
    sbc_color_print(DARY_GRAY,"device is :%s\n",header->device);
}

int parse_one_message(char *msg)
{
    if(msg==NULL){
	sbc_print("msg is null\n");
	return -1;
    }
    size_t len=strlen(msg);
    sbc_print("Get One Msg is:\n%s\nlen=%zd\n",msg,len);
    if(len<sizeof(msg_header_t)){
	sbc_print("message size is error\n");
	sleep(1);
	return -1;
    }
    char *tmp_buf=malloc(len+1);
    memset(tmp_buf,0,len+1);
    memcpy(tmp_buf,msg,len);

    msg_header_t *header=(msg_header_t*)tmp_buf;

    /*remove all the | and << and >>*/
    remove_strings_splitsymbol((char*)header,len);

    //print_header_struct(header);

    if(strncmp(header->id,MSG_ID,strlen(header->id)-1)!=0){
	sbc_print("Id is error:%s\n",header->id);
	return -1;
    }
    if(strncmp(header->version,VERSION,strlen(header->version)-1)!=0){
	sbc_print("version is error:%s\n",header->version);
	return -1;
    }

    if(strncmp(header->device,DEVICE,strlen(header->device)-1)!=0){
	sbc_print("device is error:%s\n",header->device);
	return -1;
    }

    if(atoi(header->length) != len){
	//send_response(msg,STATUS_MSG_INCOMPLETE,"error");
	sbc_print("length is error:%s\n",header->length);
	return -1;
    }

    if(strncmp(header->itype, ITYPE_CS_COMMITINFO,strlen(ITYPE_CS_COMMITINFO))==0){
	handler->funcs.sc_commit_rsp(header,len);
    }else if(strncmp(header->itype, ITYPE_CS_SCAN,strlen(ITYPE_CS_SCAN))==0){
	handler->funcs.sc_scan_rsp(header,len);
    }else if(strncmp(header->itype, ITYPE_CS_PAIR_OK,strlen(ITYPE_CS_PAIR_OK))==0){
	handler->funcs.sc_pair_rsp(header,len);
    }else if(strncmp(header->itype, ITYPE_CS_HEARTBEAT,strlen(ITYPE_CS_HEARTBEAT))==0){
	handler->funcs.sc_hb_rsp(header,len);
    }else if(strncmp(header->itype, ITYPE_SC_SETDEV,strlen(ITYPE_SC_SETDEV))==0){
	handler->funcs.sc_setdev(header,len);
    }else if(strncmp(header->itype, ITYPE_SC_RMDEV,strlen(ITYPE_SC_RMDEV))==0){
	handler->funcs.sc_rmdev(header,len);
    }else if(strncmp(header->itype, ITYPE_SC_BT_RESTORE,strlen(ITYPE_SC_BT_RESTORE))==0){
	handler->funcs.sc_bt_restore(header,len);
    }else if(strncmp(header->itype, ITYPE_SC_BT_BACKUP,strlen(ITYPE_SC_BT_BACKUP))==0){
	handler->funcs.sc_bt_backup(header,len);
    }else if(strncmp(header->itype, ITYPE_SC_BT_QUERY,strlen(ITYPE_SC_BT_QUERY))==0){
	handler->funcs.sc_bt_query(header,len);
    }else{
	sbc_print("Invalid cmd type =%s\n",header->itype);
    }
    free(header);
    return 0;
}

void *send_hb(void *addr)
{
    sbc_print("Thread init\n");
    sleep(5);
    cs_hb_msg_t data_buf={
	.Mac=MAC_ADDRESS,
    };
    char *msg = (char *)make_send_msg(ITYPE_CS_HEARTBEAT,&data_buf,sizeof(cs_hb_msg_t));
    unsigned int len=strlen(msg);
    while(1){
	send(sockfd,msg,len,0);
	sleep(30);
    }
    free(msg);
    return NULL;
}

static struct message_node_s *find_invalid_node()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&message_list){
	struct message_node_s *node = list_entry(plist,struct message_node_s,list);
	//sbc_print("Read list =%s\n",node->one_msg);
	if(node && node->valid==0){
	    return node;
	}
    }
    return NULL;
}

static void print_message_list()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&message_list){
	struct message_node_s *node = list_entry(plist,struct message_node_s,list);
	sbc_print("Read list =%s,valid=%d\n",node->one_msg,node->valid);
    }
}

int parse_message(char *buf)
{
    pthread_mutex_lock(&mutex1);

    unsigned int buf_len;
    char *spos;
    char *hpos;
    char *tpos;
__parse:
    buf_len=strlen(buf);
    spos=buf;
    hpos = strstr(buf,MSG_HEADER_STRING);
    tpos = strstr(buf,MSG_TAIL_STRING);
    /*There must be a point that if there is only  "<<", it must be in the first 2 bytes
     * Just 4 situations:
     * 1:<<XXXXXXX>> or <<XXXXXXX>><<XXXXXXX****
     * 2:XXXXXXX or XXXXXX>>
     * 3:<<XXXXXXX
     * 4:>><<XXXXXXX or XXXX>><<XXXX******
     * */
    //sbc_print("hpos=0x%p,tpos=0x%p,spos=0x%p,buf_len=%d\n",hpos,tpos,spos,buf_len);
    if(hpos && tpos && (hpos < tpos)){
	struct message_node_s *p=malloc(sizeof(message_node_t));
	memset(p,0,sizeof(message_node_t));
	memcpy(p->one_msg,hpos,tpos-hpos+2);
	p->valid=1;
	list_add(&p->list, &message_list);

	if(buf_len > (tpos-hpos+2)){
	    buf=tpos+2;
	    goto __parse;
	}

    }else if(!hpos){
	struct message_node_s *p=find_invalid_node();
	if( (strlen(p->one_msg) + buf_len) > BUFLEN ){
	    sbc_color_print(RED,"message len is too long, discard it\n");
	    list_del_init(&(p->list));
	    return 0;
	}
	if(p && !tpos){
	    memcpy(p->one_msg+strlen(p->one_msg),spos,buf_len);
	    p->valid=0;
	}else if(p && tpos){
	    memcpy(p->one_msg+strlen(p->one_msg),spos,buf_len);
	    p->valid=1;
	}


    }else if( hpos && !tpos){
	struct message_node_s *p=malloc(sizeof(message_node_t));
	memset(p,0,sizeof(message_node_t));
	memcpy(p->one_msg,hpos,buf_len);
	p->valid=0;
	list_add(&p->list, &message_list);

    }else if(hpos && tpos && tpos < hpos){
	struct message_node_s *p=find_invalid_node();
	if(p){
	    memcpy(p->one_msg+strlen(p->one_msg),spos,tpos-spos+2);
	    p->valid=1;

	    /*transfer to situation 3*/
	    buf=hpos;
	    goto __parse;
	}
    }

    //print_message_list();

    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&message_list){
	struct message_node_s *node = list_entry(plist,struct message_node_s,list);
	//sbc_print("Read list =%s,valid=%d\n",node->one_msg,node->valid);
	if(node->valid){
	    parse_one_message(node->one_msg);
	    list_del_init(plist);
	    free(node);
	}
    }

    //print_message_list();

    pthread_mutex_unlock(&mutex1);
    return 0;
}


void *recv_handler(void *addr)
{
    sbc_print("Thread init\n");
    while(1){
	char buf[BUFLEN];
	bzero(buf,BUFLEN);
	int len = recv(sockfd,buf,BUFLEN,0);
	if(len > 0){
	    //sbc_print("\n服务器发来的消息是：\n%s\n,共有字节数是: %d\n",buf,len);
	    parse_message(buf);
	}
	else{
	    if(len < 0 )
		sbc_print("接受消息失败！\n");
	    else
		sbc_print("服务器退出了，聊天终止！\n");
	    sleep(3);
	    break;
	}
    }
    exit(-1);
}

int main(int argc, char **argv)
{
    struct sockaddr_in s_addr;
    //socklen_t len;
    unsigned int port;
    char buf[BUFLEN];
    char ip_addr[20];

    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
	perror("socket");
	exit(errno);
    }else
	sbc_print("socket create success!\n");

    sighandler_t ret;
    ret = signal(SIGTSTP,sig_pipe);
    if(ret<0){
	sbc_print("Signal connect error\n");
    }

    memset(ip_addr,0,sizeof(ip_addr));
    if(argc==3){
	if(argv[2])
	    port = atoi(argv[2]);
	else
	    port = IP_PORT;
	if(argv[1])
	    memcpy(ip_addr,argv[1],strlen(argv[1]));
    }else{
	port = IP_PORT;
	memcpy(ip_addr,IP_ADDR,strlen(IP_ADDR));
    }
    sbc_print("IP=%s,port=%d\n",ip_addr,port);

    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(port);
    if (inet_aton(ip_addr, (struct in_addr *)&s_addr.sin_addr.s_addr) == 0) {
	perror(ip_addr);
	exit(errno);
    }
    if(connect(sockfd,(struct sockaddr*)&s_addr,sizeof(struct sockaddr)) == -1){
	perror("connect");
	exit(errno);
    }else
	sbc_print("conncet success!\n");

    /*build the heartbeat system*/
    int err = pthread_create(&p_tid[0], NULL, &send_hb,  (void*)&ThreadsExit);
    if (err != 0)
    {
	sbc_print("\ncan't create heartbeat thread :[%s]", strerror(err));
	return 1;
    }

    usleep(100);

    /*build the recv handler system*/
    err = pthread_create(&p_tid[1], NULL, &recv_handler,  (void*)&ThreadsExit);
    if (err != 0)
    {
	sbc_print("\ncan't create recv thread :[%s]", strerror(err));
	return 1;
    }

    serial_init();

    usleep(100);
    /*commit ASAP*/
    handler->funcs.cs_commit(NULL,0);

    usleep(1000);
    while(1){
_retry:
	bzero(buf,BUFLEN);
	sbc_print("请输入发送给对方的消息：\n");

	fgets(buf,BUFLEN,stdin);
	//fputs(buf,stdout);
	if(!strncasecmp(buf,"quit",4)){
	    sbc_print("client 请求终止聊天!\n");
	    break;
	}
	if(!strncmp(buf,"\n",1)){
	    sbc_print("输入的字符只有回车，这个是不正确的！！！\n");
	    goto _retry;
	}
#if 1
	if(strchr(buf,'\n')){
	    //memcpy(sbdata->jsdata,buf,strlen(buf)-1);
	    buf[strlen(buf)-1]=0;
	}
#endif
	if(strncmp(buf,"commit",5)==0){
	    handler->funcs.cs_commit(NULL,0);
	}
	else if(strncmp("scan",buf,4)==0){
	    handler->funcs.cs_scan(NULL,0);
	}
	else if(strncmp("pairok",buf,6)==0){
	    handler->funcs.cs_pair(NULL,0);
	}
	else if(strncmp("serialcheck",buf,10)==0){
	    send_serial_msg(3,24,0, NULL, 0);
	}
	else if(strncmp("serialpair",buf,4)==0){
	    send_serial_msg(3,24,2, NULL, 0);
	}else{
	    sbc_print("Invalid cmd!\n");
	    goto _retry;
	}
    }
    close(sockfd);

    return 0;
}
