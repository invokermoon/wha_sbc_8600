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
pthread_t p_tid[2];

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

char *make_send_msg(char *itype,void *data, unsigned int data_len)
{
    pthread_mutex_lock(&mutex);
    char slen[5];
    char iitype[4]={0};
    int i=0;
    if(data==NULL||data==0){
	sbc_print("data is null,make msg fail\n");
	return NULL;
    }

    unsigned len = data_len + sizeof(struct msg_header) + strlen(MSG_TAIL_STRING);

    msg_header_t *header=malloc(len+1);
    memset(header,0,len+1);

    memcpy(iitype,itype,2);
    memcpy(iitype+2,"|",1);
    sbc_print("data_len=%d,iitype=%s\n",data_len,iitype);

    sprintf(slen,"%4d|",len);
    for(i=0;i<4;i++){
	char *p=strstr(slen," ");
	if(p!=NULL){
	    memcpy(p,"0",1);
	}
    }

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
    /*add the tail mask*/
    memcpy(header->data+data_len,MSG_TAIL_STRING,strlen(MSG_TAIL_STRING));

    sbc_print("Complete msg:\n%s\nlen=%d\n",(char *)header,len);
    pthread_mutex_unlock(&mutex);
    return (char*)header;
}

int process_device_status(device_status_t *data)
{
    return 0;

}

int sending_response(void *buf,char *status,char *error)
{
    unsigned int buf_len=strlen(buf);
    unsigned int len=buf_len+strlen("|")+sizeof(msg_rsp_t);
    char buff[len+2];
    memset(buff,0,len+2);
    if(buf==NULL){
	sbc_print("buf is NULL,error\n");
	return 0;
    }
    memcpy(buff,buf,strlen(buf));
    strcat(buff,"|");
    strcat(buff,status);
    strcat(buff,"|");
    strcat(buff,error);
    sbc_print("rsp msg is: \n%s\n",buff);
    send(sockfd,buff,len,0);
    return 0;
}

int parse_one_message(char *msg)
{
    int i=0;
    if(msg==NULL){
	sbc_print("msg is null\n");
	return -1;
    }
    size_t len=strlen(msg);
    sbc_print("Get one msg is:\n%s\nlen=%zd\n",msg,len);
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
    for(i=0;i<len;i++){
	if(tmp_buf[i]=='|'){
	    tmp_buf[i]='\0';
	}else if(tmp_buf[i]=='<'){
	    tmp_buf[i]='\0';
	}else if(tmp_buf[i]=='>'){
	    tmp_buf[i]=0;
	}
    }

    if(strncmp(header->id,MSG_ID,strlen(header->id)-1)!=0){
	sbc_print("Id is error:%s\n",header->id);
	//sending_response(msg,STATUS_FORMAT_FAIL,"error");
	return -1;
    }
    if(strncmp(header->version,VERSION,strlen(header->version)-1)!=0){
	sbc_print("version is error:%s\n",header->version);
	//sending_response(msg,STATUS_FORMAT_FAIL,"error");
	return -1;
    }
    if(strncmp(header->device,DEVICE,strlen(header->device)-1)!=0){
	//sbc_print("device is error:%s\n",header->device);
	sending_response(msg,STATUS_FORMAT_FAIL,"error");
	return -1;
    }

    if(atoi(header->length) != len){
	//sending_response(msg,STATUS_MSG_INCOMPLETE,"error");
	sbc_print("length is error:%s\n",header->length);
	return -1;
    }

   if(strncmp(header->itype, ITYPE_COMMITINFO,strlen(ITYPE_COMMITINFO))==0){
	handler->funcs.recv_commit_rsp(header,0);
    }else if(strncmp(header->itype, ITYPE_SCAN,strlen(ITYPE_SCAN))==0){
	handler->funcs.recv_scan_rsp(header,0);
    }else if(strncmp(header->itype, ITYPE_PAIR_OK,strlen(ITYPE_PAIR_OK))==0){
	handler->funcs.recv_pair_rsp(header,0);
    }else if(strncmp(header->itype, ITYPE_HEARTBEAT,strlen(ITYPE_HEARTBEAT))==0){
	handler->funcs.recv_hb_rsp(header,0);
    }else if(strncmp(header->itype, ITYPE_SETSTATUS,strlen(ITYPE_SETSTATUS))==0){
	handler->funcs.recv_setting(header,0);
    }else{
	sbc_print("Invalid cmd type =%s\n",header->itype);
    }
    free(header);
    return 0;
}

void *send_hb(void *addr)
{
    hb_msg_t data_buf={
	.bleMac=MAC_ADDRESS,
    };
    char *msg = (char *)make_send_msg(ITYPE_HEARTBEAT,&data_buf,sizeof(hb_msg_t));
    unsigned int len=strlen(msg);
    while(1){
	//write(sockfd,pd,sizeof(DATA_PACK));
        send(sockfd,msg,len,0);
	sleep(30);
    }
    free(msg);
    return NULL;
}

struct message_s *find_invalid_message()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&message_list){
	struct message_s *node = list_entry(plist,struct message_s,list);    
	sbc_print("Read list =%s\n",node->one_msg);
	if(node->valid==0){
	    return node;
	}
    }
    return NULL;
}

void print_message_list()
{
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&message_list){
	struct message_s *node = list_entry(plist,struct message_s,list);    
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
    /*There is must a point that if there is only  "<<", it must be in the first 2 bytes
     * Just 4 situations:
     * 1:<<XXXXXXX>> or <<XXXXXXX>><<XXXXXXX****
     * 2:XXXXXXX
     * 3:<<XXXXXXX
     * 4:>><<XXXXXXX or XXXX>><<XXXX******
     * */
    sbc_print("hpos=0x%p,tpos=0x%p,spos=0x%p,buf_len=%d\n",hpos,tpos,spos,buf_len);
    if(hpos && tpos && (hpos < tpos)){
	struct message_s *p=malloc(sizeof(message_t));
	memset(p,0,sizeof(message_t));
	memcpy(p->one_msg,hpos,tpos-hpos+2);
	p->valid=1;
	list_add(&p->list, &message_list);

	if(buf_len > (tpos-hpos+2)){
	    buf=tpos+2;
	    goto __parse;
	}

    }else if(!hpos && !tpos){
	struct message_s *p=find_invalid_message();
	if(p){
	    memcpy(p->one_msg+strlen(p->one_msg),spos,buf_len);
	    p->valid=0;
	}

    }else if( hpos && !tpos){
	struct message_s *p=malloc(sizeof(message_t));
	memset(p,0,sizeof(message_t));
	memcpy(p->one_msg,hpos,buf_len);
	p->valid=0;
	list_add(&p->list, &message_list);

    }else if(hpos && tpos && tpos < hpos){
	struct message_s *p=find_invalid_message();
	if(p){
	    memcpy(p->one_msg+strlen(p->one_msg),spos,tpos-spos+2);
	    p->valid=1;

	    /*transfer to situation 3*/
	    buf=hpos;
	    goto __parse;
	}
    }

    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&message_list){
	struct message_s *node = list_entry(plist,struct message_s,list);    
	sbc_print("Read list =%s,valid=%d\n",node->one_msg,node->valid);
	if(node->valid){
	    parse_one_message(node->one_msg);
	    list_del_init(plist);
	    free(node);
	}
    }

    pthread_mutex_unlock(&mutex1);
    return 0;
}






void *recv_handler(void *addr)
{
    while(1){
	char buf[BUFLEN];
	/******接收消息*******/
	bzero(buf,BUFLEN);
	int len = recv(sockfd,buf,BUFLEN,0);
	if(len > 0){
	    sbc_print("\n服务器发来的消息是：\n%s\n,共有字节数是: %d\n",buf,len);
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
/*{{{*/
#if 0
static char * makeJson(void)
{
    cJSON *pJsonRoot = NULL;
    cJSON *pSubJson = NULL;
    cJSON *pSubJsonStatus = NULL;
    cJSON *pSubJsonData = NULL;
    char *p = NULL;

    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
	sbc_print("%s line=%d NULL\n", __func__, __LINE__);
	return NULL;
    }
    cJSON_AddStringToObject(pJsonRoot, "key", "key");
    cJSON_AddStringToObject(pJsonRoot, "timestamp", "12345");
    cJSON_AddStringToObject(pJsonRoot, "sign", "sign");
    cJSON_AddStringToObject(pJsonRoot, "requestParams", "789");
    //cJSON_AddNumberToObject(pJsonRoot, "number", 10010);
    //cJSON_AddBoolToObject(pJsonRoot, "bool", 1);
    pSubJson = cJSON_CreateObject();
    cJSON_AddItemToObject(pJsonRoot, "body", pSubJson);

    pSubJsonStatus = cJSON_CreateObject();
    pSubJsonData = cJSON_CreateObject();
    if(NULL == pSubJsonStatus||NULL==pSubJsonData)
    {
	sbc_print("%s line=%d NULL\n", __func__, __LINE__);
	cJSON_Delete(pJsonRoot);
	return NULL;
    }
    cJSON_AddStringToObject(pSubJsonStatus, "timestatmp", "1234567");
    cJSON_AddStringToObject(pSubJsonStatus, "gwMac", "1.1.1.1");
    cJSON_AddStringToObject(pSubJsonStatus, "version", "version1");
    cJSON_AddStringToObject(pSubJsonData, "cmdType", "type1");
    cJSON_AddStringToObject(pSubJsonData, "bleMac", "2.2.2.2");
    cJSON_AddStringToObject(pSubJsonData, "phoneMac", "2.3.3.3");
    cJSON_AddStringToObject(pSubJsonData, "time", "999999");
    cJSON_AddItemToObject(pSubJson, "status", pSubJsonStatus);
    cJSON_AddItemToObject(pSubJson, "Data", pSubJsonData);
    cJSON_AddStringToObject(pSubJson, "dataType", "datatype1");

    p = cJSON_Print(pJsonRoot);
    sbc_print("strlen= %d\n", strlen(p));

    if(NULL == p)
    {
	sbc_print("%s line=%d NULL\n", __func__, __LINE__);
	cJSON_Delete(pJsonRoot);
	return NULL;
    }

    sbc_print("p = \n%s\n\n", p);

    cJSON_Delete(pJsonRoot);

    return p;
}
#endif/*}}}*/

int main(int argc, char **argv)
{
    struct sockaddr_in s_addr;
    //socklen_t len;
    unsigned int port;
    char buf[BUFLEN];
    char ip_addr[20];

    /*建立socket*/
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
	/*设置服务器端口*/
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
    /*设置服务器ip*/
    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(port);
    if (inet_aton(ip_addr, (struct in_addr *)&s_addr.sin_addr.s_addr) == 0) {
	perror(ip_addr);
	exit(errno);
	}
    /*开始连接服务器*/
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
    else
    {
	sbc_print("Begin heartbeat send Thread\n");
    }

    /*build the heartbeat system*/
    err = pthread_create(&p_tid[1], NULL, &recv_handler,  (void*)&ThreadsExit);
    if (err != 0)
    {
	sbc_print("\ncan't create recv thread :[%s]", strerror(err));
	return 1;
    }
    else
    {
	sbc_print("Begin recv Thread\n");
    }


    while(1){
    _retry:
	sleep(1);
        /******发送消息*******/
        bzero(buf,BUFLEN);
        sbc_print("请输入发送给对方的消息：\n");
        /*fgets函数：从流中读取BUFLEN-1个字符*/
        fgets(buf,BUFLEN,stdin);
        /*打印发送的消息*/
        //fputs(buf,stdout);
        if(!strncasecmp(buf,"quit",4)){
            sbc_print("client 请求终止聊天!\n");
            break;
        }
        /*如果输入的字符串只有"\n"，即回车，那么请重新输入*/
        if(!strncmp(buf,"\n",1)){
            sbc_print("输入的字符只有回车，这个是不正确的！！！\n");
            goto _retry;
        }
#if 1
        /*如果buf中含有'\n'，那么要用strlen(buf)-1，去掉'\n'*/
        if(strchr(buf,'\n')){
	    //memcpy(sbdata->jsdata,buf,strlen(buf)-1);
	    buf[strlen(buf)-1]=0;
	}
#endif
	if(strncmp(buf,"commit",5)==0){
	    handler->funcs.send_commit(NULL,0);
	}
	else if(strncmp("scan",buf,4)==0){
	    handler->funcs.send_scan(NULL,0);
	}
	else if(strncmp("pairok",buf,6)==0){
	    handler->funcs.send_pair(NULL,0);
	}else{
	    sbc_print("Invalid cmd!\n");
	    goto _retry;
	}

#if 0/*{{{*/
	//char *data;
	unsigned int wsize=strlen(data);
	//len = send(sockfd,&sbdata,sizeof(struct sbc_msg),0);
	len = send(sockfd,data,wsize,0);
	free(data);
	if(len > 0)
            sbc_print("消息发送成功，本次共发送的字节数是：%d\n",len);
        else{
            sbc_print("消息发送失败!\n");
	    sleep(5);
            break;
        }
#endif/*}}}*/
    }
    close(sockfd);

    return 0;
}
