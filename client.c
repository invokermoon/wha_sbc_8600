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

/*color for print*/
#define NONE         "\033[m"
#define RED          "\033[0;32;31m"
#define LIGHT_RED    "\033[1;31m"
#define GREEN        "\033[0;32;32m"
#define LIGHT_GREEN  "\033[1;32m"
#define BLUE         "\033[0;32;34m"
#define LIGHT_BLUE   "\033[1;34m"
#define DARY_GRAY    "\033[1;30m"
#define CYAN         "\033[0;36m"
#define LIGHT_CYAN   "\033[1;36m"
#define PURPLE       "\033[0;35m"
#define LIGHT_PURPLE "\033[1;35m"
#define BROWN        "\033[0;33m"
#define YELLOW       "\033[1;33m"
#define LIGHT_GRAY   "\033[0;37m"
#define WHITE        "\033[1;37m"
#define sbc_print(fmt,...) do {  printf(GREEN"[%d][%s]:"NONE fmt,getpid(),__func__,##__VA_ARGS__) ;} while(0)
#define sbc_color_print(color,fmt,...) do {  printf(GREEN"[%s]:"color fmt NONE,__func__,##__VA_ARGS__) ;} while(0)

/*IP and port*/
#define IP_ADDR	"106.14.60.75"
#define IP_PORT	8668


int ThreadsExit=1;
pthread_t p_tid[2];

typedef void (*sighandler_t)(int);

int sockfd;
#define BUFLEN 300

#if 0
/*Msg header sent to server*/
#define DEVICE_MSG_HEAD		"01"
#define DEVICE_HB_HEAD		"88"
/*msg header recved*/
#define SEVER_MSG_HEAD		"61"
#endif

/*
 * interface type:clinet --->server
 * */
#define ITYPE_COMMITINFO 	"01"
#define ITYPE_SCAN		"02"
#define ITYPE_PAIR_OK		"03"
#define ITYPE_HEARTBEAT		"88"

/*interface type:server-->client*/
#define ITYPE_SETSTATUS	"61"

/*Mac*/
#define MAC_ADDRESS	"11:22:33:44:55:66"

#define MSG_ID		"10000001|"
#define VERSION		"01|"
#define DEVICE		"ABCDEFGHIJKL|"
/*Rsp status*/
#define STATUS_OK 		"00"
#define STATUS_FORMAT_FAIL	"71"
#define STATUS_MSG_INCOMPLETE	"72"
#define STATUS_INSIDE_ERROR	"73"

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

/*Common msg header*/
typedef struct msg_header{
    char id[8+1];
    char itype[2+1];
    char length[4+1];
    char version[2+1];
    char own[10+1];
    char device[12+1];
    char data[0];
}msg_header_t;
/*Common msg rsp*/
typedef struct msg_rsp_s{
    char status[2+1];
    char error[5];
}msg_rsp_t;

/*Msg struct :client to server*/
typedef struct commit_msg_s{
    char bleMac[17];
}commit_msg_t;

typedef struct hb_msg_s{
    char bleMac[17];
}hb_msg_t;

typedef struct scan_msg_s{
    char gwMac[1+1];
    char version[1+1];
    char cmdType[1+1];
    char bleMac[1+1];
    char phoneMac[1+1];
    char timestamp[1];
}scan_msg_t;

typedef struct pair_msg_s{
    char roomid[1+1];
    char cmdType[1+1];
    char bleMac[1+1];
    char phoneMac[1+1];
    char timestamp[1];
}pair_msg_t;

/*Msg struct :server to client*/
typedef struct device_status_s{
    char roomid[1+1];
    char status[1];
}device_status_t;


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

    unsigned len = data_len + sizeof(struct msg_header);
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
    sbc_print("Complete msg:\n%s\nlen=%d\n",(char *)header,len);
    pthread_mutex_unlock(&mutex);
    return (char*)header;
}

int process_device_status(device_status_t *data)
{
    return 0;

}

int sending_status_error(void *buf,char *status,char *error)
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

int parse_msg(char *msg)
{
    pthread_mutex_lock(&mutex1);

    int i=0;
    if(msg==NULL){
	sbc_print("msg is null\n");
	return -1;
    }
    size_t len=strlen(msg);
    //sbc_print("Get msg is:\n%s\nlen=%zd\n",msg,len);
    if(len<sizeof(msg_header_t)){
	sbc_print("msg size is error\n");
	sleep(2);
	sending_status_error(msg,STATUS_MSG_INCOMPLETE,"error");
	return -1;
    }
    char *tmp_buf=malloc(len);
    memcpy(tmp_buf,msg,len);
    msg_header_t *header=(msg_header_t*)tmp_buf;
    for(i=0;i<len;i++){
	if(tmp_buf[i]=='|'){
	    tmp_buf[i]='\0';
	}
    }

    if(strncmp(header->id,MSG_ID,strlen(header->id)-1)!=0){
	sbc_print("Id is error:%s\n",header->id);
	sleep(2);
	sending_status_error(msg,STATUS_FORMAT_FAIL,"error");
	return -1;
    }
    if(strncmp(header->version,VERSION,strlen(header->version)-1)!=0){
	sbc_print("version is error:%s\n",header->version);
	sleep(2);
	sending_status_error(msg,STATUS_FORMAT_FAIL,"error");
	return -1;
    }
    if(strncmp(header->device,DEVICE,strlen(header->device)-1)!=0){
	sbc_print("device is error:%s\n",header->device);
	sleep(2);
	sending_status_error(msg,STATUS_FORMAT_FAIL,"error");
	return -1;
    }
    if(atoi(header->length) != len){
	sending_status_error(msg,STATUS_MSG_INCOMPLETE,"error");
	sleep(2);
	sbc_print("length is error:%s\n",header->length);
	return -1;
    }

   if(strncmp(header->itype, ITYPE_COMMITINFO,strlen(ITYPE_COMMITINFO))==0){
	sbc_print("Handle the commit rsp\n");
    }

    if(strncmp(header->itype, ITYPE_SCAN,strlen(ITYPE_SCAN))==0){
	sbc_print("Handle the scan rsp\n");
    }

    if(strncmp(header->itype, ITYPE_PAIR_OK,strlen(ITYPE_PAIR_OK))==0){
	sbc_print("Handle the pair rsp\n");
    }
    if(strncmp(header->itype, ITYPE_HEARTBEAT,strlen(ITYPE_HEARTBEAT))==0){
	sbc_print("Handle the hb rsp\n");
    }

    if(strncmp(header->itype, ITYPE_SETSTATUS,strlen(ITYPE_SETSTATUS))==0){
	device_status_t *data=(device_status_t *)header->data;
	sbc_print("roomid:%s\n",data->roomid);
	sbc_print("status:%s\n",data->status);
	process_device_status(data);
    }
    free(header);
    pthread_mutex_unlock(&mutex1);
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

void *recv_handler(void *addr)
{
    while(1){
	char buf[BUFLEN];
	/******接收消息*******/
	bzero(buf,BUFLEN);
	int len = recv(sockfd,buf,BUFLEN,0);
	if(len > 0){
	    sbc_print("\n服务器发来的消息是：\n%s\n,共有字节数是: %d\n",buf,len);
	    parse_msg(buf);
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
    socklen_t len;
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
	char *data;
	if(strncmp(buf,"commit",5)==0){
	    commit_msg_t data_buf={
		.bleMac=MAC_ADDRESS,
	    };
	    data=(char *)make_send_msg(ITYPE_COMMITINFO,&data_buf,sizeof(commit_msg_t));
	}
	else if(strncmp("scan",buf,4)==0){
	    scan_msg_t data_buf={
		.gwMac="1|",
		.version="1|",
		.cmdType="1|",
		.bleMac="1|",
		.phoneMac="1|",
		.timestamp="1",
	    };
	    data=(char *)make_send_msg(ITYPE_SCAN,&data_buf,sizeof(scan_msg_t));
	}
	else if(strncmp("pairok",buf,6)==0){
	    pair_msg_t data_buf={
		.roomid="1|",
		.cmdType="1|",
		.bleMac="1|",
		.phoneMac="1|",
		.timestamp="1",
	    };
	    data=(char *)make_send_msg(ITYPE_PAIR_OK,&data_buf,sizeof(pair_msg_t));
	}else{
            sbc_print("Invalid cmd!\n");
	    goto _retry;
	}

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
    }
    /*关闭连接*/
    close(sockfd);

    return 0;
}
