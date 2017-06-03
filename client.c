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

int ThreadsExit=1;
pthread_t p_tid[2];

typedef void (*sighandler_t)(int);

int sockfd;
#define BUFLEN sizeof(struct sb_data) 
void sig_pipe(int signo);

void sig_pipe(int signo)
{
    printf("catch a signal...\n");
    sleep(3);
    if(signo == SIGTSTP){
	close(sockfd);
    }
    exit(-1);
}

enum data_type {
    TYPE_HB=100,
    TYPE_DATA=101,
};

struct sb_data{
    enum data_type data_type;
    char data[200];
};

void *send_hb(void *addr)
{
    struct sb_data pd={
	.data_type=TYPE_HB,
    };
    memcpy(pd.data,"111",4);
    while(1){
	//write(sockfd,pd,sizeof(DATA_PACK));
        send(sockfd,&pd,sizeof(struct sb_data),0);
	sleep(5); //定时3秒
    }
    return NULL;
}

void *recv_handler(void *addr)
{
    while(1){
	char buf[BUFLEN];
	/******接收消息*******/
	bzero(buf,BUFLEN);
	int len = recv(sockfd,buf,BUFLEN,0);
	if(len > 0)
	    printf("服务器发来的消息是：%s,共有字节数是: %d\n",buf,len);
	else{
	    if(len < 0 )
		printf("接受消息失败！\n");
	    else
		printf("服务器退出了，聊天终止！\n");
	    sleep(3);
	    break;
	}
    }
    exit(-1);
}

static char * makeJson(void)
{
    cJSON *pJsonRoot = NULL;
    cJSON *pSubJson = NULL;
    char *p = NULL;

    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
	printf("%s line=%d NULL\n", __func__, __LINE__);
	return NULL;
    }
    cJSON_AddStringToObject(pJsonRoot, "hello", "hello world");
    cJSON_AddNumberToObject(pJsonRoot, "number", 10010);
    cJSON_AddBoolToObject(pJsonRoot, "bool", 1);
    pSubJson = cJSON_CreateObject();
    if(NULL == pSubJson)
    {
	printf("%s line=%d NULL\n", __func__, __LINE__);
	cJSON_Delete(pJsonRoot);
	return NULL;
    }
    cJSON_AddStringToObject(pSubJson, "subjsonobj", "a sub json string");
    cJSON_AddItemToObject(pJsonRoot, "subobj", pSubJson);

    p = cJSON_Print(pJsonRoot);
    printf("strlen= %d\n", strlen(p));

    if(NULL == p)
    {
	printf("%s line=%d NULL\n", __func__, __LINE__);
	cJSON_Delete(pJsonRoot);
	return NULL;
    }

    printf("p = \n%s\n\n", p);

    cJSON_Delete(pJsonRoot);

    return p;
}

int main(int argc, char **argv)
{
    struct sockaddr_in s_addr;
    socklen_t len;
    unsigned int port;
    char buf[BUFLEN];

    /*建立socket*/
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
        perror("socket");
        exit(errno);
    }else
        printf("socket create success!\n");

    sighandler_t ret;
    ret = signal(SIGTSTP,sig_pipe);
    if(ret<0){
	printf("Signal connect error\n");
    }

    /*设置服务器端口*/
    if(argv[2])
        port = atoi(argv[2]);
    else
        port = 4567;
    /*设置服务器ip*/
    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(port);
    if (inet_aton(argv[1], (struct in_addr *)&s_addr.sin_addr.s_addr) == 0) {
        perror(argv[1]);
        exit(errno);
    }
    /*开始连接服务器*/
    if(connect(sockfd,(struct sockaddr*)&s_addr,sizeof(struct sockaddr)) == -1){
        perror("connect");
        exit(errno);
    }else
        printf("conncet success!\n");

    /*build the heartbeat system*/
    int err = pthread_create(&p_tid[0], NULL, &send_hb,  (void*)&ThreadsExit);
    if (err != 0)
    {
	printf("\ncan't create heartbeat thread :[%s]", strerror(err));
	return 1;
    }
    else
    {
	printf("\n Begin heartbeat send Thread\n");
    }

    /*build the heartbeat system*/
    err = pthread_create(&p_tid[1], NULL, &recv_handler,  (void*)&ThreadsExit);
    if (err != 0)
    {
	printf("\ncan't create recv thread :[%s]", strerror(err));
	return 1;
    }
    else
    {
	printf("\n Begin recv send Thread\n");
    }




    while(1){
    _retry:
        /******发送消息*******/
        bzero(buf,BUFLEN);
        printf("请输入发送给对方的消息：");
        /*fgets函数：从流中读取BUFLEN-1个字符*/
        fgets(buf,BUFLEN,stdin);
        /*打印发送的消息*/
        //fputs(buf,stdout);
        if(!strncasecmp(buf,"quit",4)){
            printf("client 请求终止聊天!\n");
            break;
        }
        /*如果输入的字符串只有"\n"，即回车，那么请重新输入*/
        if(!strncmp(buf,"\n",1)){
            printf("输入的字符只有回车，这个是不正确的！！！\n");
            goto _retry;
        }
	struct sb_data sbdata={
	    .data_type=TYPE_DATA,
	};
        /*如果buf中含有'\n'，那么要用strlen(buf)-1，去掉'\n'*/
        if(strchr(buf,'\n')){
	    memcpy(sbdata.data,buf,strlen(buf)-1);
	}
        /*如果buf中没有'\n'，则用buf的真正长度strlen(buf)*/
        else{
	    memcpy(sbdata.data,buf,strlen(buf));
	}
	unsigned int wsize=sizeof(int)+strlen(sbdata.data);
	//len = send(sockfd,&sbdata,sizeof(struct sb_data),0);
	len = send(sockfd,&sbdata,wsize,0);
	if(len > 0)
            printf("消息发送成功，本次共发送的字节数是：%d\n",len);
        else{
            printf("消息发送失败!\n");
            break;
        }
    }
    /*关闭连接*/
    close(sockfd);

    return 0;
}
