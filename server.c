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
#define sbc_print(fmt,...) do {  printf(GREEN"[%s]:"NONE fmt,__func__,##__VA_ARGS__) ;} while(0)
#define sbc_color_print(color,fmt,...) do {  printf(GREEN"[%s]:"color fmt NONE,__func__,##__VA_ARGS__) ;} while(0)

/*Msg header sent to server*/
#define DEVICE_MSG_HEAD		"01"
#define DEVICE_HB_HEAD		"88"
/*msg header recved*/
#define SEVER_MSG_HEAD		"61"


typedef void (*sighandler_t)(int);
#define BUFLEN 200
int sockfd, newfd;
void sig_pipe(int signo);

void sig_pipe(int signo)
{
    printf("catch a signal...\n");
    sleep(2);
    if(signo == SIGTSTP){
	close(sockfd);
	close(newfd);
    }
    exit(-1);
}

char *make_msg(char *header,char *roomid,char *status)
{
    char *msg=malloc(BUFLEN);
    char slen[5];
    int i=0;
    unsigned int len=6+strlen(roomid)+strlen(status)+strlen("|")*2;
    memset(msg,0,BUFLEN);
    if(header==NULL||roomid==NULL||status==NULL){
	sbc_print("parameters is error\n");
	sleep(2);
	return NULL;
    }
    memcpy(msg,header,strlen(header));
    sprintf(slen,"%4d|",len);
    for(i=0;i<4;i++){
	char *p=strstr(slen," ");
	if(p!=NULL){
	    memcpy(p,"0",1);
	}
    }

    strcat(msg,slen);
    strcat(msg,roomid);
    strcat(msg,"|");
    strcat(msg,status);
    sbc_print("Complete msg=%s,len=%d\n",msg,len);
    sleep(5);
    return msg;
}

int main(int argc, char **argv)
{
    struct sockaddr_in s_addr, c_addr;
    char buf[BUFLEN];
    socklen_t len;
    unsigned int port, listnum;

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
    /*设置侦听队列长度*/
    if(argv[3])
        listnum = atoi(argv[3]);
    else
        listnum = 3;
    /*设置服务器ip*/
    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(port);
    if(argv[1])
        s_addr.sin_addr.s_addr = inet_addr(argv[1]);
    else
        s_addr.sin_addr.s_addr = INADDR_ANY;
    /*把地址和端口帮定到套接字上*/
    if((bind(sockfd, (struct sockaddr*) &s_addr,sizeof(struct sockaddr))) == -1){
        perror("bind");
        exit(errno);
    }else
        printf("bind success!\n");
    /*侦听本地端口*/
    if(listen(sockfd,listnum) == -1){
        perror("listen");
        exit(errno);
    }else
        printf("the server is listening!\n");
    while(1){
        printf("*****************聊天开始***************\n");
        len = sizeof(struct sockaddr);
        if((newfd = accept(sockfd,(struct sockaddr*) &c_addr, &len)) == -1){
            perror("accept");
            exit(errno);
        }else
            printf("正在与您聊天的客户端是：%s: %d\n",inet_ntoa(c_addr.sin_addr),ntohs(c_addr.sin_port));
        while(1){
            /******接收消息*******/
            bzero(buf,BUFLEN);
            len = recv(newfd,buf,BUFLEN,0);
            if(len > 0){
		    printf("\n客户端发来的信息是：\n%s,共有字节数是: %d\n",buf,len);
		    //char *msg = (char *)make_msg(SEVER_MSG_HEAD,"8888","01");
		    //send(newfd,msg,strlen(msg),0);
	    }
            else{
                if(len < 0 )
                    printf("接受消息失败！\n");
                else
                    printf("客户端退出了，聊天终止！\n");
                break;
            }
        }
        /*关闭聊天的套接字*/
        close(newfd);
        /*是否退出服务器*/
        printf("服务器是否退出程序：y->是；n->否? ");
        bzero(buf, BUFLEN);
        fgets(buf,BUFLEN, stdin);
        if(!strncasecmp(buf,"y",1)){
            printf("server 退出!\n");
            break;
        }
    }
    /*关闭服务器的套接字*/
    close(sockfd);
    return 0;
}
