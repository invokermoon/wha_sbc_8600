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
enum data_type {
    TYPE_HB = 100,
    TYPE_DATA=101,
};

struct sb_data{
    enum data_type data_type;
    char data[100];
};



typedef void (*sighandler_t)(int);
#define BUFLEN sizeof(struct sb_data)
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
#if 0
        _retry:
            /******发送消息*******/
            bzero(buf,BUFLEN);
            printf("请输入发送给对方的消息：");
            /*fgets函数：从流中读取BUFLEN-1个字符*/
            fgets(buf,BUFLEN,stdin);
            /*打印发送的消息*/
            //fputs(buf,stdout);
            if(!strncasecmp(buf,"quit",4)){
                printf("server 请求终止聊天!\n");
                break;
            }
            /*如果输入的字符串只有"\n"，即回车，那么请重新输入*/
            if(!strncmp(buf,"\n",1)){
                printf("输入的字符只有回车，这个是不正确的！！！\n");
                goto _retry;
            }
            /*如果buf中含有'\n'，那么要用strlen(buf)-1，去掉'\n'*/
            if(strchr(buf,'\n'))
                len = send(newfd,buf,strlen(buf)-1,0);
            /*如果buf中没有'\n'，则用buf的真正长度strlen(buf)*/
            else
                len = send(newfd,buf,strlen(buf),0);
            if(len > 0)
                printf("消息发送成功，本次共发送的字节数是：%d\n",len);
            else{
                printf("消息发送失败!\n");
                break;
            }
#endif
            /******接收消息*******/
            bzero(buf,BUFLEN);
            len = recv(newfd,buf,BUFLEN,0);
            if(len > 0){
		struct sb_data *data=(struct sb_data*)buf;
		if(data->data_type==TYPE_HB)
		    printf("HB：%s,共有字节数是: %d\n",data->data,len);
		else
		    printf("客户端发来的信息是：%s,共有字节数是: %d\n",data->data,len);
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
