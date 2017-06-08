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
#include "client.h"

int send_scan(void *buf,unsigned int length)
{
    scan_msg_t data_buf={
	.gwMac="1|",
	.version="1|",
	.cmdType="1|",
	.bleMac="1|",
	.phoneMac="1|",
	.timestamp="1",
    };
    char *data=(char *)make_send_msg(ITYPE_SCAN,&data_buf,sizeof(scan_msg_t));

    unsigned int wsize=strlen(data);
    socklen_t len = send(sockfd,data,wsize,0);
    if(len > 0)
	sbc_print("Send success：%d\n",len);
    else{
	sbc_print("Send fail!\n");
	sleep(1);
    }

    free(data);

    return 0;
}

int send_commit(void *buf,unsigned int length)
{
    commit_msg_t data_buf={
	.bleMac=MAC_ADDRESS,
    };
    char *data=(char *)make_send_msg(ITYPE_COMMITINFO,&data_buf,sizeof(commit_msg_t));
    unsigned int wsize=strlen(data);
    socklen_t len = send(sockfd,data,wsize,0);
    if(len > 0)
	sbc_print("Send success：%d\n",len);
    else{
	sbc_print("Send fail!\n");
	sleep(1);
    }

    free(data);
    return 0;
}
int send_pair(void *buf,unsigned int length)
{
    pair_msg_t data_buf={
	.roomid="1|",
	.cmdType="1|",
	.bleMac="1|",
	.phoneMac="1|",
	.timestamp="1",
    };
    char *data=(char *)make_send_msg(ITYPE_PAIR_OK,&data_buf,sizeof(pair_msg_t));
    unsigned int wsize=strlen(data);
    socklen_t len = send(sockfd,data,wsize,0);
    if(len > 0)
	sbc_print("Send success：%d\n",len);
    else{
	sbc_print("Send fail!\n");
	sleep(1);
    }


    free(data);

    return 0;
}

#if 0
int send_heatbeat(void *buf,unsigned int len)
{
    return 0;
}
#endif

int recv_setting(msg_header_t *buf,unsigned int len)
{
    device_status_t *data=(device_status_t *)buf->data;
    sbc_print("roomid:%s\n",data->roomid);
    sbc_print("status:%s\n",data->status);
    sending_response(buf,STATUS_FORMAT_FAIL,"error");
    return 0;
}

int commit_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(commit_msg_t)+1);
    sbc_print("Handle the commit rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int scan_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(scan_msg_t)+1);
    sbc_print("Handle the scan rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int pair_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(pair_msg_t)+1);
    sbc_print("Handle the pair rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int hb_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(hb_msg_t)+1);
    sbc_print("Handle the hb rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

struct handler_driver SBC8600= {
	.sname = "SBC",
	.funcs = {
	    .send_scan=&send_scan,
	    .send_commit=&send_commit,
	    .send_pair=&send_pair,
	    .send_hb=NULL,
	    .recv_setting=&recv_setting,
	    .recv_scan_rsp=&scan_rsp,
	    .recv_commit_rsp=&commit_rsp,
	    .recv_pair_rsp=&pair_rsp,
	    .recv_hb_rsp=&hb_rsp,
	}
};

define_drvinit(SBC8600);

