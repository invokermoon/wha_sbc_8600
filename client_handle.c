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

int cs_scan(void *buf,unsigned int length)
{
    cs_scan_msg_t data_buf={
	.Mac=MAC_ADDRESS__,
	.roomid="9999|",
	.bleMac=BLE_MAC_ADDRESS__,
	.phoneMac=PHONE_MAC_ADDRESS__,
	.timestamp="YYYY/MM/DD HH:MM:SS",
    };
    /*we need ignore the "\0" of timestamp */
    char *data=(char *)make_send_msg(ITYPE_CS_SCAN,&data_buf,sizeof(cs_scan_msg_t)-1);

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

int cs_commit(void *buf,unsigned int length)
{
    cs_commit_msg_t data_buf={
	.Mac="12345678",
    };
    char *data=(char *)make_send_msg(ITYPE_CS_COMMITINFO,&data_buf,sizeof(cs_commit_msg_t)-1);
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
int cs_pair(void *buf,unsigned int length)
{
    cs_pair_msg_t data_buf={
	.roomid="9999|",
	.Mac=MAC_ADDRESS__,
	.bleMac=BLE_MAC_ADDRESS__,
	.phoneMac=PHONE_MAC_ADDRESS__,
	.timestamp="YYYY/MM/DD HH:MM:SS",
    };
    char *data=(char *)make_send_msg(ITYPE_CS_PAIR_OK,&data_buf,sizeof(cs_pair_msg_t)-1);
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

int sc_setdev(msg_header_t *buf,unsigned int len)
{
    sc_dev_status_t *data=(sc_dev_status_t *)buf->data;
    sbc_print("roomid:%s\n",data->roomid);
    sbc_print("status:%s\n",data->status);
    sending_response(buf,STATUS_FORMAT_FAIL,"error");
    return 0;
}

int sc_rmdev(msg_header_t *buf,unsigned int len)
{
    return 0;
}

int sc_bt_restore(msg_header_t *buf,unsigned int len)
{
    return 0;
}

int sc_bt_backup(msg_header_t *buf,unsigned int len)
{
    return 0;
}

int sc_bt_query(msg_header_t *buf,unsigned int len)
{
    return 0;
}


int commit_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(cs_commit_msg_t));
    sbc_print("Handle the commit rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int scan_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(cs_scan_msg_t));
    sbc_print("Handle the scan rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int pair_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(cs_pair_msg_t));
    sbc_print("Handle the pair rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int hb_rsp(msg_header_t *buf,unsigned int len)
{
    msg_rsp_t *rsp=(msg_rsp_t*)(buf->data+sizeof(cs_hb_msg_t));
    cs_hb_msg_t *hb=(cs_hb_msg_t*)(buf->data);
    sbc_print("Handle the hb rsp status=%s,error=%s,hb->Mac=%s\n",rsp->status,rsp->error,hb->Mac);
    return 0;
}

struct handler_driver SBC8600= {
	.sname = "SBC",
	.funcs = {
	    .cs_scan=&cs_scan,
	    .cs_commit=&cs_commit,
	    .cs_pair=&cs_pair,
	    .cs_hb=NULL,

	    .sc_setdev=&sc_setdev,
	    .sc_rmdev=&sc_rmdev,
	    .sc_bt_query=&sc_bt_query,
	    .sc_bt_restore=&sc_bt_restore,
	    .sc_bt_backup=&sc_bt_backup,

	    .sc_scan_rsp=&scan_rsp,
	    .sc_commit_rsp=&commit_rsp,
	    .sc_pair_rsp=&pair_rsp,
	    .sc_hb_rsp=&hb_rsp,
	}
};

define_drvinit(SBC8600);

