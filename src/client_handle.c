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
#include "serial.h"

int cs_scan(void *buf,unsigned int length)
{
    cs_scan_msg_t data_buf={
	.Mac=MAC_ADDRESS__,
	.roomid="9999|",
	.bleMac=BLE_MAC_ADDRESS__,
	.phoneMac=PHONE_MAC_ADDRESS__,
    };
    memcpy(data_buf.timestamp,system_timestamp(),sizeof(data_buf.timestamp));
    /*we need ignore the "\0" of timestamp */
    socket_send_message(ITYPE_CS_SCAN,&data_buf,sizeof(cs_scan_msg_t)-1);

    return 0;
}

int cs_commit(void *buf,unsigned int length)
{
    cs_commit_msg_t data_buf={
	.Mac="12345678",
    };
    socket_send_message(ITYPE_CS_COMMITINFO,&data_buf,sizeof(cs_commit_msg_t)-1);
    return 0;
}

int cs_pair(void *buf,unsigned int length)
{
    cs_pair_msg_t data_buf={
	.roomid="9999|",
	.Mac=MAC_ADDRESS__,
	.bleMac=BLE_MAC_ADDRESS__,
	.phoneMac=PHONE_MAC_ADDRESS__,
    };
    memcpy(data_buf.timestamp,system_timestamp(),sizeof(data_buf.timestamp));
    socket_send_message(ITYPE_CS_PAIR_OK,&data_buf,sizeof(cs_pair_msg_t)-1);

    return 0;
}

int sc_setdev(socket_message_header_t *buf,unsigned int len)
{
    sc_dev_status_t *data=(sc_dev_status_t *)buf->data;
    socket_print("roomid:%s\n",data->roomid);
    socket_print("status:%s\n",data->status);
    if(strncmp(data->status, DEVICE_STATUS_READYPAIR,strlen(DEVICE_STATUS_READYPAIR))==0){
        char floorid=atoi(data->roomid)/100;
        char roomid=atoi(data->roomid)%100;
        serial_send_message(floorid,roomid,2, NULL, 0);
    }

    socket_send_response((char*)buf,len,STATUS_OK,"0");

    return 0;
}

int sc_rmdev(socket_message_header_t *buf,unsigned int len)
{
    sc_dev_rm_t *data=(sc_dev_rm_t *)buf->data;
    socket_print("roomid:%s\n",data->roomid);

    socket_send_response((char*)buf,len,STATUS_OK,"0");

    return 0;
}

int sc_bt_restore(socket_message_header_t *buf,unsigned int len)
{
    sc_bt_restore_t *data=(sc_bt_restore_t *)buf->data;
    socket_print("roomid:%s\n",data->roomid);
    socket_print("backups:%s\n",data->backups);

    socket_send_response((char*)buf,len,STATUS_OK,"0");

    return 0;
}

int sc_bt_backup(socket_message_header_t *buf,unsigned int len)
{
    sc_bt_backup_t *data=(sc_bt_backup_t *)buf->data;
    socket_print("roomid:%s\n",data->roomid);

    sc_bt_backup_rsp_t rsp_data={
	.backups=DEVICE_BACKUPS,
    };

    char *rsp_buf=malloc(len+sizeof(rsp_data)+1);
    memset(rsp_buf,0,len+sizeof(rsp_data)+1);
    memcpy(rsp_buf,buf,len);
    memcpy(rsp_buf+len-1,&rsp_data,sizeof(rsp_data));

    socket_send_response(rsp_buf,len+sizeof(rsp_data)+1,STATUS_OK,"0");
    free(rsp_buf);

    return 0;
}

int sc_bt_query(socket_message_header_t *buf,unsigned int len)
{
    sc_bt_query_t *data=(sc_bt_query_t *)buf->data;
    socket_print("roomid:%s\n",data->roomid);

    sc_bt_query_rsp_t rsp_data={
	.status=DEVICE_STATUS_READYPAIR,
    };

    char *rsp_buf=malloc(len+sizeof(rsp_data)+1);
    memset(rsp_buf,0,len+sizeof(rsp_data)+1);
    memcpy(rsp_buf,buf,len);
    memcpy(rsp_buf+len-1,&rsp_data,sizeof(rsp_data));
    socket_send_response(rsp_buf,len+sizeof(rsp_data)+1,STATUS_OK,"0");
    free(rsp_buf);

    return 0;
}

int commit_rsp(socket_message_header_t *buf,unsigned int len)
{
    socket_message_rsp_t *rsp=(socket_message_rsp_t*)(buf->data+sizeof(cs_commit_msg_t));
    socket_print("Handle the commit rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int scan_rsp(socket_message_header_t *buf,unsigned int len)
{
    socket_message_rsp_t *rsp=(socket_message_rsp_t*)(buf->data+sizeof(cs_scan_msg_t));
    socket_print("Handle the scan rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int pair_rsp(socket_message_header_t *buf,unsigned int len)
{
    socket_message_rsp_t *rsp=(socket_message_rsp_t*)(buf->data+sizeof(cs_pair_msg_t));
    socket_print("Handle the pair rsp status=%s,error=%s\n",rsp->status,rsp->error);
    return 0;
}

int hb_rsp(socket_message_header_t *buf,unsigned int len)
{
    socket_message_rsp_t *rsp=(socket_message_rsp_t*)(buf->data+sizeof(cs_hb_msg_t));
    cs_hb_msg_t *hb=(cs_hb_msg_t*)(buf->data);
    socket_print("Handle the hb rsp status=%s,error=%s,hb->Mac=%s\n",rsp->status,rsp->error,hb->Mac);
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

