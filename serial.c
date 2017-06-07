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

#define MSG_COMMON_BYTES	11
#define HEAD_BYTE1_VALUE	0xA5
#define HEAD_BYTE2_VALUE	0x7f
#define TAIL_BYTE1_VALUE	0x7f
#define TAIL_BYTE2_VALUE	0x5A

typedef enum cmd_type_s{
    CMD_TYPE_CHECK=0x0,
    CMD_TYPE_TEST=0x1,
    CMD_TYPE_PAIR=0x2,
    CMD_TYPE_PAIR_RSP=0x3,
    CMD_TYPE_PAIR_BCK=0x4,
    CMD_TYPE_PAIR_BCK_RSP=0x5,
}cmd_type_t;

/*serial response message,except for checksum,tail 2 bytes*/
typedef struct serial_rsp_msg{
    char head1;
    char head2;
    char ret_num1;
    char ret_num2;
    char floorid;
    char roomid;
    cmd_type_t cmd_id;
    char data_len;
    /*these 3 bytes should be parsed in last*/
    char checksum;
    char tail1;
    char tail2;

    char data[0];
}serial_rsp_msg_t;

char *send_serial_msg(char floorid,\
			char roomid, \
			cmd_type_t cmd_id, \
			char *data,  \
			unsigned int data_len)
{
    int i=0;
    char *buf=malloc(data_len+MSG_COMMON_BYTES);
    buf[0]=HEAD_BYTE1_VALUE;
    buf[1]=HEAD_BYTE2_VALUE;
    buf[2]=floorid;
    buf[3]=roomid;
    //buf[4]=0x0;
    //buf[5]=0x0;
    buf[4]=cmd_id;
    buf[5]=data_len;
    if(data_len>0){
	memcpy(&buf[6],data,data_len);
    }
    unsigned int checksum=floorid+roomid+cmd_id+data_len;
    for(i=0;i<data_len;i++){
	checksum=checksum+buf[6+i];
    }
    checksum=~(checksum&&0xff)+0x01;
    int tmp_len=6+data_len;
    buf[tmp_len]=checksum;
    buf[tmp_len+1]=TAIL_BYTE1_VALUE;
    buf[tmp_len+2]=TAIL_BYTE2_VALUE;
    for(i=0;i++;i<data_len+MSG_COMMON_BYTES){
	sbc_print("buf[%d]=%x\n",i,buf[i]);
    }
    return buf;
}

serial_rsp_msg_t *parse_recv_msg(char *buf)
{
    int data_len=buf[7];
    serial_rsp_msg_t *data=malloc(sizeof(serial_rsp_msg_t)+data_len);
    data->head1=buf[0];
    data->head2=buf[1];
    data->ret_num1=buf[2];
    data->ret_num2=buf[3];
    if(data->head1!=HEAD_BYTE1_VALUE ||data->head2!=HEAD_BYTE2_VALUE ){
	sbc_print("Head Bytes are error,0x%x,0x%x\n",data->head1,data->head2);
	return NULL;
    }
    if(data->ret_num1!=0x88||data->ret_num2!=0x88){
	sbc_print("ret nums are error,0x%x,0x%x\n",data->ret_num1,data->ret_num2);
	return NULL;
    }

    data->floorid=buf[4];
    data->roomid=buf[5];
    data->cmd_id=buf[6];
    data->data_len=buf[7];
    data->checksum=buf[8+data_len];
    data->tail1=buf[9+data_len];
    data->tail2=buf[10+data_len];
    if(data->tail1!=TAIL_BYTE1_VALUE||data->tail2!=TAIL_BYTE2_VALUE){
	sbc_print("tails are error,0x%x,0x%x\n",data->tail1,data->tail2);
	return NULL;
    }
    if(data_len>0){
	memcpy(data->data,buf[8],data_len);
    }

    sbc_print("serial_rsp_msg: \nfloorid=0x%x\nroom_id=0x%x\ncmd_id=0x%x\ndata_len=%d\n",\
	    data->floorid,data->roomid,data->cmd_id,data->data_len);
    for(i=0;i++;i<data_len){
	sbc_print("serial_rsp data[%d]=%x\n",i,data->data[i]);
    }
    return buf;
}

int serial_init()
{
    int fd,bytes_read;

    fd = open("/dev/ttyO0", O_RDWR);
    if(fd==-1){
	sbc_print("Open dev error\n");
	return -1;
    }
    while(bytes_read= read(fd,buffer,BUFFSIZE)){
	if ((bytes_read == -1) && (errno != EINTR)) break;
	else if (bytes_read > 0) {
	    serial_rsp_msg_t *rsp_msg = parse_recv_msg(buffer);
	    process(rsp_msg);
	    free(rsp_msg);
	}
    }
    sbc_print("Exception,Shut Down\n");
    sleep(1);
    return 0;
}
