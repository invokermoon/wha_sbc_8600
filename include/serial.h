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
#ifndef _SERIAL_H_
#define _SERIAL_H_

#define BUFFER_SIZE 100

#define HEAD_BYTE1_VALUE	0xA5
#define HEAD_BYTE2_VALUE	0x7F
#define TAIL_BYTE1_VALUE	0xF7
#define TAIL_BYTE2_VALUE	0x5A

/*including checksum tail1 and tail2*/
#define MSG_TAIL_SIZE	3

typedef enum serial_cmd_type_e{
    serial_cmd_type_CHECK=0x0,
    serial_cmd_type_TEST=0x1,
    serial_cmd_type_PAIR=0x2,
    serial_cmd_type_PAIR_RSP=0x3,
    serial_cmd_type_PAIR_BCK=0x4,
    serial_cmd_type_PAIR_BCK_RSP=0x5,
}serial_cmd_type_t;

/*serial response message,except for checksum,tail 2 bytes*/
typedef struct serial_recv_message_s{
    unsigned char head1;
    unsigned char head2;
    unsigned char ret_num1;
    unsigned char ret_num2;
    unsigned char floorid;
    unsigned char roomid;
    unsigned char cmd_id;
    unsigned char data_len;
    /*these 3 bytes should be parsed in last*/
    unsigned char checksum;
    unsigned char tail1;
    unsigned char tail2;

    unsigned char data[0];
}serial_recv_message_t;

enum header_send_index_e{
    INDEX_SEND_HEAD1=0,
    INDEX_SEND_HEAD2=1,
    INDEX_SEND_FLOORID=2,
    INDEX_SEND_ROOMID=3,
    INDEX_SEND_CMDID=4,
    INDEX_SEND_DATA_LEN=5,
    INDEX_SEND_HEADER_SIZE=6,
};

enum header_recv_index_e{
    INDEX_RECV_HEAD1=0,
    INDEX_RECV_HEAD2=1,
    INDEX_RECV_RETNUM1=2,
    INDEX_RECV_RETNUM2=3,
    INDEX_RECV_FLOORID=4,
    INDEX_RECV_ROOMID=5,
    INDEX_RECV_CMDID=6,
    INDEX_RECV_DATA_LEN=7,
    INDEX_RECV_HEADER_SIZE=8,
};

typedef struct serial_send_node_s{
    int rewrite;
    int buf_len;
    unsigned char buf[100];
    struct list_head list;
}serial_send_node_t;

/*Node of serial recv message*/
typedef struct serial_recv_node_s{
    unsigned char one_msg[BUFLEN];
    int valid_len;
    int valid;
    struct list_head list;
}serial_recv_node_t;


int serial_init(void );
unsigned char *serial_send_message(char,char,serial_cmd_type_t,char *,unsigned int );

#endif
