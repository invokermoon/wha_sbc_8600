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
#ifndef _CLIENT_H_
#define _CLIENT_H_
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>  
#include "cJSON.h"
#include "list.h"


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
#define blue_print(fmt,...) do {  printf(BLUE"[%s]:"NONE fmt,__func__,##__VA_ARGS__) ;} while(0)
#define sbc_color_print(color,fmt,...) do {  printf(color"[%s]:"color fmt NONE,__func__,##__VA_ARGS__) ;} while(0)

/*IP and port*/
#define IP_ADDR	"106.14.60.75"
#define IP_PORT	8668

/*socket fd*/
int sockfd;

#define DEVICE_BACKUPS			"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789ABCDEFGHIJKLMNOPQR"

#define BUFLEN 300

/*
 * |: message split symbol
 * #: attribute symbol
 * &: arry symbol
 * */
/*MSG header and tail*/
#define MSG_HEADER_STRING	"<<"
#define MSG_TAIL_STRING		">>"

/*
 * interface type:clinet --->server
 * */
#define ITYPE_CS_COMMITINFO 	"01"
#define ITYPE_CS_SCAN		"02"
#define ITYPE_CS_PAIR_OK		"03"
#define ITYPE_CS_HEARTBEAT		"88"

/*interface type:server-->client*/
#define ITYPE_SC_SETDEV		"61"
#define ITYPE_SC_RMDEV		"62"
#define ITYPE_SC_BT_RESTORE	"63"
#define ITYPE_SC_BT_BACKUP	"64"
#define ITYPE_SC_BT_QUERY	"65"

/*Mac*/
#define MAC_ADDRESS		"12345678"
#define MAC_ADDRESS__		"12345678|"
#define BLE_MAC_ADDRESS		"88:22:33:44:55:66"
#define BLE_MAC_ADDRESS__	"88:22:33:44:55:66|"
#define PHONE_MAC_ADDRESS	"77:22:33:44:55:66"
#define PHONE_MAC_ADDRESS__	"77:22:33:44:55:66|"


#define MSG_ID		"10000001|"
#define VERSION		"01|"
#define DEVICE		"ABCDEFGHIJKL|"
/*Rsp status*/
#define STATUS_OK 		"00"
#define STATUS_FORMAT_FAIL	"71"
#define STATUS_MSG_INCOMPLETE	"72"
#define STATUS_INSIDE_ERROR	"73"
#define STATUS_NOSBC		"74"
#define STATUS_SBC_OFFLINE	"75"
#define STATUS_BT_DAMAGE	"76"

/*device status*/
#define DEVICE_STATUS_RECONNECT		"01"
#define DEVICE_STATUS_READYPAIR 	"02"
#define DEVICE_STATUS_NOPAIR 		"03"
#define DEVICE_STATUS_DAMAGE 		"04"

/*Common msg header*/
typedef struct msg_header{
    char head[2];/*message header string*/
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
    char error[0];
}msg_rsp_t;

/*Msg struct :client to server*/
/*commit cmd*/
typedef struct cs_commit_msg_s{
    char Mac[8+1];
}cs_commit_msg_t;

typedef struct cs_hb_msg_s{
    char Mac[8+1];
}cs_hb_msg_t;

/*scan cmd*/
typedef struct cs_scan_msg_s{
    char Mac[8+1];
    char roomid[4+1];
    char bleMac[17+1];
    char phoneMac[17+1];
    char timestamp[19+1];
}cs_scan_msg_t;

typedef struct cs_pair_msg_s{
    char roomid[4+1];
    char Mac[8+1];
    char bleMac[17+1];
    char phoneMac[17+1];
    char timestamp[19+1];
}cs_pair_msg_t;

/*Msg struct :server to client*/
typedef struct sc_dev_status_s{
    char roomid[4+1];
    char status[2+1];	/*refer to the macro  DEVICE_STATUS_XXXX*/
}sc_dev_status_t;

typedef struct sc_dev_rm_s{
    char roomid[4+1];
}sc_dev_rm_t;

typedef struct sc_bt_restore_s{
    char roomid[4+1];
    char backups[64+1];
}sc_bt_restore_t;

typedef struct sc_bt_backup_s{
    char roomid[4+1];
}sc_bt_backup_t;

typedef struct sc_bt_backup_rsp_s{
    char backups[64+1];
}sc_bt_backup_rsp_t;

typedef struct sc_bt_query_s{
    char roomid[4+1];
}sc_bt_query_t;

typedef struct sc_bt_query_rsp_s{
    char status[2+1];	/*refer to the macro  DEVICE_STATUS_XXXX*/
}sc_bt_query_rsp_t;

/*Node of message*/
typedef struct message_node_s{
    char one_msg[BUFLEN];
    int valid;
    struct list_head list;
}message_node_t;


/****Funcs****/
char *make_send_msg(char *itype,void *data, unsigned int data_len);
int send_message(char *itype,void *data, unsigned int data_len);
int send_response(char *buf,unsigned int,char *status,char *error);

int serial_init(void );
char *system_timestamp();

struct handler_driver;
typedef int (*cs_scan_f)(void *,unsigned int);
typedef int (*cs_commit_f)(void *,unsigned int);
typedef int (*cs_pairok_f)(void *,unsigned int);
typedef int (*cs_hb_f)(void *,unsigned int);

typedef int (*sc_setdev_f)(msg_header_t*,unsigned int);
typedef int (*sc_rmdev_f)(msg_header_t*,unsigned int);
typedef int (*sc_bt_restore_f)(msg_header_t*,unsigned int);
typedef int (*sc_bt_backup_f)(msg_header_t*,unsigned int);
typedef int (*sc_bt_query_f)(msg_header_t*,unsigned int);

typedef int (*sc_rsp_scan_f)(msg_header_t*,unsigned int);
typedef int (*sc_rsp_commit_f)(msg_header_t*,unsigned int);
typedef int (*sc_rsp_hb_f)(msg_header_t*,unsigned int);
typedef int (*sc_rsp_pair_f)(msg_header_t*,unsigned int);


struct handler_funcs {
	cs_scan_f cs_scan;
	cs_commit_f cs_commit;
	cs_pairok_f cs_pair;
	cs_hb_f cs_hb;

	sc_setdev_f sc_setdev;
	sc_rmdev_f sc_rmdev;
	sc_bt_restore_f sc_bt_restore;
	sc_bt_backup_f sc_bt_backup;
	sc_bt_query_f sc_bt_query;

	sc_rsp_scan_f sc_scan_rsp;
	sc_rsp_commit_f sc_commit_rsp;
	sc_rsp_pair_f sc_pair_rsp;
	sc_rsp_hb_f sc_hb_rsp;
};

struct handler_driver {
	struct handler_funcs funcs;
	char sname[20];
};

//struct handler_driver SBC8600;
struct handler_driver *handler;
#define define_drvinit(name) struct handler_driver *handler=&name

#endif
