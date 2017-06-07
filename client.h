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
struct handler_driver;
typedef int (*cmd_scan_f)(void *,unsigned int);
typedef int (*cmd_commit_f)(void *,unsigned int);
typedef int (*cmd_pairok_f)(void *,unsigned int);
typedef int (*cmd_hb_f)(void *,unsigned int);
typedef int (*cmd_setting_f)(void *,unsigned int);
typedef int (*rsp_scan_f)(void *,unsigned int);
typedef int (*rsp_commit_f)(void *,unsigned int);
typedef int (*rsp_hb_f)(void *,unsigned int);
typedef int (*rsp_pair_f)(void *,unsigned int);


struct handler_funcs {
	cmd_scan_f send_scan;
	cmd_commit_f send_commit;
	cmd_pairok_f send_pair;
	cmd_hb_f send_hb;
	cmd_setting_f recv_setting;
	rsp_scan_f rsp_scan;
	rsp_commit_f rsp_commit;
	rsp_pair_f rsp_pair;
	rsp_hb_f rsp_hb;
};

struct handler_driver {
	struct handler_funcs funcs;
	char sname[20];
};

struct handler_driver *hanlder=NULL;
#define define_init(name) \
    extern struct handler_driver name;\
    handler=&name

#endif
