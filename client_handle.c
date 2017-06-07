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

int send_scan(void *buf,unsigned int len)
{
    return 0;
}

int send_commit(void *buf,unsigned int len)
{
    return 0;
}
int send_pair(void *buf,unsigned int len)
{
    return 0;
}

int send_heatbeat(void *buf,unsigned int len)
{
    return 0;
}

int recv_setting(void *buf,unsigned int len)
{
    return 0;
}

static struct handler_driver SBC8600= {
	.sname = "SBC",
	.funcs = {
	    .send_scan=&send_scan,
	    .send_commit=&send_commit,
	    .send_pair=&send_pair,
	    .send_hb=&send_heatbeat,
	    .recv_setting=&recv_setting,
	    .rsp_scan=NULL,
	    .rsp_commit=NULL,
	    .rsp_pair=NULL,
	    .rsp_hb=NULL,
	}
};

define_drvinit(SBC8600);
