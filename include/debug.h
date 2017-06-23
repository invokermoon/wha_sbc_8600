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
#ifndef _DEBUG_H_
#define _DEBUG_H_

/*colors*/
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

enum debug_level_e{
    DEBUG_ERROR=0,
    DEBUG_WARN=1,
    DEBUG_INFO=2,
    DEBUG_DBG=3,
    DEBUG_ALL=4,
};

#define DEBUG_LEVEL 3 

static __inline__ char *get_debug_level_name(enum debug_level_e eee)
{
    if(eee == DEBUG_DBG) return "DEBUG ";
    else if(eee == DEBUG_INFO) return "INFO  ";
    else if(eee == DEBUG_WARN) return "WARN  ";
    else if(eee == DEBUG_DBG) return "ERROR ";
    return "DEBUG_ALL";
}


#define sbc_print(flag,fmt,...) do { \
   if( flag < DEBUG_LEVEL ) {printf("%s %s"YELLOW"[%s][%s]:"NONE fmt,__DATE__,__TIME__,get_debug_level_name(flag),__func__,##__VA_ARGS__) ; fflush(stdout);}} while(0)
#define socket_print(fmt,...) do {  printf("%s %s"GREEN"[Socket][%s]:"NONE fmt,__DATE__,__TIME__,__func__,##__VA_ARGS__) ; fflush(stdout);} while(0)
#define serial_print(fmt,...) do {  printf("%s %s"BLUE"[Serial][%s]:"NONE fmt,__DATE__,__TIME__,__func__,##__VA_ARGS__) ; fflush(stdout);} while(0)
#define sbc_color_print(color,fmt,...) do {  printf("%s %s"color"[%s]:"color fmt NONE,__DATE__,__TIME__,__func__,##__VA_ARGS__) ; fflush(stdout);} while(0)

#endif
