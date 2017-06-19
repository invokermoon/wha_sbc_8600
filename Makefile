cur_dir :=$(PWD)

client_src_dir :=$(cur_dir)/src
server_src_dir :=$(cur_dir)/src
client_out_dir :=$(cur_dir)/out/client
server_out_dir :=$(cur_dir)/out/server


out_dir :=$(cur_dir)/out
client := $(out_dir)/client_bin
server := $(out_dir)/server_bin


$(info client_src_dir=$(client_src_dir))
$(info server_src_dir=$(server_src_dir))

#common_src:= $(client_src_dir)/cJSON.c
client_src:= $(client_src_dir)/client.c
client_src+= $(client_src_dir)/client_handle.c
client_src+= $(client_src_dir)/serial.c
#client_src+= $(common_src)

server_src:= $(server_src_dir)/server.c
#server_src+= $(common_src)
#OBJS    := $(patsubst %.c,$(arm_out_dir)/%.o,$(SOURCES))

#DIR_OBJS        = $(patsubst %.c,%.o,$(SOURCES))
#OBJS_NAME        = $(notdir $(patsubst %.c,%.o,$(SOURCES)))
#OBJS          = $(addprefix $(arm_out_dir)/,$(notdir $(patsubst %.c,%.o,$(SOURCES))))

client_objs = $(patsubst %.c,%.o,$(client_src))
client_objs_name = $(notdir $(patsubst %.c,%.o,$(client_src)))
client_objs_out = $(addprefix $(client_out_dir)/,$(client_objs_name))

server_objs = $(patsubst %.c,%.o,$(server_src))
server_objs_name = $(notdir $(patsubst %.c,%.o,$(server_src)))
server_objs_out = $(addprefix $(server_out_dir)/,$(server_objs_name))



$(info server_objs_out=$(server_objs_out))
$(info client_objs_out=$(client_objs_out))

#CC      := gcc
#CC :=/home/sherlock/envtools/Gcc/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-gcc
#CC :=/home/sherlock/envtools/Gcc/arm-2013.11/bin/arm-none-linux-gnueabi-gcc
CC :=/home/sherlock/envtools/Gcc/arm-2009q1/bin/arm-none-linux-gnueabi-gcc
CFLAGS  := -ggdb -Wall -DBUILD_TIME="\"`date`\"" -DDEBUG_
INCLUDE := -I ./include
LIB     := -lpthread -ldl -lrt -lm

.PHONY: clean all

all:$(client) $(server)

#client:$(client)
#server:$(server)

$(client):$(client_objs_out)
	$(CC) -o $@ $^ $(LIB)

$(client_objs_out):$(client_out_dir)/%.o:$(client_src_dir)/%.c
	mkdir -p $(client_out_dir)
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDE)

$(server):$(server_objs_out)
	    $(CC) -o $@ $^ $(LIB)

$(server_objs_out):$(server_out_dir)/%.o:$(client_src_dir)/%.c
	mkdir -p $(server_out_dir)
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDE)

clean:
	    rm $(client_objs_out) $(server_objs_out) $(out_dir) -rf

#install: $(client)
#	    cp $(client) ./bin/
