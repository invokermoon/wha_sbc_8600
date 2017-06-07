client_src_dir :=$(PWD)
server_src_dir :=$(PWD)
client_out_dir :=$(PWD)/out/client
server_out_dir :=$(PWD)/out/server

out_dir :=$(PWD)/out
client := $(out_dir)/client_bin
server := $(out_dir)/server_bin


$(info client_src_dir=$(client_src_dir))
$(info server_src_dir=$(server_src_dir))


common_src:= $(client_src_dir)/cJSON.c
client_src:= $(client_src_dir)/client.c
client_src+= $(client_src_dir)/client_handle.c
client_src+= $(common_src)

server_src:= $(server_src_dir)/server.c
server_src+= $(common_src)
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

CC      := gcc
CFLAGS  := -ggdb -Wall -DBUILD_TIME="\"`date`\"" -DDEBUG_
INCLUDE := -I ./
LIB     := -lpthread -ldl -lrt -lm

.PHONY: clean all

all:$(client) $(server)

#client:$(client)
#server:$(server)

$(client):$(client_objs_out)
	$(CC) -o $@ $^ $(LIB)

$(client_objs_out):$(client_out_dir)/%.o:%.c
	mkdir -p $(client_out_dir)
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDE)

$(server):$(server_objs_out)
	    $(CC) -o $@ $^ $(LIB)

$(server_objs_out):$(server_out_dir)/%.o:%.c
	mkdir -p $(server_out_dir)
	$(CC) -o $@ -c $< $(CFLAGS) $(INCLUDE)

clean:
	    rm $(client_objs_out) $(server_objs_out) $(out_dir) -rf

#install: $(client)
#	    cp $(client) ./bin/
