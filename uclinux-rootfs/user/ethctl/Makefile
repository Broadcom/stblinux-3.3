#
# Makefile for ethctl utility.
#
PROGRAM_NAME:= ethctl

OBJS := ethctl.o ethctl_cmd.o

CFLAGS =-s -Os -fomit-frame-pointer 

KERNEL_DIR = $(realpath ../../linux)

INC_ETHERNET_DRIVER_PATH=$(KERNEL_DIR)/drivers/net/brcmint7038
INC_KERNEL_INC_PATH=$(KERNEL_DIR)/include
CFLAGS += -Wall -I$(INC_ETHERNET_DRIVER_PATH) -I$(INC_KERNEL_INC_PATH)

ifeq ($(strip $(BUILD_WLCTL)), static)
CFLAGS += -DBUILD_STATIC 
endif
#CC=mipsel-linux-gcc
#CC=mips-linux-gcc

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

all: $(PROGRAM_NAME)

install::
	install -m 755 $(PROGRAM_NAME) $(INSTALL_DIR)/bin
	$(STRIP) $(INSTALL_DIR)/bin/$(PROGRAM_NAME)
	mkdir -p $(INSTALL_DIR)/sbin
	ln -sf ../bin/$(PROGRAM_NAME) $(INSTALL_DIR)/sbin/ethctl

dynamic: $(PROGRAM_NAME) install

static: $(PROGRAM_NAME).a

$(PROGRAM_NAME): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -Wl,--allow-shlib-undefined -o $@ $^

$(PROGRAM_NAME).a: $(OBJS)
	$(AR) rcs $(PROGRAM_NAME).a $^

clean:
	-rm -f $(PROGRAM_NAME) *.o *.a *.elf *.gdb

romfs:
	$(ROMFSINST) /bin/$(PROGRAM_NAME)
