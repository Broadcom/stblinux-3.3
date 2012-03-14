#CC := mipsel-linux-gcc
#LD := mipsel-linux-ld
#AR := mipsel-linux-ar
#RANLIB := mipsel-linux-ranlib

#TOPDIR := $(shell pwd)/..



#KERNEL := /exp/leisun/leisun_latest_2631/vobs/Ref_Linux_Kernel/stblinux-2.6.31

#CFLAGS += -D__ASSEMBLY__ -iwithprefix include -fno-stack-protector -fno-pic -mno-abicalls -G 0 -Os -W -Wall -Wno-sign-compare -Wno-unused-parameter
#CFLAGS += -nostdinc -I$(TOPDIR)/include/arch/mips -I$(TOPDIR)/include/bits32 -I$(KERNEL)/include -I$(KERNEL)/arch/mips/include 

CFLAGS += -D__ASSEMBLY__ 

TARGET := syscalls.a 
CRTOBJS := crt0.o

SRCS := syscalls.S vfork.S setjmp.S
OBJS = $(SRCS:.S=.o)

all: $(TARGET) $(CRTOBJS)

$(TARGET): $(OBJS)
	@rm -f $@
	$(AR) cr  $@ $(OBJS)
	$(RANLIB) $@

$(CRTOBJS): crt0.S
	@echo "Compiling " $@ $^
	$(CC) $(CFLAGS) -o $@ -c $^

$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) -o $@ -c $(@:.o=.S) 

clean:
	@rm -f $(TARGET) $(OBJS) $(CRTOBJS)
