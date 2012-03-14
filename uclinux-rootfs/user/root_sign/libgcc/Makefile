#CC := mipsel-linux-gcc
#LD := mipsel-linux-ld
#AR := mipsel-linux-ar
#RANLIB := mipsel-linux-ranlib

#TOPDIR := $(shell pwd)/..



#KERNEL := /exp/leisun/leisun_latest_2631/vobs/Ref_Linux_Kernel/stblinux-2.6.31

#CFLAGS += -iwithprefix include -fno-stack-protector -fno-pic -mno-abicalls -G 0 -Os -W -Wall \
	-Wno-sign-compare -Wno-unused-parameter
#CFLAGS += -nostdinc -I$(TOPDIR)/include/arch/mips -I$(TOPDIR)/include/bits32 -I$(KERNEL)/include -I$(KERNEL)/arch/mips/include 
#CFLAGS += -nostdinc -I$(TOPDIR)/include/arch/mips -I$(TOPDIR)/include/bits32 

TARGET := libgcc.a 

SRCS = __ashldi3.c __ashrdi3.c __clzsi2.c __divdi3.c __divsi3.c __lshrdi3.c __moddi3.c __modsi3.c\
	   __udivdi3.c __udivmoddi4.c __udivmodsi4.c __udivsi3.c __umoddi3.c __umodsi3.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	@rm -f $@
	$(AR) cr  $@ $(OBJS)
	$(RANLIB) $@

$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) -o $@ -c $(@:.o=.c) 

clean:
	@rm -f $(TARGET) $(OBJS)
