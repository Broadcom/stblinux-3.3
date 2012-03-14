.EXPORT_ALL_VARIABLES:

TOPDIR := $(shell pwd)

CFLAGS := -nostdinc -iwithprefix include -fno-stack-protector -fno-pic -mno-abicalls \
		-G 0 -Os -W -Wall -Wno-sign-compare -Wno-unused-parameter

CFLAGS += -I$(TOPDIR)/include/arch/mips \
	-I$(TOPDIR)/include/bits32 \
	-I$(TOPDIR)/include \
	-I$(ROOTDIR)/$(LINUXDIR)/include \
	-I$(ROOTDIR)/$(LINUXDIR)/arch/mips/include

SUBDIRS := $(TOPDIR)/syscalls $(TOPDIR)/libc $(TOPDIR)/libgcc

LIBS := $(TOPDIR)/syscalls/syscalls.a  $(TOPDIR)/libc/libc.a $(TOPDIR)/libgcc/libgcc.a

LDFLAGS += -G 0 -static -nostdlib

TARGET  := init
UTILITY := sha1sum

SRCS := init.c do_mounts.c name_to_dev.c sha1.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET) $(UTILITY)

$(TARGET): $(OBJS) $(LIBS)
	@rm -f $@
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ ./syscalls/crt0.o --start-group $(OBJS) $(LIBS) --end-group

$(UTILITY): sha1sum.c sha1.c 
	@rm -f sha1sum.o sha1.o
	$(CC) -I./ --static -o $@ $^

$(LIBS):
	for i in $(SUBDIRS) ; do $(MAKE) -C $$i || exit $?; done
	
romfs:
	chmod a+x init
	cp init $(ROOTDIR)/initramfs/
	#Generate signature list if rootfs directory is given.
	@if [ -n "$(ROOTFSDIR)" ]; then \
		./gen_signature.sh $(SIGLIST) $(ROOTFSDIR) || exit 1; \
		cp signature.txt $(ROOTDIR)/initramfs/etc; \
	else\
		cp $(SIGLIST) $(ROOTDIR)/initramfs/etc/;\
	fi;

clean:
	@rm -f $(TARGET) $(OBJS) $(UTILITY) 
	for i in $(SUBDIRS) ; do $(MAKE) -C $$i clean || exit $?; done


