#
# Makefile for xfsprogs/mkfs
#
# 09/05/03 Ton Truong <ttruong@broadcom.com>
#
EXEC   =  mkfs.xfs

MAXTRRES = maxtrres
CFILES = trtab.c proto.c xfs_mkfs.c

HFILES = xfs_mkfs.h $(MAXTRRES).h
OBJS   =  $(CFILES:.c=.o)
CFLAGS += -I../include -I.. -I. \
		-I$(ROOTDIR)/romfs/usr/include \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" \
		-fPIC
		
LDLIBS	+= -lxfs -ldisk -luuid
LDFLAGS += -L$(ROOTDIR)/romfs/lib

LTEXEC = libtool --mode=execute
# THT: These build things are on the Build machine (the PC).
BUILD_CC=gcc
BUILD_CFLAGS = -I$(ROOTDIR)/$(LINUXDIR)/include -I../include -I.. -I. \
	-DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
	-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE
BUILD_LDFLAGS=
BUILD_LIBXFS=../libxfs/build/libxfs.a
BUILD_LIBUUID=/lib/libuuid.so.1
BUILD_MAXTRLIBS = $(BUILD_LIBXFS) $(BUILD_LIBUUID)

all: $(EXEC)

$(EXEC): $(MAXTRRES).h $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

# THT: Remove build rule for maxtress.h as some platforms (RHEL) does not have the optional library libuuid installed
#$(MAXTRRES): $(MAXTRRES).o
#	$(BUILD_CC) $@.c -o $@ $(BUILD_CFLAGS) $(BUILD_LDFLAGS) $(BUILD_MAXTRLIBS)

#$(MAXTRRES).h: $(MAXTRRES)
#	./$(MAXTRRES) > $@ || ( rm -f $@ && exit 1 )

romfs:
	$(ROMFSINST) /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o
