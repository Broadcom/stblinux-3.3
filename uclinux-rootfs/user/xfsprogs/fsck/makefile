#
# Makefile for xfsprogs/fsck
#
# 09/05/03 Ton Truong <ttruong@broadcom.com>
#
EXEC   =  fsck.xfs
OBJS   =  xfs_fsck.o
CFLAGS += -I../include \
		-I$(ROOTDIR)/romfs/usr/include \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" \
		-fPIC
		
LDFLAGS += -L$(ROOTDIR)/romfs/lib
LDADD  = ../libxfs/libxfs.so

all: $(EXEC)

$(EXEC): $(OBJS) $(LDADD)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDADD) $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o
