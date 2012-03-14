#
# Makefile for xfsprogs/mkfs
#
# 09/05/03 Ton Truong <ttruong@broadcom.com>
#
EXEC   =  xfs_repair
HFILES = agheader.h attr_repair.h avl.h avl64.h bmap.h dinode.h dir.h \
	dir2.h dir_stack.h err_protos.h globals.h incore.h protos.h rt.h \
	scan.h versions.h

CFILES = agheader.c attr_repair.c avl.c avl64.c bmap.c dino_chunks.c \
	dinode.c dir.c dir2.c dir_stack.c globals.c incore.c \
	incore_bmc.c init.c incore_ext.c incore_ino.c io.c phase1.c \
	phase2.c phase3.c phase4.c phase5.c phase6.c phase7.c rt.c sb.c \
	scan.c versions.c xfs_repair.c
OBJS   =  $(CFILES:.c=.o)
CFLAGS += -I../include -I.. -I. \
		-I$(ROOTDIR)/romfs/usr/include \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" \
		-fPIC

LDLIBS  += -lxfs -lxlog -luuid
LDFLAGS += -L../libxfs -L../libxlog

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS)  $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o
