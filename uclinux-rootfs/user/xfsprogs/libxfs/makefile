#
# Makefile for xfsprogs/libxfs.a
#
# 09/05/03 Ton Truong
#

LIBXFS = libxfs.so
BUILD_LIBXFS = build/libxfs.a

BUILD_CFLAGS = -I$(LINUXDIR)/include -I../include -I.. -I. \
	-I$(ROOTDIR)/romfs/usr/include \
	-DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
	-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE

BUILD_CC = gcc

#THT Some .c files include <xfs.h>, but should be "xfs.h" to pick up the local dir.
# as there is another xfs.h from ./include
CFLAGS += -I. -I../include -I.. \
		-I$(ROOTDIR)/romfs/usr/include \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" \
		-fPIC

HFILES = xfs.h init.h
CFILES = bit.c init.c logitem.c rdwr.c trans.c util.c \
	xfs_alloc.c xfs_ialloc.c xfs_rtalloc.c \
	xfs_inode.c xfs_btree.c xfs_alloc_btree.c xfs_ialloc_btree.c \
	xfs_bmap_btree.c xfs_da_btree.c xfs_dir.c xfs_dir_leaf.c \
	xfs_dir2.c xfs_dir2_leaf.c xfs_attr_leaf.c xfs_dir2_block.c \
	xfs_dir2_node.c xfs_dir2_data.c xfs_dir2_sf.c xfs_bmap.c \
	xfs_mount.c xfs_trans.c \
	linux.c

OBJS   =  $(CFILES:.c=.o)
BUILD_OBJS = $(patsubst %,build/%,$(OBJS))


LDSHARED=$(CC) --shared -Wl,-soname,$(LIBXFS)

# jipeng - remove $(BUILD_LIBXFS) to avoid building host side binary 
#all: build $(ROOTDIR)/romfs/lib/$(LIBXFS) $(BUILD_LIBXFS)
all: build $(ROOTDIR)/romfs/lib/$(LIBXFS)

.PHONY:
build:
	[ -d build ] || mkdir build
	
$(BUILD_LIBXFS): $(BUILD_OBJS)
	ar crv $@ $(BUILD_OBJS)
	ranlib $@


$(LIBXFS): $(OBJS)
	$(RM) -f $(LIBXFS)
	$(LDSHARED) -o $(LIBXFS) $(OBJS)


$(ROOTDIR)/romfs/lib/$(LIBXFS): $(LIBXFS)
	cp -f $(LIBXFS) $@
	$(CROSS)strip $@

romfs:

clean:
	rm -rf $(OBJS) $(LIBXFS) build
	
build/%.o: %.c
	$(BUILD_CC) $(BUILD_CFLAGS) -c -o $@ $<

build/%: %.c
	$(BUILD_CC) $(BUILD_CFLAGS) -o $@ $<

