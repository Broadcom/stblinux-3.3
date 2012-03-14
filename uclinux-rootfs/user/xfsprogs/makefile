#
# Makefile for xfsprogs top level
#
# 09/05/03 Ton Truong <ttruong@broadcom.com>
#
TOOL_DIR_PREFIX=$(shell dirname $(shell which $(CC)))/..
XFSPROGS_DIR=$(shell pwd)

#CFLAGS += -I$(TOOL_DIR_PREFIX)/mipsel-linux/include/linux/xfs_support/
CFLAGS += \
		-I$(XFSPROGS_DIR) -I$(XFSPROGS_DIR)/include -I. \
		-I$(ROOTDIR)/user/e2fsprogs/lib \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" -fPIC
LDFLAGS += -L$(ROOTDIR)/user/e2fsprogs/lib

.EXPORT_ALL_VARIABLES:


# THT Took out uuid.  Library now available in romfs/lib:wq
# jipeng - remove libxfs to avoid building host side utils
DIRS =  libdisk libxfs libxlog fsck mkfs repair

all:
	-$(RM) xfs && ln -s include ./xfs
	-$(RM) disk && ln -s include ./disk
	-$(RM) log && ln -s include ./log
	for i in $(DIRS) ; do make -C $$i || exit 1 ; done

romfs:
	for i in $(DIRS) ; do make -C $$i romfs || exit 1 ; done

clean:
	-for i in $(DIRS) ; do make -C $$i clean ; done
	-$(RM) xfs disk log
