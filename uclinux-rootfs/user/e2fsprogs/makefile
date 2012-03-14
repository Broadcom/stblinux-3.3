include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os
SHELL			:= bash

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux || exit 1 ; \
	fi
	$(MAKE) -f Makefile libs progs

.PHONY: romfs
romfs::
	$(STRIP) e2fsck/e2fsck resize/resize2fs debugfs/debugfs \
		misc/{mke2fs,badblocks,chattr,dumpe2fs,fsck,lsattr} \
		misc/{mklost+found,tune2fs}
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_E2FSCK_E2FSCK \
		e2fsck/e2fsck /sbin/e2fsck
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_E2FSCK_E2FSCK -s \
		e2fsck /sbin/fsck.ext2
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_E2FSCK_E2FSCK -s \
		e2fsck /sbin/fsck.ext3
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_E2FSCK_E2FSCK -s \
		e2fsck /sbin/fsck.ext4
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_MKE2FS \
		misc/mke2fs /sbin/mke2fs
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_MKE2FS -s \
		mke2fs /sbin/mkfs.ext2
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_MKE2FS -s \
		mke2fs /sbin/mkfs.ext3
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_MKE2FS -s \
		mke2fs /sbin/mkfs.ext4
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_MKE2FS -s \
		mke2fs /sbin/mkfs.ext4dev
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_BADBLOCKS \
		misc/badblocks /sbin/badblocks
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_CHATTR \
		misc/chattr /bin/chattr
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_DUMPE2FS \
		misc/dumpe2fs /sbin/dumpe2fs
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_FSCK \
		misc/fsck /sbin/fsck
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_LSATTR \
		misc/lsattr /bin/lsattr
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_MKLOST_FOUND \
		misc/mklost+found /sbin/mklost+found
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_TUNE2FS \
		misc/tune2fs /sbin/tune2fs
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_TUNE2FS -s \
		tune2fs /sbin/e2label
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_TUNE2FS -s \
		tune2fs /sbin/findfs
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_RESIZE_RESIZE2FS \
		resize/resize2fs /sbin/resize2fs
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_MISC_UUIDGEN \
		misc/uuidgen /bin/uuidgen
	$(ROMFSINST) -e CONFIG_USER_E2FSPROGS_DEBUGFS_DEBUGFS \
		debugfs/debugfs /sbin/debugfs

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
