TOPDIR			:= $(shell pwd)
OLDPC			:= 0

ifeq ($(OLDPC),1)
# OLDPC=1: Red Hat 7.3 toolchain build environment
OLDPCFLAGS		:= -DAI_ADDRCONFIG=0
endif

.PHONY: all distclean
all: distclean
	mkdir bin
	tar zxf dist/lzo-2.03.tar.gz
	tar jxf dist/mtdutils-20100923.tar.bz2
	tar zxf dist/squashfs4.0.tar.gz
	tar zxf dist/module-init-tools-3.8.tar.gz
	patch -p1 -d mtd-utils < $(TOPDIR)/dist/mtd-utils.patch
ifeq ($(OLDPC),1)
	patch -p1 -d mtd-utils < $(TOPDIR)/dist/mtd-utils-oldpc.patch
endif
	patch -p1 -d squashfs4.0 < $(TOPDIR)/dist/squashfs4.0.patch
	patch -p1 -d module-init-tools-3.8 < $(TOPDIR)/dist/module-init-tools.patch
	cd lzo-2.03 && ./configure
	make -C lzo-2.03
	cd mtd-utils && make WITHOUT_XATTR=1 \
		LZOLDFLAGS="-L$(TOPDIR)/lzo-2.03/src/.libs" \
		LZOCPPFLAGS="-I$(TOPDIR)/lzo-2.03/include $(OLDPCFLAGS)"
	cp mtd-utils/{mkfs.jffs2,sumtool} mtd-utils/mkfs.ubifs/mkfs.ubifs \
		mtd-utils/ubi-utils/ubinize bin/
	make -C squashfs4.0/squashfs-tools mksquashfs unsquashfs
	cp squashfs4.0/squashfs-tools/{mksquashfs,unsquashfs} bin/
	cd module-init-tools-3.8 && ./configure --target=mipsel-linux
	make -C module-init-tools-3.8/build depmod
	cp module-init-tools-3.8/build/depmod bin/mipsel-linux-depmod
	make -C module-init-tools-3.8/build clean
	cd module-init-tools-3.8 && ./configure --target=mips-linux
	make -C module-init-tools-3.8/build depmod
	cp module-init-tools-3.8/build/depmod bin/mips-linux-depmod
	strip bin/*

distclean:
	rm -rf lzo-2.03 mtd-utils squashfs4.0 module-init-tools-3.8 bin
