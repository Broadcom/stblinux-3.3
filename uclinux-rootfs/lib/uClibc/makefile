include $(ROOTDIR)/config.arch

unexport MAKEFILE_LIST

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

# Only build locally if necessary
ifeq ($(CONFIG_LIB_LIBC_FROM_ROOTFS),y)
libc-source := rootfs
else
# unfortunately, we don't have ldd/ldconfig in stbgcc-4.5 (uClibc) sysroot
ifneq ($(CONFIG_LIB_LIBC_LDD)$(CONFIG_LIB_LIBC_LDCONFIG),)
libc-source := rootfs
else
libc-source := toolchain
endif
endif

.PHONY: all
all:: all-$(libc-source)

.PHONY: all-rootfs
all-rootfs:
	$(MAKE) -f Makefile
	$(MAKE) -f Makefile utils

.PHONY: all-toolchain
all-toolchain:

.PHONY: romfs
romfs:: romfs-$(libc-source)

.PHONY: romfs-rootfs
romfs-rootfs:
	mkdir -p $(ROMFSDIR)/lib
	cp -df lib/*.so* $(ROMFSDIR)/lib/
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDD utils/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDCONFIG utils/ldconfig /sbin/ldconfig

.PHONY: romfs-toolchain
romfs-toolchain::
	mkdir -p $(ROMFSDIR)/lib
	cp -df $(GCCSYSROOT)/lib/*.so* $(ROMFSDIR)/lib
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDD $(GCCSYSROOT)/usr/bin/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDCONFIG $(GCCSYSROOT)/sbin/ldconfig /sbin/ldconfig

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
