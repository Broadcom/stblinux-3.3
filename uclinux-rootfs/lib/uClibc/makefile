include $(ROOTDIR)/config.arch

unexport MAKEFILE_LIST

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

.PHONY: all
all::
	$(MAKE) -f Makefile
	$(MAKE) -f Makefile utils

.PHONY: romfs
romfs::
	mkdir -p $(ROMFSDIR)/lib
	cp -df lib/*.so* $(ROMFSDIR)/lib/
	$(ROMFSINST) -e CONFIG_LIB_UCLIBC_LDD utils/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_UCLIBC_LDCONFIG utils/ldconfig /sbin/ldconfig

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
