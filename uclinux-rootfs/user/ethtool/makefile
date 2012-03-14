include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux \
			ac_cv_func_malloc_0_nonnull=yes || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) ethtool 
	$(ROMFSINST) -e CONFIG_USER_ETHTOOL ethtool /bin/ethtool

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
