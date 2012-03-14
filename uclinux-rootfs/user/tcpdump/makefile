include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		ln -sf $(ROOTDIR)/lib/libpcap . ; \
		./configure --host=$(MACHINE)-linux \
			--without-crypto --disable-ipv6 \
			ac_cv_linux_vers=2 || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) tcpdump
	$(ROMFSINST) -e CONFIG_USER_TCPDUMP_TCPDUMP /bin/tcpdump

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
