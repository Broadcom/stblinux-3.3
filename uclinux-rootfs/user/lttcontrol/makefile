include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:=

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux --target=$(MACHINE)-linux \
			--disable-shared --prefix=/ || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) lttctl/lttctl lttd/lttd
	$(ROMFSINST) -e CONFIG_USER_LTT_CONTROL lttctl/ltt-armall \
		/bin/ltt-armall
	$(ROMFSINST) -e CONFIG_USER_LTT_CONTROL lttctl/ltt-armtap \
		/bin/ltt-armtap
	$(ROMFSINST) -e CONFIG_USER_LTT_CONTROL lttctl/ltt-disarmall \
		/bin/ltt-disarmall
	$(ROMFSINST) -e CONFIG_USER_LTT_CONTROL lttctl/ltt-disarmtap \
		/bin/ltt-disarmtap
	$(ROMFSINST) -e CONFIG_USER_LTT_CONTROL lttctl/lttctl /bin/lttctl
	$(ROMFSINST) -e CONFIG_USER_LTT_CONTROL lttd/lttd /bin/lttd

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
