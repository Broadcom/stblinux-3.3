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
			--disable-shared --enable-static || exit 1 ; \
	fi
	$(MAKE) -f Makefile MANS=

.PHONY: romfs
romfs::
	$(STRIP) iptables-multi ip6tables-multi
	$(ROMFSINST) -e CONFIG_USER_IPTABLES_IPTABLES \
		iptables-multi /sbin/iptables
	$(ROMFSINST) -e CONFIG_USER_IPTABLES_IPTABLES \
		ip6tables-multi /sbin/ip6tables

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
