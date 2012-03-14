include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux || exit 1 ; \
	fi
	cp -f scancode.c.saved scancode.c
	touch scancode.c
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) mawk
	$(ROMFSINST) -e CONFIG_USER_MAWK_AWK mawk /bin/awk

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
