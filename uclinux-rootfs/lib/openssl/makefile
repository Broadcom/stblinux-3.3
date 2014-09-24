include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e .configured ]; then \
		./Configure linux-generic32 || exit 1 ; \
	fi
	touch .configured
	$(MAKE) -f Makefile
	rm -f lib
	ln -s . lib

.PHONY: romfs
romfs::

.PHONY: clean distclean
clean distclean::
	[ ! -f .configured ] || $(MAKE) -f Makefile clean
	rm -f .configured

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
