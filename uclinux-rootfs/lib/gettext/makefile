include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all:
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux --prefix=$(ROMFSDIR); \
	fi
	$(MAKE) -f Makefile SUBDIRS=intl

.PHONY: romfs
romfs:
	make -f Makefile SUBDIRS=intl install-exec
	rm -fr $(ROMFSDIR)/lib/libintl.a

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@

