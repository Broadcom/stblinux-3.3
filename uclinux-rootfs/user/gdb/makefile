include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

CPPFLAGS		:= -I$(ROOTDIR)/user/bash/build/lib/termcap
LDFLAGS			:= -L$(ROOTDIR)/user/bash/build/lib/termcap

export CPPFLAGS LDFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux --disable-nls \
			--without-included-gettext --without-uiout \
			--disable-tui --disable-gdbtk --without-x || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) gdb/gdb
	$(ROMFSINST) -e CONFIG_USER_GDB_GDB gdb/gdb /bin/gdb

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
