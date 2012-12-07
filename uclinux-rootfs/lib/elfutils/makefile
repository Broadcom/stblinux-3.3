include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

MY_LIBS = -lintl
MY_LDFLAGS = -L$(ROOTDIR)/lib/gettext/intl/.libs -lintl
UCLIBC_INC = -I$(ROOTDIR)/lib/libuargp -I$(ROOTDIR)/lib/gettext/intl
MY_CFLAGS = $(UCLIBC_INC)

.PHONY: all
all:
	if [ ! -e Makefile ]; then \
		CFLAGS="$(MY_CFLAGS)" \
		LDFLAGS="$(MY_LDFLAGS)" \
		./configure --host=$(MACHINE)-linux --prefix=$(ROMFSDIR) \
		--disable-nls --disable-werror; \
	fi
	make -f Makefile SUBDIRS=libelf

.PHONY: romfs
romfs:
	-$(RM) -f $(ROMFSDIR)/lib/libelf.so*
	cp -af libelf/libelf.so libelf/libelf.so*[0-9] \
		$(ROMFSDIR)/lib/
	$(STRIP) $(ROMFSDIR)/lib/libelf.so

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@

