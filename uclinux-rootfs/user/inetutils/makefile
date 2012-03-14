include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

# NOTE: many of the inetutils components do not currently build
SUBDIRS			:= libinetutils syslogd

.PHONY: all
all::
	if [ ! -e build/Makefile ]; then \
		mkdir -p build; \
		cd build; \
		../configure --host=$(MACHINE)-linux \
			--sysconfdir=/etc --localstatedir=/var || exit 1; \
	fi
	$(MAKE) -j1 -C build SUBDIRS="$(SUBDIRS)"

.PHONY: distclean
distclean::
	rm -rf build

.PHONY: romfs
romfs::
	$(ROMFSINST) -e CONFIG_USER_INETUTILS_SYSLOGD build/syslogd/syslogd \
		/bin/syslogd

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -C build $@
