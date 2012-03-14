include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip

CFLAGS			:= -I$(ROOTDIR)/lib/popt
CFLAGS			+= -I$(ROOTDIR)/lib/zlib
CFLAGS			+= -I$(ROOTDIR)/lib/binutils/include
CFLAGS			+= -I$(ROOTDIR)/lib/binutils/bfd

CXXFLAGS		:= $(CFLAGS)

LDFLAGS			:= -L$(ROOTDIR)/lib/popt/.libs
LDFLAGS			+= -L$(ROOTDIR)/lib/zlib
LDFLAGS			+= -L$(ROOTDIR)/lib/binutils/bfd
LDFLAGS			+= -L$(ROOTDIR)/lib/binutils/libiberty
LDFLAGS			+= -L$(ROOTDIR)/lib/binutils/intl
LDFLAGS			+= -lintl

export CC CFLAGS CXXFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux --disable-shared \
			--prefix=/usr --with-kernel-support \
			CXXFLAGS="$(CXXFLAGS)" \
			CFLAGS="$(CFLAGS)" LDFLAGS="$(LDFLAGS)" || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) pp/opannotate pp/oparchive pp/opgprof pp/opreport \
		utils/ophelp daemon/oprofiled
	chmod 0755 utils/opcontrol
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE pp/opannotate \
		/bin/opannotate
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE pp/opgprof \
		/bin/opgprof
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE pp/opreport \
		/bin/opreport
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE utils/ophelp \
		/bin/ophelp
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE utils/opcontrol \
		/bin/opcontrol
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE daemon/oprofiled \
		/bin/oprofiled
	mkdir -p $(ROMFSDIR)/usr/share/oprofile
	$(ROMFSINST) -e CONFIG_USER_PROFILE_OPROFILE events/mips \
		/usr/share/oprofile/mips

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
