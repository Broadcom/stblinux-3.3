include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip

CFLAGS			:= -include locale.h

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux CFLAGS="$(CFLAGS)" \
			|| exit 1 ; \
	fi
	$(MAKE) -f Makefile SUBDIRS="fdisk"

.PHONY: romfs
romfs::
	$(STRIP) fdisk/fdisk fdisk/sfdisk
	$(ROMFSINST) -e CONFIG_USER_FDISK_FDISK fdisk/fdisk \
		/bin/fdisk
	$(ROMFSINST) -e CONFIG_USER_FDISK_FDISK fdisk/sfdisk \
		/bin/sfdisk

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
