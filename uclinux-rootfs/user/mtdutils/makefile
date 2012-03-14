#
# General settings
#

# for $(CROSS), $(CC), $(STRIPTOOL)
include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CFLAGS			:= -Os -DCONFIG_BRCMNAND_MTD_EXTENSION

export CC CFLAGS

#
# Settings that are specific to this package
#

MTD_TARGETS		:= flash_erase nanddump flash_unlock \
				flash_lock mtd_debug nandwrite \
				nandtest sumtool

UBI_TARGETS		:= ubiupdatevol ubimkvol ubirmvol ubicrc32 ubinfo \
				ubiattach ubidetach ubinize ubiformat \
				ubirename mtdinfo ubirsvol

BUILDDIR		:= $(CURDIR)
PWD			:= $(CURDIR)
MTD_PATHS		:= $(patsubst %,$(BUILDDIR)/%,$(MTD_TARGETS))
UBI_PATHS		:= $(patsubst %,$(BUILDDIR)/ubi-utils/%,$(UBI_TARGETS))

export BUILDDIR PWD

#
# Build targets
#

.PHONY: all
all::
	$(MAKE) -f Makefile $(MTD_PATHS) $(UBI_PATHS)

.PHONY: romfs
romfs::
	$(STRIPTOOL) $(MTD_PATHS) $(UBI_PATHS)
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_ERASE /bin/flash_erase
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_ERASE /bin/flash_eraseall
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_UNLOCK /bin/flash_unlock
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_LOCK /bin/flash_lock
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_NANDDUMP /bin/nanddump
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_NANDTEST /bin/nandtest
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_NANDWRITE /bin/nandwrite
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_MTD_DEBUG /bin/mtd_debug
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_SUMTOOL /bin/sumtool
	for x in $(UBI_PATHS); do \
		$(ROMFSINST) -e CONFIG_USER_MTDUTILS_UBI_UTILS $$x /bin/ || \
			exit 1; \
	done

.PHONY: clean distclean
clean distclean::
	$(MAKE) -f Makefile clean

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
