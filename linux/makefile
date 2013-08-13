BOGUS_TARGETS		:= FORCE makefile .config

-include .config

ifeq ($(CONFIG_MIPS),y)
export ARCH		:= mips
else
ifeq ($(CONFIG_ARM),y)
export ARCH		:= arm
endif
endif

# don't override the build target name if an external module source
# directory was specified
EXT_MODULE_BUILD	:= n

ifdef KBUILD_EXTMOD
EXT_MODULE_BUILD	:= y
endif

ifdef M
EXT_MODULE_BUILD	:= y
endif

.PHONY: default
default::
ifeq ($(EXT_MODULE_BUILD),y)
	$(MAKE) -f Makefile
else
ifeq ($(ARCH),mips)
	$(MAKE) -f Makefile vmlinux
	gzip < vmlinux > vmlinuz
else
ifeq ($(ARCH),arm)
	$(MAKE) -f Makefile zImage
ifeq ($(CONFIG_ARM_APPENDED_DTB),y)
	rm -f arch/arm/boot/*.dtb
	$(MAKE) -f Makefile dtbs
	cat arch/arm/boot/zImage arch/arm/boot/*.dtb > vmlinuz
else
	cp arch/arm/boot/zImage vmlinuz
endif
else
	$(MAKE) -f Makefile
endif
endif
endif

.PHONY: %_defconfig
%_defconfig::
	@if [ -e arch/mips/configs/$@ ]; then \
		$(MAKE) -f Makefile $@ ARCH=mips || exit 1; \
	elif [ -e arch/arm/configs/$@ ]; then \
		$(MAKE) -f Makefile $@ ARCH=arm || exit 1; \
	else \
		$(MAKE) -f Makefile $@ || exit 1; \
	fi

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
# Since this file doesn't contain dependency information, don't parallelize
# here.
.NOTPARALLEL: %
%:: FORCE
	$(MAKE) -f Makefile $@
