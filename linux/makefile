BOGUS_TARGETS		:= FORCE makefile .config

-include .config

ifeq ($(CONFIG_MIPS),y)
export ARCH		:= mips
else
ifeq ($(CONFIG_ARM),y)
export ARCH		:= arm
endif
endif

.PHONY: default
default::
ifeq ($(ARCH),mips)
	$(MAKE) -f Makefile vmlinux
	gzip < vmlinux > vmlinuz
else
ifeq ($(ARCH),arm)
	$(MAKE) -f Makefile zImage
	cp arch/arm/boot/zImage vmlinuz
else
	$(MAKE) -f Makefile
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
%:: FORCE
	$(MAKE) -f Makefile $@
