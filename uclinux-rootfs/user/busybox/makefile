#
# General settings
#

include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os -fno-strict-aliasing

export CC CFLAGS

#
# Settings that are specific to this package
#

#
# Build targets
#

.PHONY: all
all::
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(MAKE) -f Makefile CONFIG_PREFIX=$(ROOTDIR)/romfs install

.PHONY: clean distclean
clean distclean::
	$(MAKE) -f Makefile clean

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
