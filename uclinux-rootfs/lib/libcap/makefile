#
# General settings
#

# for $(CROSS), $(CC), $(STRIPTOOL)
include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CFLAGS			:= -Os

export CC CFLAGS

#
# Settings that are specific to this package
#


#
# Build targets
#

.PHONY: all
all::
	cp -f _makenames.sh libcap/_makenames
	$(MAKE) -f Makefile CC=$(CC) AR=$(AR) RANLIB=$(RANLIB) BUILD_CC=true \
		LIBATTR=no -C libcap libcap.a

.PHONY: romfs
romfs::

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
