#
# General settings
#

# for $(CROSS), $(CC), $(STRIPTOOL)
include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CFLAGS			:= -Os -I.


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
	$(ROMFSINST) -e CONFIG_LIB_UARGP /lib/libuargp.so.1.0.1
	$(ROMFSINST) -e CONFIG_LIB_UARGP -s libuargp.so.1.0.1 \
		/lib/libuargp.so

.PHONY: clean distclean
clean distclean::
	$(MAKE) -f Makefile clean

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
