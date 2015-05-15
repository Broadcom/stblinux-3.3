#
# I2C tools
#
# General settings
#
# for $(CROSS), $(CC), $(STRIPTOOL)


include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

I2C_TOOLS_PATH		:= tools/i2cdetect tools/i2cset tools/i2cget tools/i2cdump 

.PHONY: all
all::
	$(MAKE) -f Makefile CC=$(CC) CFLAGS=$(CFLAGS)

.PHONY: romfs
romfs::
	for x in $(I2C_TOOLS_PATH); do \
		$(STRIPTOOL) $$x; \
		$(ROMFSINST) -e CONFIG_USER_I2C_TOOLS $$x /bin/ || \
			exit 1; \
	done

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
