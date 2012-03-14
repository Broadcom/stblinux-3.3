#
# General settings
#

# for $(CROSS), $(CC), $(STRIPTOOL)
include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch
SHELL			:= bash

CFLAGS			:= -Os

export CC CFLAGS

#
# Settings that are specific to this package
#

CPPFLAGS		:= -I$(ROOTDIR)/lib/libcap/libcap/include
LDFLAGS			:= -L$(ROOTDIR)/lib/libcap/libcap

LXCPROGS		:= lxc-start lxc-info lxc-attach lxc-freeze \
	lxc-monitor lxc-execute lxc-setuid lxc-destroy lxc-stop \
	lxc-setcap lxc-netstat lxc-checkconfig lxc-ls lxc-wait \
	lxc-unfreeze lxc-restart lxc-cgroup lxc-create lxc-unshare \
	lxc-checkpoint lxc-kill lxc-version lxc-console

export CPPFLAGS LDFLAGS

#
# Build targets
#

.PHONY: all
all::
	if [ ! -e build/Makefile ]; then \
		mkdir -p build; \
		cd build; \
		../configure --host=$(MACHINE)-linux --prefix= \
			--disable-doc || exit 1 ; \
	fi
	$(MAKE) -C build

.PHONY: romfs
romfs::
	rm -rf build/romfs
	mkdir -p build/romfs
	$(MAKE) -C build install DESTDIR=$(shell pwd)/build/romfs
	cp -rf build/romfs/bin/* $(ROMFSDIR)/bin/
	cp -rf build/romfs/lib/* $(ROMFSDIR)/lib/
	cp -rf brcmscripts/* $(ROMFSDIR)/
	mkdir -p $(ROMFSDIR)/{cgroup,var/lib/lxc}

.PHONY: clean
clean::
	$(MAKE) -C build clean

.PHONY: distclean
distclean::
	rm -rf build

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
