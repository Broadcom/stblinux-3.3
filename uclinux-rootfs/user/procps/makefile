#
# General settings
#

include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

#
# Settings that are specific to this package
#

CPPFLAGS		:= -I$(ROOTDIR)/lib/ncurses/include
CURSES			:= -L$(ROOTDIR)/lib/ncurses/lib -lncurses

#
# Build targets
#

.PHONY: all
all::
	$(MAKE) -f Makefile CPPFLAGS="$(CPPFLAGS)" CURSES="$(CURSES)"

.PHONY: romfs
romfs::
	$(ROMFSINST) proc/libproc-3.2.8.so /lib/libproc-3.2.8.so
	$(ROMFSINST) -e CONFIG_USER_PROCPS_FREE /bin/free
	$(ROMFSINST) -e CONFIG_USER_PROCPS_KILL /bin/kill
	$(ROMFSINST) -e CONFIG_USER_PROCPS_PGREP /bin/pgrep
	$(ROMFSINST) -e CONFIG_USER_PROCPS_PKILL /bin/pkill
	$(ROMFSINST) -e CONFIG_USER_PROCPS_PS ps/ps /bin/ps
	$(ROMFSINST) -e CONFIG_USER_PROCPS_SNICE /bin/snice
	$(ROMFSINST) -e CONFIG_USER_PROCPS_SYSCTL /bin/sysctl
	$(ROMFSINST) -e CONFIG_USER_PROCPS_TLOAD /bin/tload
	$(ROMFSINST) -e CONFIG_USER_PROCPS_TOP /bin/top
	$(ROMFSINST) -e CONFIG_USER_PROCPS_UPTIME /bin/uptime
	$(ROMFSINST) -e CONFIG_USER_PROCPS_VMSTAT /bin/vmstat
	$(ROMFSINST) -e CONFIG_USER_PROCPS_W /bin/w

.PHONY: clean distclean
clean distclean::
	$(MAKE) -f Makefile clean

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
