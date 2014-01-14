include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os
SHELL			:= bash
VPATH			:= $(GCCSYSROOT)/lib:$(VPATH)

.LIBPATTERNS += $(shell echo lib%.so.{0..9})

export CC CFLAGS VPATH .LIBPATTERNS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux \
			--with-zlib=$(ROOTDIR)/lib/zlib || exit 1 ; \
	fi
	$(MAKE) -f Makefile PROGRAMS="dropbear dbclient dropbearkey scp"

.PHONY: romfs
romfs::
	$(STRIP) dropbear dropbearkey dbclient scp
	$(ROMFSINST) -e CONFIG_USER_SSH_SSHD /sbin/dropbear
	$(ROMFSINST) -e CONFIG_USER_SSH_SSHKEYGEN /bin/dropbearkey
	$(ROMFSINST) -e CONFIG_USER_SSH_SSH /bin/dbclient
	$(ROMFSINST) -e CONFIG_USER_SSH_SSH /bin/scp
	$(ROMFSINST) -e CONFIG_USER_SSH_SSHD /bin/scp

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
