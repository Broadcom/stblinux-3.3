#
# General settings
#
# for $(CROSS), $(CC), $(STRIPTOOL)

include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch
SHELL			:= bash
CFLAGS			:= -Os

XFSPROGS_DIR		:= $(shell pwd)

CFLAGS			+= -I$(XFSPROGS_DIR) -I$(ROOTDIR)/user/e2fsprogs/lib
LDFLAGS			+= -L$(ROOTDIR)/user/e2fsprogs/lib 

export CC CFLAGS

XFSPROGS_TOOLS_PATH	:= copy/xfs_copy db/xfs_db estimate/xfs_estimate  \
			fsck/xfs_fsck.sh fsr/xfs_fsr growfs/xfs_growfs  \
			io/xfs_io logprint/xfs_logprint mkfs/mkfs.xfs  \
			quota/xfs_quota mdrestore/xfs_mdrestore  \
			repair/xfs_repair rtcp/xfs_rtcp 

TOOL_SUBDIRS		:= $(dir $(XFSPROGS_TOOLS_PATH))

#
# These are the options passed to the configure script called from Makefile
#
LOCAL_CONFIGURE_OPTIONS	:= --host=$(MACHINE)-linux \
			--disable-shared --enable-gettext=no

#
# Build targets
#
.PHONY: all
all::
	$(MAKE) -j1 -f Makefile TOOL_SUBDIRS="$(TOOL_SUBDIRS)" \
		LOCAL_CONFIGURE_OPTIONS="$(LOCAL_CONFIGURE_OPTIONS)" \
		ac_cv_header_aio_h=yes \
		ac_cv_lib_rt_lio_listio=yes

.PHONY: distclean
distclean::
	$(MAKE) -f Makefile distclean

.PHONY: romfs
romfs::
	for x in $(XFSPROGS_TOOLS_PATH); do \
		[[ "$$x" != *.sh ]] && $(STRIPTOOL) $$x; \
		$(ROMFSINST) -e CONFIG_USER_XFS_XFSPROGS $$x /bin/ || \
			exit 1; \
	done

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
