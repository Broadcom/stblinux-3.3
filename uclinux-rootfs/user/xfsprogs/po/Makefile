#
# Copyright (c) 2001-2003 Silicon Graphics, Inc.  All Rights Reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of version 2 of the GNU General Public License as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it would be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
# Further, this software is distributed without any warranty that it is
# free of the rightful claim of any third person regarding infringement
# or the like.  Any license provided herein, whether implied or
# otherwise, applies only to this software file.  Patent licenses, if
# any, provided herein do not apply to combinations of this program with
# other software, or any other product whatsoever.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write the Free Software Foundation, Inc., 59
# Temple Place - Suite 330, Boston MA 02111-1307, USA.
#
# Contact information: Silicon Graphics, Inc., 1600 Amphitheatre Pkwy,
# Mountain View, CA  94043, or:
#
# http://www.sgi.com
#
# For further information regarding this notice, see:
#
# http://oss.sgi.com/projects/GenInfo/SGIGPLNoticeExplan/
#

TOPDIR = ..
include $(TOPDIR)/include/builddefs

# Currently LINGUAS is undefined, so buildmacros provides no targets.
LINGUAS =
LSRCFILES = $(LINGUAS:%=%.po) $(PKG_NAME).pot

# TODO: db/ logprint/
XGETTEXTFILES =	\
		$(TOPDIR)/freeze/xfs_freeze.c \
		$(TOPDIR)/growfs/explore.c \
		$(TOPDIR)/growfs/xfs_growfs.c \
		$(TOPDIR)/imap/xfs_imap.c \
		$(TOPDIR)/io/bmap.c \
		$(TOPDIR)/io/command.c \
		$(TOPDIR)/io/fsync.c \
		$(TOPDIR)/io/help.c \
		$(TOPDIR)/io/init.c \
		$(TOPDIR)/io/open.c \
		$(TOPDIR)/io/pread.c \
		$(TOPDIR)/io/prealloc.c \
		$(TOPDIR)/io/pwrite.c \
		$(TOPDIR)/io/quit.c \
		$(TOPDIR)/io/resblks.c \
		$(TOPDIR)/io/truncate.c \
		$(TOPDIR)/mkfile/xfs_mkfile.c \
		$(TOPDIR)/mkfs/proto.c \
		$(TOPDIR)/mkfs/xfs_mkfs.c \
		$(TOPDIR)/libdisk/drivers.c \
		$(TOPDIR)/libdisk/lvm.c \
		$(TOPDIR)/libdisk/md.c \
		$(TOPDIR)/libxfs/darwin.c \
		$(TOPDIR)/libxfs/freebsd.c \
		$(TOPDIR)/libxfs/init.c \
		$(TOPDIR)/libxfs/irix.c \
		$(TOPDIR)/libxfs/linux.c \
		$(TOPDIR)/libxfs/rdwr.c \
		$(TOPDIR)/libxfs/trans.c \
		$(TOPDIR)/libxfs/util.c \
		$(TOPDIR)/libxlog/util.c \
		$(TOPDIR)/repair/agheader.c \
		$(TOPDIR)/repair/attr_repair.c \
		$(TOPDIR)/repair/avl.c \
		$(TOPDIR)/repair/avl64.c \
		$(TOPDIR)/repair/bmap.c \
		$(TOPDIR)/repair/dino_chunks.c \
		$(TOPDIR)/repair/dinode.c \
		$(TOPDIR)/repair/dir.c \
		$(TOPDIR)/repair/dir2.c \
		$(TOPDIR)/repair/dir_stack.c \
		$(TOPDIR)/repair/incore.c \
		$(TOPDIR)/repair/incore_ext.c \
		$(TOPDIR)/repair/incore_ino.c \
		$(TOPDIR)/repair/init.c \
		$(TOPDIR)/repair/io.c \
		$(TOPDIR)/repair/phase1.c \
		$(TOPDIR)/repair/phase2.c \
		$(TOPDIR)/repair/phase3.c \
		$(TOPDIR)/repair/phase4.c \
		$(TOPDIR)/repair/phase5.c \
		$(TOPDIR)/repair/phase6.c \
		$(TOPDIR)/repair/phase7.c \
		$(TOPDIR)/repair/rt.c \
		$(TOPDIR)/repair/sb.c \
		$(TOPDIR)/repair/scan.c \
		$(TOPDIR)/repair/versions.c \
		$(TOPDIR)/repair/xfs_repair.c \
		$(TOPDIR)/rtcp/xfs_rtcp.c

default: $(LINGUAS:%=%.mo)

include $(BUILDRULES)

install: default
	$(INSTALL_LINGUAS)

install-dev install-lib:
