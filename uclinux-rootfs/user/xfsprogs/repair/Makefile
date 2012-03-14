#
# Copyright (c) 2000-2002 Silicon Graphics, Inc.  All Rights Reserved.
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

LTCOMMAND = xfs_repair

HFILES = agheader.h attr_repair.h avl.h avl64.h bmap.h dinode.h dir.h \
	dir2.h dir_stack.h err_protos.h globals.h incore.h protos.h rt.h \
	scan.h versions.h

CFILES = agheader.c attr_repair.c avl.c avl64.c bmap.c dino_chunks.c \
	dinode.c dir.c dir2.c dir_stack.c globals.c incore.c \
	incore_bmc.c init.c incore_ext.c incore_ino.c io.c phase1.c \
	phase2.c phase3.c phase4.c phase5.c phase6.c phase7.c rt.c sb.c \
	scan.c versions.c xfs_repair.c

LLDLIBS = $(LIBXFS) $(LIBXLOG) $(LIBUUID)
LTDEPENDENCIES = $(LIBXFS) $(LIBXLOG)
LLDFLAGS = -static

default: $(LTCOMMAND)

include $(BUILDRULES)

#
# Tracing flags:
# -DXR_BMAP_DBG		incore block bitmap debugging
# -DXR_INODE_TRACE	inode processing
# -DXR_BMAP_TRACE	bmap btree processing
# -DXR_DIR_TRACE	directory processing
# -DXR_DUP_TRACE	duplicate extent processing
# -DXR_BCNT_TRACE	incore bcnt freespace btree building
# -DXR_BLD_FREE_TRACE	building on-disk freespace (bcnt/bno) btrees
# -DXR_BLD_INO_TRACE	building on-disk inode allocation btrees
# -DXR_BLD_ADD_EXTENT	track phase 5 block extent creation
# -DXR_BCKPTR_DBG	parent list debugging info
#
#CFLAGS += ...

install: default
	$(INSTALL) -m 755 -d $(PKG_SBIN_DIR)
	$(LTINSTALL) -m 755 $(LTCOMMAND) $(PKG_SBIN_DIR)
install-dev:
