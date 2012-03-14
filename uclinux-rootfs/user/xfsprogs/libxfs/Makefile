#
# Copyright (c) 2000-2003 Silicon Graphics, Inc.  All Rights Reserved.
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

LTLIBRARY = libxfs.la
LT_CURRENT = 0
LT_REVISION = 0
LT_AGE = 0

HFILES = xfs.h init.h
CFILES = bit.c init.c logitem.c rdwr.c trans.c util.c \
	xfs_alloc.c xfs_ialloc.c xfs_rtalloc.c \
	xfs_inode.c xfs_btree.c xfs_alloc_btree.c xfs_ialloc_btree.c \
	xfs_bmap_btree.c xfs_da_btree.c xfs_dir.c xfs_dir_leaf.c \
	xfs_dir2.c xfs_dir2_leaf.c xfs_attr_leaf.c xfs_dir2_block.c \
	xfs_dir2_node.c xfs_dir2_data.c xfs_dir2_sf.c xfs_bmap.c \
	xfs_mount.c xfs_trans.c

CFILES += $(PKG_PLATFORM).c
PCFILES = darwin.c freebsd.c irix.c linux.c
LSRCFILES = $(shell echo $(PCFILES) | sed -e "s/$(PKG_PLATFORM).c//g")

#
# Tracing flags:
# -DIO_DEBUG		reads and writes of buffers
# -DMEM_DEBUG		all zone memory use
# -DLI_DEBUG		log item (ino/buf) manipulation
# -DXACT_DEBUG		transaction state changes
#
LCFLAGS += -I.

# don't try linking xfs_repair with a debug libxfs.
DEBUG = -DNDEBUG

default:  $(LTLIBRARY)

include $(BUILDRULES)

install: default

install-dev: default
	$(INSTALL_LTLIB_STATIC)
