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

# TODO: mmap, file locking, fadvise

LTCOMMAND = xfs_io
HFILES = command.h input.h init.h
CFILES = command.c input.c init.c \
	bmap.c fsync.c help.c open.c pread.c prealloc.c pwrite.c \
	quit.c resblks.c truncate.c
LSRCFILES = xfs_bmap.sh

ifeq ($(ENABLE_READLINE),yes)
LLDLIBS += $(LIBREADLINE)
CFLAGS += -DENABLE_READLINE
endif

ifeq ($(ENABLE_EDITLINE),yes)
LLDLIBS += $(LIBEDITLINE)
CFLAGS += -DENABLE_EDITLINE
endif

default: $(LTCOMMAND)

include $(BUILDRULES)

install: default
	$(INSTALL) -m 755 -d $(PKG_BIN_DIR)
	$(LTINSTALL) -m 755 $(LTCOMMAND) $(PKG_BIN_DIR)
	$(LTINSTALL) -m 755 xfs_bmap.sh $(PKG_BIN_DIR)/xfs_bmap
install-dev:
