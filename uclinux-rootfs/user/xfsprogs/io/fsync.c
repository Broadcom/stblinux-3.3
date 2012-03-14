/*
 * Copyright (c) 2003 Silicon Graphics, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.  Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Contact information: Silicon Graphics, Inc., 1600 Amphitheatre Pkwy,
 * Mountain View, CA  94043, or:
 *
 * http://www.sgi.com
 *
 * For further information regarding this notice, see:
 *
 * http://oss.sgi.com/projects/GenInfo/SGIGPLNoticeExplan/
 */

#include <xfs/libxfs.h>
#include "command.h"
#include "init.h"

static cmdinfo_t fsync_cmd;
static cmdinfo_t fdatasync_cmd;

static int
fsync_f(
	int			argc,
	char			**argv)
{
	if (fsync(fdesc) < 0) {
		perror("fsync");
		return 0;
	}
	return 0;
}

static int
fdatasync_f(
	int			argc,
	char			**argv)
{
	if (fdatasync(fdesc) < 0) {
		perror("fdatasync");
		return 0;
	}
	return 0;
}

void
fsync_init(void)
{
	fsync_cmd.name = _("fsync");
	fsync_cmd.altname = _("s");
	fsync_cmd.cfunc = fsync_f;
	fsync_cmd.oneline =
		_("calls fsync(2) to flush all in-core file state to disk");

	fdatasync_cmd.name = _("fdatasync");
	fdatasync_cmd.altname = _("ds");
	fdatasync_cmd.cfunc = fdatasync_f;
	fdatasync_cmd.oneline =
		_("calls fdatasync(2) to flush the files in-core data to disk");

	add_command(&fsync_cmd);
	add_command(&fdatasync_cmd);
}
