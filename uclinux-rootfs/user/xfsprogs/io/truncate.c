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
#include "input.h"
#include "init.h"

static cmdinfo_t truncate_cmd;

static int
truncate_f(
	int			argc,
	char			**argv)
{
	off64_t			offset;

	offset = cvtnum(fgeom.blocksize, fgeom.sectsize, argv[1]);
	if (offset < 0) {
		printf(_("non-numeric truncate argument -- %s\n"), argv[1]);
		return 0;
	}

	if (ftruncate64(fdesc, offset) < 0) {
		perror("ftruncate");
		return 0;
	}
	return 0;
}

void
truncate_init(void)
{
	truncate_cmd.name = _("truncate");
	truncate_cmd.cfunc = truncate_f;
	truncate_cmd.argmin = 1;
	truncate_cmd.argmax = 1;
	truncate_cmd.args = _("off");
	truncate_cmd.oneline =
		_("truncates the current file at the given offset");

	add_command(&truncate_cmd);
}
