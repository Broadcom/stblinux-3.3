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

static cmdinfo_t resblks_cmd;
static int resblks_f(int argc, char **argv);

static int
resblks_f(
	int			argc,
	char			**argv)
{
	xfs_fsop_resblks_t	res;
	long long		blks;

	if (argc == 2) {
		blks = cvtnum(fgeom.blocksize, fgeom.sectsize, argv[1]);
		if (blks < 0) {
			printf(_("non-numeric argument -- %s\n"), argv[1]);
			return 0;
		}
		res.resblks = blks;
		if (xfsctl(fname, fdesc, XFS_IOC_SET_RESBLKS, &res) < 0) {
			perror("xfsctl(XFS_IOC_SET_RESBLKS)");
			return 0;
		}
	} else if (xfsctl(fname, fdesc, XFS_IOC_GET_RESBLKS, &res) < 0) {
		perror("xfsctl(XFS_IOC_GET_RESBLKS)");
		return 0;
	}
	printf(_("reserved blocks = %llu\n"),
			(unsigned long long) res.resblks);
	printf(_("available reserved blocks = %llu\n"),
			(unsigned long long) res.resblks_avail);
	return 0;
}

void
resblks_init(void)
{
	resblks_cmd.name = _("resblks");
	resblks_cmd.cfunc = resblks_f;
	resblks_cmd.argmin = 0;
	resblks_cmd.argmax = 1;
	resblks_cmd.args = _("[blocks]");
	resblks_cmd.oneline =
		_("get and/or set count of reserved filesystem blocks");

	add_command(&resblks_cmd);
}
