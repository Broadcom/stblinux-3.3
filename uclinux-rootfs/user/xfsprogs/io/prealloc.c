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

static cmdinfo_t allocsp_cmd;
static cmdinfo_t freesp_cmd;
static cmdinfo_t resvsp_cmd;
static cmdinfo_t unresvsp_cmd;

static int
offset_length(
	char		*offset,
	char		*length,
	xfs_flock64_t	*segment)
{
	memset(segment, 0, sizeof(*segment));
	segment->l_whence = SEEK_SET;
	segment->l_start = cvtnum(fgeom.blocksize, fgeom.sectsize, offset);
	if (segment->l_start < 0) {
		printf(_("non-numeric offset argument -- %s\n"), offset);
		return 0;
	}
	segment->l_len = cvtnum(fgeom.blocksize, fgeom.sectsize, length);
	if (segment->l_len < 0) {
		printf(_("non-numeric length argument -- %s\n"), length);
		return 0;
	}
	return 1;
}

static int
allocsp_f(
	int		argc,
	char		**argv)
{
	xfs_flock64_t	segment;

	if (!offset_length(argv[1], argv[2], &segment))
		return 0;

	if (xfsctl(fname, fdesc, XFS_IOC_ALLOCSP64, &segment) < 0) {
		perror("xfsctl(XFS_IOC_ALLOCSP64)");
		return 0;
	}
	return 0;
}

static int
freesp_f(
	int		argc,
	char		**argv)
{
	xfs_flock64_t	segment;

	if (!offset_length(argv[1], argv[2], &segment))
		return 0;

	if (xfsctl(fname, fdesc, XFS_IOC_FREESP64, &segment) < 0) {
		perror("xfsctl(XFS_IOC_FREESP64)");
		return 0;
	}
	return 0;
}

static int
resvsp_f(
	int		argc,
	char		**argv)
{
	xfs_flock64_t	segment;

	if (!offset_length(argv[1], argv[2], &segment))
		return 0;

	if (xfsctl(fname, fdesc, XFS_IOC_RESVSP64, &segment) < 0) {
		perror("xfsctl(XFS_IOC_RESVSP64)");
		return 0;
	}
	return 0;
}

static int
unresvsp_f(
	int		argc,
	char		**argv)
{
	xfs_flock64_t	segment;

	if (!offset_length(argv[1], argv[2], &segment))
		return 0;

	if (xfsctl(fname, fdesc, XFS_IOC_UNRESVSP64, &segment) < 0) {
		perror("xfsctl(XFS_IOC_UNRESVSP64)");
		return 0;
	}
	return 0;
}

void
prealloc_init(void)
{
	allocsp_cmd.name = _("allocsp");
	allocsp_cmd.cfunc = allocsp_f;
	allocsp_cmd.argmin = 2;
	allocsp_cmd.argmax = 2;
	allocsp_cmd.args = _("off len");
	allocsp_cmd.oneline = _("allocates zeroed space for part of a file");

	freesp_cmd.name = _("freesp");
	freesp_cmd.cfunc = freesp_f;
	freesp_cmd.argmin = 2;
	freesp_cmd.argmax = 2;
	freesp_cmd.args = _("off len");
	freesp_cmd.oneline = _("frees space associated with part of a file");

	resvsp_cmd.name = _("resvsp");
	resvsp_cmd.cfunc = resvsp_f;
	resvsp_cmd.argmin = 2;
	resvsp_cmd.argmax = 2;
	resvsp_cmd.args = _("off len");
	resvsp_cmd.oneline =
		_("reserves space associated with part of a file");

	unresvsp_cmd.name = _("unresvsp");
	unresvsp_cmd.cfunc = unresvsp_f;
	unresvsp_cmd.argmin = 2;
	unresvsp_cmd.argmax = 2;
	unresvsp_cmd.args = _("off len");
	unresvsp_cmd.oneline =
		_("frees reserved space associated with part of a file");

	add_command(&allocsp_cmd);
	add_command(&freesp_cmd);
	add_command(&resvsp_cmd);
	add_command(&unresvsp_cmd);
}
