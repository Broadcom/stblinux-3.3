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

cmdinfo_t	*cmdtab;
int		ncmds;

static int
cmd_compare(const void *a, const void *b)
{
	return strcmp(((const cmdinfo_t *)a)->name,
		      ((const cmdinfo_t *)b)->name);
}

void
add_command(
	const cmdinfo_t	*ci)
{
	cmdtab = realloc((void *)cmdtab, ++ncmds * sizeof(*cmdtab));
	cmdtab[ncmds - 1] = *ci;
	qsort(cmdtab, ncmds, sizeof(*cmdtab), cmd_compare);
}

int
command(
	int		argc,
	char		**argv)
{
	char		*cmd;
	const cmdinfo_t	*ct;

	cmd = argv[0];
	ct = find_command(cmd);
	if (!ct) {
		fprintf(stderr, _("command \"%s\" not found\n"), cmd);
		return 0;
	}
	if (argc-1 < ct->argmin || (ct->argmax != -1 && argc-1 > ct->argmax)) {
		if (ct->argmax == -1)
			fprintf(stderr,
	_("bad argument count %d to %s, expected at least %d arguments\n"),
				argc-1, cmd, ct->argmin);
		else if (ct->argmin == ct->argmax)
			fprintf(stderr,
	_("bad argument count %d to %s, expected %d arguments\n"),
				argc-1, cmd, ct->argmin);
		else
			fprintf(stderr,
	_("bad argument count %d to %s, expected between %d and %d arguments\n"),
			argc-1, cmd, ct->argmin, ct->argmax);
		return 0;
	}
	optind = 0;
	return ct->cfunc(argc, argv);
}

const cmdinfo_t *
find_command(
	const char	*cmd)
{
	cmdinfo_t	*ct;

	for (ct = cmdtab; ct < &cmdtab[ncmds]; ct++) {
		if (strcmp(ct->name, cmd) == 0 ||
		    (ct->altname && strcmp(ct->altname, cmd) == 0))
			return (const cmdinfo_t *)ct;
	}
	return NULL;
}

void
init_commands(void)
{
	bmap_init();
	help_init();
	open_init();
	pread_init();
	prealloc_init();
	pwrite_init();
	quit_init();
	resblks_init();
	fsync_init();
	truncate_init();
}
