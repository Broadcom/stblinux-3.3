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
#include "input.h"
#include "command.h"

char	*progname;
int	exitcode;

int	fdesc;
char	*fname;
xfs_fsop_geom_t	fgeom;

int	readonly;
int	directio;
int	realtime;
int	append;
int	osync;
int	trunc;

static int	ncmdline;
static char	**cmdline;

void
usage(void)
{
	fprintf(stderr,
		_("Usage: %s [-r] [-p prog] [-c cmd]... file\n"), progname);
	exit(1);
}

void
init(
	int		argc,
	char		**argv)
{
	int		fflag = 0;
	int		c;

	progname = basename(argv[0]);
	setlocale(LC_ALL, "");
	bindtextdomain(PACKAGE, LOCALEDIR);
	textdomain(PACKAGE);

	while ((c = getopt(argc, argv, "ac:dfp:rstVx")) != EOF) {
		switch (c) {
		case 'a':	/* append */
			append = 1;
			break;
		case 'c':	/* commands */
			ncmdline++;
			cmdline = realloc(cmdline, sizeof(char*) * (ncmdline));
			if (!cmdline) {
				perror("realloc");
				exit(1);
			}
			cmdline[ncmdline-1] = optarg;
			break;
		case 'd':	/* directIO */
			directio = 1;
			break;
		case 'f':	/* create */
			fflag = 1;
			break;
		case 'p':	/* progname */
			progname = optarg;
			break;
		case 'r':	/* readonly */
			readonly = 1;
			break;
		case 's':	/* sync */
			osync = 1;
			break;
		case 't':	/* truncate */
			trunc = 1;
			break;
		case 'x':	/* realtime */
			realtime = 1;
			break;
		case 'V':
			printf(_("%s version %s\n"), progname, VERSION);
			exit(0);
		default:
			usage();
		}
	}

	if (optind != argc - 1)
		usage();

	fname = strdup(argv[optind]);
	if ((fdesc = openfile(fname, &fgeom, append, fflag, directio,
				readonly, osync, trunc, realtime)) < 0)
		exit(1);

	init_commands();
}

int
main(
	int	argc,
	char	**argv)
{
	int	c, i, done = 0;
	char	*input;
	char	**v;

	init(argc, argv);

	for (i = 0; !done && i < ncmdline; i++) {
		v = breakline(cmdline[i], &c);
		if (c)
			done = command(c, v);
		free(v);
	}
	if (cmdline) {
		free(cmdline);
		return exitcode;
	}
	while (!done) {
		if ((input = fetchline()) == NULL)
			break;
		v = breakline(input, &c);
		if (c)
			done = command(c, v);
		doneline(input, v);
	}
	return exitcode;
}
