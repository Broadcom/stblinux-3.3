/*
 * Copyright (c) 2001-2003 Silicon Graphics, Inc.  All Rights Reserved.
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

char	*progname;

static void
usage(void)
{
	fprintf(stderr, _(
"Usage: %s [options] mountpoint\n\n\
Options:\n\
	-f          freeze filesystem access\n\
	-u          unfreeze filesystem access\n"),
		progname);
	exit(2);
}

int
main(int argc, char **argv)
{
	int			c;	/* current option character */
	int			ffd;	/* mount point file descriptor */
	int			fflag, uflag;
	int			level;
	char			*fname;

	fflag = uflag = 0;
	progname = basename(argv[0]);
	setlocale(LC_ALL, "");
	bindtextdomain(PACKAGE, LOCALEDIR);
	textdomain(PACKAGE);

	while ((c = getopt(argc, argv, "fu")) != EOF) {
		switch (c) {
		case 'f':
			fflag = 1;
			break;
		case 'u':
			uflag = 1;
			break;
		case '?':
		default:
			usage();
		}
	}
	if (argc - optind != 1)
		usage();
	if ((fflag + uflag) != 1)
		usage();

	fname = argv[optind];
	ffd = open(fname, O_RDONLY);
	if (ffd < 0) {
		perror(fname);
		return 1;
	}

	if (!platform_test_xfs_fd(ffd)) {
		fprintf(stderr, _("%s: specified file "
			"[\"%s\"] is not on an XFS filesystem\n"),
			progname, argv[optind]);
		exit(1);
	}

	if (fflag) {
		level = 1;
		if (xfsctl(fname, ffd, XFS_IOC_FREEZE, &level) < 0) {
			fprintf(stderr, _("%s: cannot freeze filesystem"
				" mounted at %s: %s\n"),
				progname, argv[optind], strerror(errno));
			exit(1);
		}
	}

	if (uflag) {
		if (xfsctl(fname, ffd, XFS_IOC_THAW, &level) < 0) {
			fprintf(stderr, _("%s: cannot unfreeze filesystem"
				" mounted at %s: %s\n"),
				progname, argv[optind], strerror(errno));
			exit(1);
		}
	}

	close(ffd);
	return 0;
}
