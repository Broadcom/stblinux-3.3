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

/*
 * Inode map display utility for XFS.
 */

#include <xfs/libxfs.h>

int main(int argc, char **argv)
{
	int		count;
	int		nent;
	int		fd;
	int		i;
	char		*name;
	char		*progname;
	__s64		last = 0;
	xfs_inogrp_t	*t;
	xfs_fsop_bulkreq_t bulkreq;

	progname = basename(argv[0]);
	setlocale(LC_ALL, "");
	bindtextdomain(PACKAGE, LOCALEDIR);
	textdomain(PACKAGE);

	if (argc < 2)
		name = ".";
	else
		name = argv[1];
	fd = open(name, O_RDONLY);
	if (fd < 0) {
		perror(name);
		return 1;
	}
	if (!platform_test_xfs_fd(fd)) {
		fprintf(stderr, _("%s: specified file "
			"[\"%s\"] is not on an XFS filesystem\n"),
			progname, name);
		exit(1);
	}

	if (argc < 3)
		nent = 1;
	else
		nent = atoi(argv[2]);

	t = malloc(nent * sizeof(*t));

	bulkreq.lastip  = &last;
	bulkreq.icount  = nent;
	bulkreq.ubuffer = t;
	bulkreq.ocount  = &count;

	while (xfsctl(name, fd, XFS_IOC_FSINUMBERS, &bulkreq) == 0) {
		if (count == 0)
			return 0;
		for (i = 0; i < count; i++) {
			printf(_("ino %10llu count %2d mask %016llx\n"),
				(unsigned long long)t[i].xi_startino,
				t[i].xi_alloccount,
				(unsigned long long)t[i].xi_allocmask);
		}
	}
	perror("xfsctl(XFS_IOC_FSINUMBERS)");
	return 1;
}
