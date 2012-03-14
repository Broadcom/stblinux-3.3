/*
 * Copyright (c) 2000-2002 Silicon Graphics, Inc.  All Rights Reserved.
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
#include <mntent.h>

extern char	*fname;		/* mount point name */
extern char	*datadev;	/* data device name */
extern char	*logdev;	/*  log device name */
extern char	*rtdev;		/*   RT device name */

void
explore_mtab(char *mtab, char *mntpoint)
{
	struct mntent	*mnt;
	struct stat64	statuser;
	struct stat64	statmtab;
	FILE		*mtp;
	char		*rtend;
	char		*logend;

	if (!mtab)
		mtab = MOUNTED;

	if ((mtp = setmntent(mtab, "r")) == NULL) {
		fprintf(stderr, _("%s: cannot access mount list %s: %s\n"),
			progname, MOUNTED, strerror(errno));
		exit(1);
	}
	if (stat64(mntpoint, &statuser) < 0) {
		fprintf(stderr, _("%s: cannot access mount point %s: %s\n"),
			progname, mntpoint, strerror(errno));
		exit(1);
	}

	while ((mnt = getmntent(mtp)) != NULL) {
		if (stat64(mnt->mnt_dir, &statmtab) < 0) {
			fprintf(stderr, _("%s: ignoring entry %s in %s: %s\n"),
				progname, mnt->mnt_dir, mtab, strerror(errno));
			continue;
		}
		if (statuser.st_ino != statmtab.st_ino ||
				statuser.st_dev != statmtab.st_dev)
			continue;
		else if (strcmp(mnt->mnt_type, "xfs") != 0) {
			fprintf(stderr, _("%s: %s is not an XFS filesystem\n"),
				progname, mntpoint);
			exit(1);
		}
		break;	/* we've found it */
	}

	if (mnt == NULL) {
		fprintf(stderr,
		_("%s: %s is not a filesystem mount point, according to %s\n"),
			progname, mntpoint, MOUNTED);
		exit(1);
	}

	/* find the data, log (logdev=), and realtime (rtdev=) devices */
	rtend = logend = NULL;
	fname = mnt->mnt_dir;
	datadev = mnt->mnt_fsname;
	if ((logdev = hasmntopt(mnt, "logdev="))) {
		logdev += 7;
		logend = strtok(logdev, " ,");
	}
	if ((rtdev = hasmntopt(mnt, "rtdev="))) {
		rtdev += 6;
		rtend = strtok(rtdev, " ,");
	}

	/* Do this only after we've finished processing mount options */
	if (logdev && logend != logdev)
		*logend = '\0';	/* terminate end of log device name */
	if (rtdev && rtend != rtdev)
		*rtend = '\0';	/* terminate end of rt device name */

	endmntent(mtp);
}
