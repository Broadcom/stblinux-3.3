/*
 * Copyright (c) 2000-2003 Silicon Graphics, Inc.  All Rights Reserved.
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

#include "drivers.h"

#ifndef LVM_BLK_MAJOR
#define LVM_BLK_MAJOR	58
#endif

int
mnt_is_lvm_subvol(
	dev_t		dev)
{
	if (major(dev) == LVM_BLK_MAJOR)
		return 1;
	return get_driver_block_major("lvm", major(dev));
}

int
lvm_get_subvol_stripe(
	char		*dfile,
	sv_type_t	type,
	int		*sunit,
	int		*swidth,
	struct stat64	*sb)
{
	int		lvpipe[2], stripes = 0, stripesize = 0;
	char		*largv[3], buf[1024];
	FILE		*stream;
	char		tmppath[MAXPATHLEN];

	if (!mnt_is_lvm_subvol(sb->st_rdev))
		return 0;

	/* Quest for lvdisplay */
	if (!access("/usr/local/sbin/lvdisplay", R_OK|X_OK))
		largv[0] = "/usr/local/sbin/lvdisplay";
	else if (!access("/usr/sbin/lvdisplay", R_OK|X_OK))
		largv[0] = "/usr/sbin/lvdisplay";
	else if (!access("/sbin/lvdisplay", R_OK|X_OK))
		largv[0] = "/sbin/lvdisplay";
	else {
		fprintf(stderr,
			"Warning - LVM device, but no lvdisplay(8) found\n");
		return 0;
	}

	/* lvm tools want the full, real path to a logical volume.
	 * (lvm doesn't really open the file, it just wants a
	 * string that happens to look like a path :/ )
	 */
	if (dfile[0] != '/') {
		getcwd(tmppath, MAXPATHLEN);
		strcat(tmppath, "/");
		dfile = strcat(tmppath, dfile);
	}
	largv[1] = realpath(dfile, tmppath);
	largv[2] = NULL;

	/* Open pipe */
	if (pipe(lvpipe) < 0) {
		fprintf(stderr, _("Could not open pipe\n"));
		exit(1);
	}

	/* Spawn lvdisplay */
	switch (fork()) {
	case 0:
		/* Plumbing */
		close(lvpipe[0]);

		if (lvpipe[1] != STDOUT_FILENO)
			dup2(lvpipe[1], STDOUT_FILENO);

		execv(largv[0], largv);

		fprintf(stderr, _("Failed to execute %s\n"), largv[0]);
		exit(1);

	case -1:
		fprintf(stderr, _("Failed forking lvdisplay process\n"));
		exit(1);

	default:
		break;
	}

	close(lvpipe[1]);
	stream = fdopen(lvpipe[0], "r");

	/* Scan stream for keywords */
	while (fgets(buf, 1023, stream) != NULL) {

		if (!strncmp(buf, "Stripes", 7))
			sscanf(buf, "Stripes %d", &stripes);

		if (!strncmp(buf, "Stripe size", 11))
			sscanf(buf, "Stripe size (KByte) %d", &stripesize);
	}

	/* Update sizes */
	*sunit = stripesize << 1;
	*swidth = (stripes * stripesize) << 1;

	fclose(stream);

	return 1;
}
