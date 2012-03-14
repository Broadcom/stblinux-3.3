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

void
get_subvol_stripe_wrapper(char *dev, sv_type_t type, int *sunit, int *swidth)
{
	struct stat64 sb;

	if (dev == NULL)
		return;

	if (stat64(dev, &sb)) {
		fprintf(stderr, _("Cannot stat %s: %s\n"),
			dev, strerror(errno));
		exit(1);
	}

	if (  md_get_subvol_stripe(dev, type, sunit, swidth, &sb))
		return;
	if ( lvm_get_subvol_stripe(dev, type, sunit, swidth, &sb))
		return;
	if ( xvm_get_subvol_stripe(dev, type, sunit, swidth, &sb))
		return;
	if (evms_get_subvol_stripe(dev, type, sunit, swidth, &sb))
		return;

	/* ... add new device drivers here */
}

#define DEVICES	"/proc/devices"

/*
 * General purpose routine which dredges through procfs trying to
 * match up device driver names with the associated major numbers
 * being used in the running kernel.
 */
int
get_driver_block_major(const char *driver, int major)
{
	FILE	*f;
	char	buf[64], puf[64];
	int	dmajor, match = 0;

	if ((f = fopen(DEVICES, "r")) == NULL)
		return match;
	while (fgets(buf, sizeof(buf), f))	/* skip to block dev section */
		if (strncmp("Block devices:\n", buf, sizeof(buf)) == 0)
			break;
	while (fgets(buf, sizeof(buf), f))
		if ((sscanf(buf, "%u %s\n", &dmajor, puf) == 2) &&
		    (strncmp(puf, driver, sizeof(puf)) == 0) &&
		    (dmajor == major))
			match = 1;
	fclose(f);
	return match;
}
