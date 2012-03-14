/*
 * Copyright (c) 2000-2001 Silicon Graphics, Inc.  All Rights Reserved.
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

#define _DIOC_(x)        (('d'<<8) | x)
#define DIOCGETVOLDEV    _DIOC_(36) /* subvolume devices */
#define DIOCGETVOLSTRIPE _DIOC_(47) /* subvolume stripe info */

/*
 * Structure returned by the DIOCGETVOLDEV ioctl to list the
 * subvolume device nodes in a volume.  These are external device
 * numbers.
 */
#define XVM_GETDEV_VERS 1

typedef __uint32_t xvm_dev_t;

typedef struct {
	__uint32_t              version;
	xvm_dev_t		data_subvol_dev;

	xvm_dev_t		log_subvol_dev;
	xvm_dev_t		rt_subvol_dev;

	xvm_dev_t		sp_subvol_dev;
} xvm_getdev_t;

/*
 * Structure returned by the DIOCGETVOLSTRIPE ioctl to describe
 * the subvolume stripe units and width.
 */
#define XVM_SUBVOL_GEOMETRY_VERS  1
typedef struct xvm_subvol_stripe_s {
	__uint32_t              version;
	__uint32_t              unit_size;      /* in blocks */
	__uint32_t              width_size;     /* in blocks */
	__uint32_t		pad1;		/* padding */
	xvm_dev_t		dev;
} xvm_subvol_stripe_t;
