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
#ifndef XFS_GROWFS_EXPLORE_H
#define XFS_GROWFS_EXPLORE_H

/*
 * This was written as part of the Linux port.  On IRIX,
 * the volume managers have knowledge of log and realtime
 * subvolumes, and the equivalent functionality is built
 * into the kernel - XFS/XLV/XVM talk amongst themselves
 * and there are no rtdev/logdev mount parameters at all.
 */
#ifdef __linux__
extern void explore_mtab(char *mtab, char *mntpoint);
#else
# define explore_mtab(mtab, mpoint)	do { } while (0)
#endif

#endif	/* XFS_GROWFS_EXPLORE_H */
