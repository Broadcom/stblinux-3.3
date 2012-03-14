/*
 * Copyright (c) 2002-2003 Silicon Graphics, Inc.  All Rights Reserved.
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

#ifndef MD_MAJOR
#define MD_MAJOR		9 /* we also check at runtime */
#endif

#define GET_ARRAY_INFO          _IOR (MD_MAJOR, 0x11, struct md_array_info)

#define MD_SB_CLEAN		0
#define MD_SB_ERRORS		1

struct md_array_info {
	/*
	 * Generic constant information
	 */
	__uint32_t major_version;
	__uint32_t minor_version;
	__uint32_t patch_version;
	__uint32_t ctime;
	__uint32_t level;
	__uint32_t size;
	__uint32_t nr_disks;
	__uint32_t raid_disks;
	__uint32_t md_minor;
	__uint32_t not_persistent;

	/*
	 * Generic state information
	 */
	__uint32_t utime;	  /*  0 Superblock update time		  */
	__uint32_t state;	  /*  1 State bits (clean, ...)		  */
	__uint32_t active_disks;  /*  2 Number of currently active disks  */
	__uint32_t working_disks; /*  3 Number of working disks		  */
	__uint32_t failed_disks;  /*  4 Number of failed disks		  */
	__uint32_t spare_disks;	  /*  5 Number of spare disks		  */

	/*
	 * Personality information
	 */
	__uint32_t layout;	  /*  0 the array's physical layout	  */
	__uint32_t chunk_size;	  /*  1 chunk size in bytes		  */

};
