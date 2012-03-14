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
#ifndef _XR_ATTRREPAIR_H
#define _XR_ATTRREPAIR_H

#define ACL_USER_OBJ	0x01	/* owner */
#define ACL_USER	0x02	/* additional users */
#define ACL_GROUP_OBJ	0x04	/* group */
#define ACL_GROUP	0x08	/* additional groups */
#define ACL_MASK	0x10	/* mask entry */
#define ACL_OTHER	0x20	/* other entry */

#define ACL_READ	04
#define ACL_WRITE	02
#define ACL_EXECUTE	01

struct blkmap;
extern int process_attributes (xfs_mount_t *, xfs_ino_t, xfs_dinode_t *,
				struct blkmap *, int *);

#endif /* _XR_ATTRREPAIR_H */
