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

/*
 * An on-disk allocation group header is composed of 4 structures,
 * each of which is 1 disk sector long where the sector size is at
 * least 512 bytes long (BBSIZE).
 *
 * There's one ag_header per ag and the superblock in the first ag
 * is the contains the real data for the entire filesystem (although
 * most of the relevant data won't change anyway even on a growfs).
 *
 * The filesystem superblock specifies the number of AG's and
 * the AG size.  That splits the filesystem up into N pieces,
 * each of which is an AG and has an ag_header at the beginning.
 */
typedef struct ag_header  {
	xfs_sb_t	*xfs_sb;	/* superblock for filesystem or AG */
	xfs_agf_t	*xfs_agf;	/* free space info */
	xfs_agi_t	*xfs_agi;	/* free inode info */
	xfs_agfl_t	*xfs_agfl;	/* AG freelist */
	char		*residue;
	int		residue_length;
} ag_header_t;

/*
 * The position/buf_position, length/buf_length, data/buffer pairs
 * exist because of alignment constraints for direct I/O and dealing
 * with scenarios where either the source or target or both is a file
 * and the blocksize of the filesystem where file resides is different
 * from that of the filesystem image being duplicated.  You can get
 * alignment problems resulting from things like AG's starting on
 * non-aligned points in the filesystem.  So you have to be able
 * to read from points "before" the requested starting point and
 * read in more data than requested.
 */
struct t_args;
typedef struct {
	int		id;		/* buffer ID */
	size_t		size;		/* size of buffer -- fixed */
	size_t		min_io_size;	/* for direct I/O */
	xfs_off_t	position;	/* requested position (bytes) */
	size_t		length;		/* requested length (bytes) */
	char		*data;		/* pointer to data buffer */
	struct t_args	*owner;		/* for non-parallel writes */
} wbuf;

typedef struct t_args {
	int		id;
	uuid_t		uuid;
	pthread_mutex_t	wait;
	int		fd;
} thread_args;

typedef struct {
	pthread_mutex_t mutex;
	int		num_working;
	wbuf		*buffer;
} thread_control;

typedef int thread_id;
typedef int tm_index;			/* index into thread mask array */
typedef __uint32_t thread_mask;		/* a thread mask */

typedef struct {
	char		*name;
	int		fd;
	xfs_off_t	position;
	pthread_t	pid;
	int		state;
	int		error;
	int		err_type;
} target_control;

