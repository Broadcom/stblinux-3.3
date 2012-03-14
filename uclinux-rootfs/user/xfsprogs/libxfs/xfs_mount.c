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

#include <xfs.h>

/*
 * Mount initialization code establishing various mount
 * fields from the superblock associated with the given
 * mount structure.
 */
void
xfs_mount_common(xfs_mount_t *mp, xfs_sb_t *sbp)
{
	int	i;

	mp->m_agfrotor = mp->m_agirotor = 0;
	spinlock_init(&mp->m_agirotor_lock, "m_agirotor_lock");
	mp->m_maxagi = mp->m_sb.sb_agcount;
	mp->m_blkbit_log = sbp->sb_blocklog + XFS_NBBYLOG;
	mp->m_blkbb_log = sbp->sb_blocklog - BBSHIFT;
	mp->m_sectbb_log = sbp->sb_sectlog - BBSHIFT;
	mp->m_agno_log = xfs_highbit32(sbp->sb_agcount - 1) + 1;
	mp->m_agino_log = sbp->sb_inopblog + sbp->sb_agblklog;
	mp->m_litino = sbp->sb_inodesize -
		((uint)sizeof(xfs_dinode_core_t) + (uint)sizeof(xfs_agino_t));
	mp->m_blockmask = sbp->sb_blocksize - 1;
	mp->m_blockwsize = sbp->sb_blocksize >> XFS_WORDLOG;
	mp->m_blockwmask = mp->m_blockwsize - 1;
	INIT_LIST_HEAD(&mp->m_del_inodes);

	/*
	 * Setup for attributes, in case they get created.
	 * This value is for inodes getting attributes for the first time,
	 * the per-inode value is for old attribute values.
	 */
	ASSERT(sbp->sb_inodesize >= 256 && sbp->sb_inodesize <= 2048);
	switch (sbp->sb_inodesize) {
	case 256:
		mp->m_attroffset = XFS_LITINO(mp) - XFS_BMDR_SPACE_CALC(2);
		break;
	case 512:
	case 1024:
	case 2048:
		mp->m_attroffset = XFS_BMDR_SPACE_CALC(12);
		break;
	default:
		ASSERT(0);
	}
	ASSERT(mp->m_attroffset < XFS_LITINO(mp));

	for (i = 0; i < 2; i++) {
		mp->m_alloc_mxr[i] = XFS_BTREE_BLOCK_MAXRECS(sbp->sb_blocksize,
			xfs_alloc, i == 0);
		mp->m_alloc_mnr[i] = XFS_BTREE_BLOCK_MINRECS(sbp->sb_blocksize,
			xfs_alloc, i == 0);
	}
	for (i = 0; i < 2; i++) {
		mp->m_bmap_dmxr[i] = XFS_BTREE_BLOCK_MAXRECS(sbp->sb_blocksize,
			xfs_bmbt, i == 0);
		mp->m_bmap_dmnr[i] = XFS_BTREE_BLOCK_MINRECS(sbp->sb_blocksize,
			xfs_bmbt, i == 0);
	}
	for (i = 0; i < 2; i++) {
		mp->m_inobt_mxr[i] = XFS_BTREE_BLOCK_MAXRECS(sbp->sb_blocksize,
			xfs_inobt, i == 0);
		mp->m_inobt_mnr[i] = XFS_BTREE_BLOCK_MINRECS(sbp->sb_blocksize,
			xfs_inobt, i == 0);
	}

	mp->m_bsize = XFS_FSB_TO_BB(mp, 1);
	mp->m_ialloc_inos = (int)MAX((__uint16_t)XFS_INODES_PER_CHUNK,
					sbp->sb_inopblock);
	mp->m_ialloc_blks = mp->m_ialloc_inos >> sbp->sb_inopblog;
}

static struct {
    short offset;
    short type;	    /* 0 = integer
		     * 1 = binary / string (no translation)
		     */
} xfs_sb_info[] = {
    { offsetof(xfs_sb_t, sb_magicnum),	 0 },
    { offsetof(xfs_sb_t, sb_blocksize),	 0 },
    { offsetof(xfs_sb_t, sb_dblocks),	 0 },
    { offsetof(xfs_sb_t, sb_rblocks),	 0 },
    { offsetof(xfs_sb_t, sb_rextents),	 0 },
    { offsetof(xfs_sb_t, sb_uuid),	 1 },
    { offsetof(xfs_sb_t, sb_logstart),	 0 },
    { offsetof(xfs_sb_t, sb_rootino),	 0 },
    { offsetof(xfs_sb_t, sb_rbmino),	 0 },
    { offsetof(xfs_sb_t, sb_rsumino),	 0 },
    { offsetof(xfs_sb_t, sb_rextsize),	 0 },
    { offsetof(xfs_sb_t, sb_agblocks),	 0 },
    { offsetof(xfs_sb_t, sb_agcount),	 0 },
    { offsetof(xfs_sb_t, sb_rbmblocks),	 0 },
    { offsetof(xfs_sb_t, sb_logblocks),	 0 },
    { offsetof(xfs_sb_t, sb_versionnum), 0 },
    { offsetof(xfs_sb_t, sb_sectsize),	 0 },
    { offsetof(xfs_sb_t, sb_inodesize),	 0 },
    { offsetof(xfs_sb_t, sb_inopblock),	 0 },
    { offsetof(xfs_sb_t, sb_fname[0]),	 1 },
    { offsetof(xfs_sb_t, sb_blocklog),	 0 },
    { offsetof(xfs_sb_t, sb_sectlog),	 0 },
    { offsetof(xfs_sb_t, sb_inodelog),	 0 },
    { offsetof(xfs_sb_t, sb_inopblog),	 0 },
    { offsetof(xfs_sb_t, sb_agblklog),	 0 },
    { offsetof(xfs_sb_t, sb_rextslog),	 0 },
    { offsetof(xfs_sb_t, sb_inprogress), 0 },
    { offsetof(xfs_sb_t, sb_imax_pct),	 0 },
    { offsetof(xfs_sb_t, sb_icount),	 0 },
    { offsetof(xfs_sb_t, sb_ifree),	 0 },
    { offsetof(xfs_sb_t, sb_fdblocks),	 0 },
    { offsetof(xfs_sb_t, sb_frextents),	 0 },
    { offsetof(xfs_sb_t, sb_uquotino),	 0 },
    { offsetof(xfs_sb_t, sb_gquotino),	 0 },
    { offsetof(xfs_sb_t, sb_qflags),	 0 },
    { offsetof(xfs_sb_t, sb_flags),	 0 },
    { offsetof(xfs_sb_t, sb_shared_vn),	 0 },
    { offsetof(xfs_sb_t, sb_inoalignmt), 0 },
    { offsetof(xfs_sb_t, sb_unit),	 0 },
    { offsetof(xfs_sb_t, sb_width),	 0 },
    { offsetof(xfs_sb_t, sb_dirblklog),	 0 },
    { offsetof(xfs_sb_t, sb_logsectlog), 0 },
    { offsetof(xfs_sb_t, sb_logsectsize),0 },
    { offsetof(xfs_sb_t, sb_logsunit),	 0 },
    { sizeof(xfs_sb_t),			 0 }
};

/*
 * xfs_xlatesb
 *     data	  - on disk version of sb
 *     sb	  - a superblock
 *     dir	  - conversion direction: <0 - convert sb to buf
 *					  >0 - convert buf to sb
 *     arch	  - architecture to read/write from/to buf
 *     fields	  - which fields to copy (bitmask)
 */
void
xfs_xlatesb(
	void		*data,
	xfs_sb_t	*sb,
	int		dir,
	xfs_arch_t	arch,
	__int64_t	fields)
{
	xfs_caddr_t	buf_ptr;
	xfs_caddr_t	mem_ptr;
	xfs_sb_field_t	f;
	int		first;
	int		size;

	ASSERT(dir);
	ASSERT(fields);

	if (!fields)
		return;

	buf_ptr = (xfs_caddr_t)data;
	mem_ptr = (xfs_caddr_t)sb;

	while (fields) {
		f = (xfs_sb_field_t)xfs_lowbit64((__uint64_t)fields);
		first = xfs_sb_info[f].offset;
		size = xfs_sb_info[f + 1].offset - first;

		ASSERT(xfs_sb_info[f].type == 0 || xfs_sb_info[f].type == 1);

		if (arch == ARCH_NOCONVERT ||
		    size == 1 ||
		    xfs_sb_info[f].type == 1) {
			if (dir > 0) {
				memcpy(mem_ptr + first, buf_ptr + first, size);
			} else {
				memcpy(buf_ptr + first, mem_ptr + first, size);
			}
		} else {
			switch (size) {
			case 2:
				INT_XLATE(*(__uint16_t*)(buf_ptr+first),
					  *(__uint16_t*)(mem_ptr+first),
					  dir, arch);
				break;
			case 4:
				INT_XLATE(*(__uint32_t*)(buf_ptr+first),
					  *(__uint32_t*)(mem_ptr+first),
					  dir, arch);
				break;
			case 8:
				INT_XLATE(*(__uint64_t*)(buf_ptr+first),
					  *(__uint64_t*)(mem_ptr+first), dir, arch);
				break;
			default:
				ASSERT(0);
			}
		}

		fields &= ~(1LL << f);
	}
}

void
xfs_initialize_perag(xfs_mount_t *mp, int agcount)
{
	int		index, max_metadata;
	xfs_perag_t	*pag;
	xfs_agino_t	agino;
	xfs_ino_t	ino;
	xfs_sb_t	*sbp = &mp->m_sb;
	xfs_ino_t	max_inum = XFS_MAXINUMBER_32;

	/* Check to see if the filesystem can overflow 32 bit inodes */
	agino = XFS_OFFBNO_TO_AGINO(mp, sbp->sb_agblocks - 1, 0);
	ino = XFS_AGINO_TO_INO(mp, agcount - 1, agino);

	/* Clear the mount flag if no inode can overflow 32 bits
	 * on this filesystem.
	 */
	if (ino <= max_inum) {
		mp->m_flags &= ~XFS_MOUNT_32BITINODES;
	}

	/* If we can overflow then setup the ag headers accordingly */
	if (mp->m_flags & XFS_MOUNT_32BITINODES) {
		/* Calculate how much should be reserved for inodes to
		 * meet the max inode percentage.
		 */
		if (mp->m_maxicount) {
			__uint64_t	icount;

			icount = sbp->sb_dblocks * sbp->sb_imax_pct;
			do_div(icount, 100);
			icount += sbp->sb_agblocks - 1;
			do_div(icount, mp->m_ialloc_blks);
			max_metadata = icount;
		} else {
			max_metadata = agcount;
		}
		for (index = 0; index < agcount; index++) {
			ino = XFS_AGINO_TO_INO(mp, index, agino);
			if (ino > max_inum) {
				index++;
				break;
			}

			/* This ag is prefered for inodes */
			pag = &mp->m_perag[index];
			pag->pagi_inodeok = 1;
			if (index < max_metadata)
				pag->pagf_metadata = 1;
		}
	} else {
		/* Setup default behavior for smaller filesystems */
		for (index = 0; index < agcount; index++) {
			pag = &mp->m_perag[index];
			pag->pagi_inodeok = 1;
		}
	}
	mp->m_maxagi = index;
}
