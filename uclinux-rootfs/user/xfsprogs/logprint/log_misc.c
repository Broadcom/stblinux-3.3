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

#include "logprint.h"

#define ZEROED_LOG	(-4)
#define FULL_READ	(-3)
#define PARTIAL_READ	(-2)
#define BAD_HEADER	(-1)
#define NO_ERROR	(0)

static int logBBsize;
char *trans_type[] = {
	"",
	"SETATTR",
	"SETATTR_SIZE",
	"INACTIVE",
	"CREATE",
	"CREATE_TRUNC",
	"TRUNCATE_FILE",
	"REMOVE",
	"LINK",
	"RENAME",
	"MKDIR",
	"RMDIR",
	"SYMLINK",
	"SET_DMATTRS",
	"GROWFS",
	"STRAT_WRITE",
	"DIOSTRAT",
	"WRITE_SYNC",
	"WRITEID",
	"ADDAFORK",
	"ATTRINVAL",
	"ATRUNCATE",
	"ATTR_SET",
	"ATTR_RM",
	"ATTR_FLAG",
	"CLEAR_AGI_BUCKET",
	"QM_SBCHANGE",
	"DUMMY1",
	"DUMMY2",
	"QM_QUOTAOFF",
	"QM_DQALLOC",
	"QM_SETQLIM",
	"QM_DQCLUSTER",
	"QM_QINOCREATE",
	"QM_QUOTAOFF_END",
	"SB_UNIT",
	"FSYNC_TS",
	"GROWFSRT_ALLOC",
	"GROWFSRT_ZERO",
	"GROWFSRT_FREE",
	"SWAPEXT",
};

typedef struct xlog_split_item {
	struct xlog_split_item	*si_next;
	struct xlog_split_item	*si_prev;
	xlog_tid_t		si_tid;
	int			si_skip;
} xlog_split_item_t;

xlog_split_item_t *split_list = 0;

void
print_xlog_op_line(void)
{
    printf("--------------------------------------"
	   "--------------------------------------\n");
}	/* print_xlog_op_line */

void
print_xlog_record_line(void)
{
    printf("======================================"
	   "======================================\n");
}	/* print_xlog_record_line */

void
print_stars(void)
{
    printf("***********************************"
	   "***********************************\n");
}	/* print_xlog_record_line */

/*
 * Given a pointer to a data segment, print out the data as if it were
 * a log operation header.
 */
void
xlog_print_op_header(xlog_op_header_t	*op_head,
		     int		i,
		     xfs_caddr_t	*ptr)
{
    xlog_op_header_t hbuf;

    /*
     * bcopy because on 64/n32, partial reads can cause the op_head
     * pointer to come in pointing to an odd-numbered byte
     */
    bcopy(op_head, &hbuf, sizeof(xlog_op_header_t));
    op_head = &hbuf;
    *ptr += sizeof(xlog_op_header_t);
    printf("Oper (%d): tid: %x  len: %d  clientid: %s  ", i,
	    INT_GET(op_head->oh_tid, ARCH_CONVERT),
	    INT_GET(op_head->oh_len, ARCH_CONVERT),
	    (op_head->oh_clientid == XFS_TRANSACTION ? "TRANS" :
	    (op_head->oh_clientid == XFS_LOG ? "LOG" : "ERROR")));
    printf("flags: ");
    if (op_head->oh_flags) {
	if (op_head->oh_flags & XLOG_START_TRANS)
	    printf("START ");
	if (op_head->oh_flags & XLOG_COMMIT_TRANS)
	    printf("COMMIT ");
	if (op_head->oh_flags & XLOG_WAS_CONT_TRANS)
	    printf("WAS_CONT ");
	if (op_head->oh_flags & XLOG_UNMOUNT_TRANS)
	    printf("UNMOUNT ");
	if (op_head->oh_flags & XLOG_CONTINUE_TRANS)
	    printf("CONTINUE ");
	if (op_head->oh_flags & XLOG_END_TRANS)
	    printf("END ");
    } else {
	printf("none");
    }
    printf("\n");
}	/* xlog_print_op_header */


void
xlog_print_add_to_trans(xlog_tid_t	tid,
			int		skip)
{
    xlog_split_item_t *item;

    item	  = (xlog_split_item_t *)calloc(sizeof(xlog_split_item_t), 1);
    item->si_tid  = tid;
    item->si_skip = skip;
    item->si_next = split_list;
    item->si_prev = 0;
    if (split_list)
	split_list->si_prev = item;
    split_list	  = item;
}	/* xlog_print_add_to_trans */


int
xlog_print_find_tid(xlog_tid_t tid, uint was_cont)
{
    xlog_split_item_t *listp = split_list;

    if (!split_list) {
	if (was_cont != 0)	/* Not first time we have used this tid */
	    return 1;
	else
	    return 0;
    }
    while (listp) {
	if (listp->si_tid == tid)
	    break;
	listp = listp->si_next;
    }
    if (!listp)  {
	return 0;
    }
    if (--listp->si_skip == 0) {
	if (listp == split_list) {		/* delete at head */
	    split_list = listp->si_next;
	    if (split_list)
		split_list->si_prev = NULL;
	} else {
	    if (listp->si_next)
		listp->si_next->si_prev = listp->si_prev;
	    listp->si_prev->si_next = listp->si_next;
	}
	free(listp);
    }
    return 1;
}	/* xlog_print_find_tid */

int
xlog_print_trans_header(xfs_caddr_t *ptr, int len)
{
    xfs_trans_header_t  *h;
    xfs_caddr_t		cptr = *ptr;
    __uint32_t          magic;
    char                *magic_c = (char *)&magic;

    *ptr += len;

    magic=*(__uint32_t*)cptr; /* XXX INT_GET soon */

    if (len >= 4) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
	printf("%c%c%c%c:",
		magic_c[3], magic_c[2], magic_c[1], magic_c[0]);
#else
	printf("%c%c%c%c:",
		magic_c[0], magic_c[1], magic_c[2], magic_c[3]);
#endif
    }
    if (len != sizeof(xfs_trans_header_t)) {
	printf("   Not enough data to decode further\n");
	return 1;
    }
    h = (xfs_trans_header_t *)cptr;
    printf("    type: %s       tid: %x       num_items: %d\n",
	   trans_type[h->th_type], h->th_tid, h->th_num_items);
    return 0;
}	/* xlog_print_trans_header */


int
xlog_print_trans_buffer(xfs_caddr_t *ptr, int len, int *i, int num_ops)
{
    xfs_buf_log_format_t *f;
    xfs_buf_log_format_v1_t *old_f;
    xfs_agi_t		 *agi;
    xfs_agf_t		 *agf;
    xfs_disk_dquot_t	 *dq;
    xlog_op_header_t	 *head = 0;
    int			 num, skip;
    int			 super_block = 0;
    int			 bucket, col, buckets;
    __int64_t		 blkno;
    xfs_buf_log_format_t lbuf;
    int			 size, blen, map_size, struct_size;
    long long		 x, y;

    /*
     * bcopy to ensure 8-byte alignment for the long longs in
     * buf_log_format_t structure
     */
    bcopy(*ptr, &lbuf, MIN(sizeof(xfs_buf_log_format_t), len));
    f = &lbuf;
    *ptr += len;

    if (f->blf_type == XFS_LI_BUF) {
	blkno = f->blf_blkno;
	size = f->blf_size;
	blen = f->blf_len;
	map_size = f->blf_map_size;
	struct_size = sizeof(xfs_buf_log_format_t);
    } else {
	old_f = (xfs_buf_log_format_v1_t*)f;
	blkno = old_f->blf_blkno;
	size = old_f->blf_size;
	blen = old_f->blf_len;
	map_size = old_f->blf_map_size;
	struct_size = sizeof(xfs_buf_log_format_v1_t);
    }
    switch (f->blf_type)  {
    case XFS_LI_BUF:
	printf("BUF:  ");
	break;
    case XFS_LI_6_1_BUF:
	printf("6.1 BUF:  ");
	break;
    case XFS_LI_5_3_BUF:
	printf("5.3 BUF:  ");
	break;
    default:
	printf("UNKNOWN BUF:  ");
	break;
    }
    if (len >= struct_size) {
	ASSERT((len - sizeof(struct_size)) % sizeof(int) == 0);
	printf("#regs: %d   start blkno: %lld (0x%llx)  len: %d  bmap size: %d\n",
	       size, (long long)blkno, (unsigned long long)blkno, blen, map_size);
	if (blkno == 0)
	    super_block = 1;
    } else {
	ASSERT(len >= 4);	/* must have at least 4 bytes if != 0 */
	printf("#regs: %d   Not printing rest of data\n", f->blf_size);
	return size;
    }
    num = size-1;

    /* Check if all regions in this log item were in the given LR ptr */
    if (*i+num > num_ops-1) {
	skip = num - (num_ops-1-*i);
	num = num_ops-1-*i;
    } else {
	skip = 0;
    }
    while (num-- > 0) {
	(*i)++;
	head = (xlog_op_header_t *)*ptr;
	xlog_print_op_header(head, *i, ptr);
	if (super_block) {
		printf("SUPER BLOCK Buffer: ");
		if (INT_GET(head->oh_len, ARCH_CONVERT) < 4*8) {
			printf("Out of space\n");
		} else {
			printf("\n");
			/*
			 * bcopy because *ptr may not be 8-byte aligned
			 */
			bcopy(*ptr, &x, sizeof(long long));
			bcopy(*ptr+8, &y, sizeof(long long));
			printf("icount: %lld  ifree: %lld  ",
				INT_GET(x, ARCH_CONVERT),
				INT_GET(y, ARCH_CONVERT));
			bcopy(*ptr+16, &x, sizeof(long long));
			bcopy(*ptr+24, &y, sizeof(long long));
			printf("fdblks: %lld  frext: %lld\n",
				INT_GET(x, ARCH_CONVERT),
				INT_GET(y, ARCH_CONVERT));
		}
		super_block = 0;
	} else if (INT_GET(*(uint *)(*ptr), ARCH_CONVERT) == XFS_AGI_MAGIC) {
		agi = (xfs_agi_t *)(*ptr);
		printf("AGI Buffer: XAGI  ");
		if (INT_GET(head->oh_len, ARCH_CONVERT) <
		    sizeof(xfs_agi_t) -
		    XFS_AGI_UNLINKED_BUCKETS*sizeof(xfs_agino_t)) {
			printf("out of space\n");
		} else {
			printf("\n");
			printf("ver: %d  ",
				INT_GET(agi->agi_versionnum, ARCH_CONVERT));
			printf("seq#: %d  len: %d  cnt: %d  root: %d\n",
				INT_GET(agi->agi_seqno, ARCH_CONVERT),
				INT_GET(agi->agi_length, ARCH_CONVERT),
				INT_GET(agi->agi_count, ARCH_CONVERT),
				INT_GET(agi->agi_root, ARCH_CONVERT));
			printf("level: %d  free#: 0x%x  newino: 0x%x\n",
				INT_GET(agi->agi_level, ARCH_CONVERT),
				INT_GET(agi->agi_freecount, ARCH_CONVERT),
				INT_GET(agi->agi_newino, ARCH_CONVERT));
			if (INT_GET(head->oh_len, ARCH_CONVERT) == 128) {
				buckets = 17;
			} else if (INT_GET(head->oh_len, ARCH_CONVERT) == 256) {
				buckets = 32 + 17;
			} else {
				if (head->oh_flags & XLOG_CONTINUE_TRANS) {
					printf("AGI unlinked data skipped ");
					printf("(CONTINUE set, no space)\n");
					continue;
				}
				buckets = XFS_AGI_UNLINKED_BUCKETS;
			}
			for (bucket = 0; bucket < buckets;) {
				printf("bucket[%d - %d]: ", bucket, bucket+3);
				for (col = 0; col < 4; col++, bucket++) {
					if (bucket < buckets) {
						printf("0x%x ",
			INT_GET(agi->agi_unlinked[bucket], ARCH_CONVERT));
					}
				}
				printf("\n");
			}
		}
	} else if (INT_GET(*(uint *)(*ptr), ARCH_CONVERT) == XFS_AGF_MAGIC) {
		agf = (xfs_agf_t *)(*ptr);
		printf("AGF Buffer: XAGF  ");
		if (INT_GET(head->oh_len, ARCH_CONVERT) < sizeof(xfs_agf_t)) {
			printf("Out of space\n");
		} else {
			printf("\n");
			printf("ver: %d  seq#: %d  len: %d  \n",
				INT_GET(agf->agf_versionnum, ARCH_CONVERT),
				INT_GET(agf->agf_seqno, ARCH_CONVERT),
				INT_GET(agf->agf_length, ARCH_CONVERT));
			printf("root BNO: %d  CNT: %d\n",
				INT_GET(agf->agf_roots[XFS_BTNUM_BNOi],
					ARCH_CONVERT),
				INT_GET(agf->agf_roots[XFS_BTNUM_CNTi],
					ARCH_CONVERT));
			printf("level BNO: %d  CNT: %d\n",
				INT_GET(agf->agf_levels[XFS_BTNUM_BNOi],
					ARCH_CONVERT),
				INT_GET(agf->agf_levels[XFS_BTNUM_CNTi],
					ARCH_CONVERT));
			printf("1st: %d  last: %d  cnt: %d  "
			       "freeblks: %d  longest: %d\n",
				INT_GET(agf->agf_flfirst, ARCH_CONVERT),
				INT_GET(agf->agf_fllast, ARCH_CONVERT),
				INT_GET(agf->agf_flcount, ARCH_CONVERT),
				INT_GET(agf->agf_freeblks, ARCH_CONVERT),
				INT_GET(agf->agf_longest, ARCH_CONVERT));
		}
	} else if (INT_GET(*(uint *)(*ptr), ARCH_CONVERT) == XFS_DQUOT_MAGIC) {
		dq = (xfs_disk_dquot_t *)(*ptr);
		printf("DQUOT Buffer: DQ  ");
		if (INT_GET(head->oh_len, ARCH_CONVERT) <
				sizeof(xfs_disk_dquot_t)) {
			printf("Out of space\n");
		}
		else {
			printf("\n");
			printf("ver: %d  flags: 0x%x  id: %d  \n",
				INT_GET(dq->d_version, ARCH_CONVERT),
				INT_GET(dq->d_flags, ARCH_CONVERT),
				INT_GET(dq->d_id, ARCH_CONVERT));
			printf("blk limits  hard: %llu  soft: %llu\n",
				(unsigned long long)
				INT_GET(dq->d_blk_hardlimit, ARCH_CONVERT),
				(unsigned long long)
				INT_GET(dq->d_blk_softlimit, ARCH_CONVERT));
			printf("blk  count: %llu  warns: %d  timer: %d\n",
				(unsigned long long)
				INT_GET(dq->d_bcount, ARCH_CONVERT),
				INT_GET(dq->d_bwarns, ARCH_CONVERT),
				INT_GET(dq->d_btimer, ARCH_CONVERT));
			printf("ino limits  hard: %llu  soft: %llu\n",
				(unsigned long long)
				INT_GET(dq->d_ino_hardlimit, ARCH_CONVERT),
				(unsigned long long)
				INT_GET(dq->d_ino_softlimit, ARCH_CONVERT));
			printf("ino  count: %llu  warns: %d  timer: %d\n",
				(unsigned long long)
				INT_GET(dq->d_icount, ARCH_CONVERT),
				INT_GET(dq->d_iwarns, ARCH_CONVERT),
				INT_GET(dq->d_itimer, ARCH_CONVERT));
		}
	} else {
		printf("BUF DATA\n");
		if (print_data) {
			uint *dp  = (uint *)*ptr;
			int  nums = INT_GET(head->oh_len, ARCH_CONVERT) >> 2;
			int  i = 0;

			while (i < nums) {
				if ((i % 8) == 0)
					printf("%2x ", i);
				printf("%8x ", *dp);
				dp++;
				i++;
				if ((i % 8) == 0)
					printf("\n");
			}
			printf("\n");
		}
	}
	*ptr += INT_GET(head->oh_len, ARCH_CONVERT);
    }
    if (head && head->oh_flags & XLOG_CONTINUE_TRANS)
	skip++;
    return skip;
}	/* xlog_print_trans_buffer */


int
xlog_print_trans_efd(xfs_caddr_t *ptr, uint len)
{
    xfs_efd_log_format_t *f;
    xfs_extent_t	 *ex;
    int			 i;
    xfs_efd_log_format_t lbuf;

    /*
     * bcopy to ensure 8-byte alignment for the long longs in
     * xfs_efd_log_format_t structure
     */
    bcopy(*ptr, &lbuf, sizeof(xfs_efd_log_format_t));
    f = &lbuf;
    *ptr += len;
    if (len >= sizeof(xfs_efd_log_format_t)) {
	printf("EFD:  #regs: %d    num_extents: %d  id: 0x%llx\n",
	       f->efd_size, f->efd_nextents, (unsigned long long)f->efd_efi_id);
	ex = f->efd_extents;
	for (i=0; i< f->efd_size; i++) {
		printf("(s: 0x%llx, l: %d) ",
			(unsigned long long)ex->ext_start, ex->ext_len);
		if (i % 4 == 3) printf("\n");
		ex++;
	}
	if (i % 4 != 0) printf("\n");
	return 0;
    } else {
	printf("EFD: Not enough data to decode further\n");
	return 1;
    }
}	/* xlog_print_trans_efd */


int
xlog_print_trans_efi(xfs_caddr_t *ptr, uint len)
{
    xfs_efi_log_format_t *f;
    xfs_extent_t	 *ex;
    int			 i;
    xfs_efi_log_format_t lbuf;

    /*
     * bcopy to ensure 8-byte alignment for the long longs in
     * xfs_efi_log_format_t structure
     */
    bcopy(*ptr, &lbuf, sizeof(xfs_efi_log_format_t));
    f = &lbuf;
    *ptr += len;
    if (len >= sizeof(xfs_efi_log_format_t)) {
	printf("EFI:  #regs: %d    num_extents: %d  id: 0x%llx\n",
	       f->efi_size, f->efi_nextents, (unsigned long long)f->efi_id);
	ex = f->efi_extents;
	for (i=0; i< f->efi_size; i++) {
		printf("(s: 0x%llx, l: %d) ",
			(unsigned long long)ex->ext_start, ex->ext_len);
		if (i % 4 == 3) printf("\n");
		ex++;
	}
	if (i % 4 != 0) printf("\n");
	return 0;
    } else {
	printf("EFI: Not enough data to decode further\n");
	return 1;
    }
}	/* xlog_print_trans_efi */


int
xlog_print_trans_qoff(xfs_caddr_t *ptr, uint len)
{
    xfs_qoff_logformat_t *f;
    xfs_qoff_logformat_t lbuf;

    bcopy(*ptr, &lbuf, sizeof(xfs_qoff_logformat_t));
    f = &lbuf;
    *ptr += len;
    if (len >= sizeof(xfs_qoff_logformat_t)) {
	printf("QOFF:  #regs: %d    flags: 0x%x\n", f->qf_size, f->qf_flags);
	return 0;
    } else {
	printf("QOFF: Not enough data to decode further\n");
	return 1;
    }
}	/* xlog_print_trans_qoff */


void
xlog_print_trans_inode_core(xfs_dinode_core_t *ip)
{
    printf("INODE CORE\n");
    printf("magic 0x%hx mode 0%ho version %d format %d\n",
	   ip->di_magic, ip->di_mode, (int)ip->di_version,
	   (int)ip->di_format);
    printf("nlink %hd uid %d gid %d\n",
	   ip->di_nlink, ip->di_uid, ip->di_gid);
    printf("atime 0x%x mtime 0x%x ctime 0x%x\n",
	   ip->di_atime.t_sec, ip->di_mtime.t_sec, ip->di_ctime.t_sec);
    printf("size 0x%llx nblocks 0x%llx extsize 0x%x nextents 0x%x\n",
	   (unsigned long long)ip->di_size, (unsigned long long)ip->di_nblocks,
	   ip->di_extsize, ip->di_nextents);
    printf("naextents 0x%x forkoff %d dmevmask 0x%x dmstate 0x%hx\n",
	   ip->di_anextents, (int)ip->di_forkoff, ip->di_dmevmask,
	   ip->di_dmstate);
    printf("flags 0x%x gen 0x%x\n",
	   ip->di_flags, ip->di_gen);
}

void
xlog_print_dir_sf(xfs_dir_shortform_t *sfp, int size)
{
	xfs_ino_t	ino;
	int		count;
	int		i;
	char		namebuf[257];
	xfs_dir_sf_entry_t	*sfep;

	/* XXX need to determine whether this is v1 or v2, then
	   print appropriate structure */

	printf("SHORTFORM DIRECTORY size %d\n",
		size);
	/* bail out for now */

	return;

	printf("SHORTFORM DIRECTORY size %d count %d\n",
	       size, sfp->hdr.count);
	bcopy(&(sfp->hdr.parent), &ino, sizeof(ino));
	printf(".. ino 0x%llx\n", (unsigned long long)INT_GET(ino, ARCH_CONVERT));

	count = (uint)(sfp->hdr.count);
	sfep = &(sfp->list[0]);
	for (i = 0; i < count; i++) {
		bcopy(&(sfep->inumber), &ino, sizeof(ino));
		bcopy((sfep->name), namebuf, sfep->namelen);
		namebuf[sfep->namelen] = '\0';
		printf("%s ino 0x%llx namelen %d\n",
		       namebuf, (unsigned long long)ino, sfep->namelen);
		sfep = XFS_DIR_SF_NEXTENTRY(sfep);
	}
}

int
xlog_print_trans_inode(xfs_caddr_t *ptr, int len, int *i, int num_ops)
{
    xfs_inode_log_format_t *f;
    xfs_inode_log_format_t_v1 *old_f;
    xfs_dinode_core_t	   dino;
    xlog_op_header_t	   *op_head;
    int			   version;
    xfs_inode_log_format_t lbuf = {0};
    int			   mode;
    int			   size;

    /*
     * print inode type header region
     *
     * bcopy to ensure 8-byte alignment for the long longs in
     * xfs_inode_log_format_t structure
     *
     * len can be smaller than xfs_inode_log_format_t sometimes... (?)
     */
    bcopy(*ptr, &lbuf, MIN(sizeof(xfs_inode_log_format_t), len));
    version = lbuf.ilf_type;
    f = &lbuf;
    (*i)++;					/* bump index */
    *ptr += len;
    if (version == XFS_LI_5_3_INODE) {
	old_f = (xfs_inode_log_format_t_v1 *)f;
	if (len == sizeof(xfs_inode_log_format_t_v1)) {
	    printf("5.3 INODE: #regs: %d   ino: 0x%llx  flags: 0x%x   dsize: %d\n",
		   old_f->ilf_size, (unsigned long long)old_f->ilf_ino,
		   old_f->ilf_fields, old_f->ilf_dsize);
	} else {
	    ASSERT(len >= 4);	/* must have at least 4 bytes if != 0 */
	    printf("5.3 INODE: #regs: %d   Not printing rest of data\n",
		   old_f->ilf_size);
	    return old_f->ilf_size;
	}
    } else {
	if (len == sizeof(xfs_inode_log_format_t)) {
	    if (version == XFS_LI_6_1_INODE)
		printf("6.1 INODE: ");
	    else printf("INODE: ");
	    printf("#regs: %d   ino: 0x%llx  flags: 0x%x   dsize: %d\n",
		   f->ilf_size, (unsigned long long)f->ilf_ino,
		   f->ilf_fields, f->ilf_dsize);
	    printf("        blkno: %lld  len: %d  boff: %d\n",
		   (long long)f->ilf_blkno, f->ilf_len, f->ilf_boffset);
	} else {
	    ASSERT(len >= 4);	/* must have at least 4 bytes if != 0 */
	    printf("INODE: #regs: %d   Not printing rest of data\n",
		   f->ilf_size);
	    return f->ilf_size;
	}
    }

    if (*i >= num_ops)			/* end of LR */
	    return f->ilf_size-1;

    /* core inode comes 2nd */
    op_head = (xlog_op_header_t *)*ptr;
    xlog_print_op_header(op_head, *i, ptr);

    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS))  {
	return f->ilf_size-1;
    }

    bcopy(*ptr, &dino, sizeof(dino));
    mode = dino.di_mode & S_IFMT;
    size = (int)dino.di_size;
    xlog_print_trans_inode_core(&dino);
    *ptr += sizeof(xfs_dinode_core_t);

    if (*i == num_ops-1 && f->ilf_size == 3)  {
	return 1;
    }

    /* does anything come next */
    op_head = (xlog_op_header_t *)*ptr;
    switch (f->ilf_fields & XFS_ILOG_NONCORE) {
	case XFS_ILOG_DEXT: {
	    ASSERT(f->ilf_size == 3);
	    (*i)++;
	    xlog_print_op_header(op_head, *i, ptr);
	    printf("EXTENTS inode data\n");
	    *ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS))  {
		return 1;
	    }
	    break;
	}
	case XFS_ILOG_DBROOT: {
	    ASSERT(f->ilf_size == 3);
	    (*i)++;
	    xlog_print_op_header(op_head, *i, ptr);
	    printf("BTREE inode data\n");
	    *ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS))  {
		return 1;
	    }
	    break;
	}
	case XFS_ILOG_DDATA: {
	    ASSERT(f->ilf_size == 3);
	    (*i)++;
	    xlog_print_op_header(op_head, *i, ptr);
	    printf("LOCAL inode data\n");
	    if (mode == S_IFDIR) {
		xlog_print_dir_sf((xfs_dir_shortform_t*)*ptr, size);
	    }
	    *ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS)) {
		return 1;
	    }
	    break;
	}
	case XFS_ILOG_AEXT: {
	    ASSERT(f->ilf_size == 3);
	    (*i)++;
	    xlog_print_op_header(op_head, *i, ptr);
	    printf("EXTENTS inode attr\n");
	    *ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS))  {
		return 1;
	    }
	    break;
	}
	case XFS_ILOG_ABROOT: {
	    ASSERT(f->ilf_size == 3);
	    (*i)++;
	    xlog_print_op_header(op_head, *i, ptr);
	    printf("BTREE inode attr\n");
	    *ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS))  {
		return 1;
	    }
	    break;
	}
	case XFS_ILOG_ADATA: {
	    ASSERT(f->ilf_size == 3);
	    (*i)++;
	    xlog_print_op_header(op_head, *i, ptr);
	    printf("LOCAL inode attr\n");
	    if (mode == S_IFDIR) {
		xlog_print_dir_sf((xfs_dir_shortform_t*)*ptr, size);
	    }
	    *ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    if (XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS)) {
		return 1;
	    }
	    break;
	}
	case XFS_ILOG_DEV: {
	    ASSERT(f->ilf_size == 2);
	    printf("DEV inode: no extra region\n");
	    break;
	}
	case XFS_ILOG_UUID: {
	    ASSERT(f->ilf_size == 2);
	    printf("UUID inode: no extra region\n");
	    break;
	}
	case 0: {
	    ASSERT(f->ilf_size == 2);
	    break;
	}
	default: {
	    xlog_panic("xlog_print_trans_inode: illegal inode type");
	}
    }
    return 0;
}	/* xlog_print_trans_inode */


int
xlog_print_trans_dquot(xfs_caddr_t *ptr, int len, int *i, int num_ops)
{
    xfs_dq_logformat_t	*f;
    xfs_dq_logformat_t	lbuf = {0};
    xfs_disk_dquot_t	ddq;
    xlog_op_header_t	*head = NULL;
    int			num, skip;

    /*
     * print dquot header region
     *
     * bcopy to ensure 8-byte alignment for the long longs in
     * xfs_dq_logformat_t structure
     */
    bcopy(*ptr, &lbuf, MIN(sizeof(xfs_dq_logformat_t), len));
    f = &lbuf;
    (*i)++;					/* bump index */
    *ptr += len;

    if (len == sizeof(xfs_dq_logformat_t)) {
	printf("#regs: %d   id: 0x%x", f->qlf_size, f->qlf_id);
	printf("  blkno: %lld  len: %d  boff: %d\n",
		(long long)f->qlf_blkno, f->qlf_len, f->qlf_boffset);
    } else {
	ASSERT(len >= 4);	/* must have at least 4 bytes if != 0 */
	printf("DQUOT: #regs: %d   Not printing rest of data\n",
		f->qlf_size);
	return f->qlf_size;
    }
    num = f->qlf_size-1;

    /* Check if all regions in this log item were in the given LR ptr */
    if (*i+num > num_ops-1) {
	skip = num - (num_ops-1-*i);
	num = num_ops-1-*i;
    } else {
	skip = 0;
    }

    while (num-- > 0) {
	head = (xlog_op_header_t *)*ptr;
	xlog_print_op_header(head, *i, ptr);
	ASSERT(INT_GET(head->oh_len, ARCH_CONVERT) == sizeof(xfs_disk_dquot_t));
	bcopy(*ptr, &ddq, sizeof(xfs_disk_dquot_t));
	printf("DQUOT: magic 0x%hx flags 0%ho\n",
	       INT_GET(ddq.d_magic, ARCH_CONVERT),
	       INT_GET(ddq.d_flags, ARCH_CONVERT));
	*ptr += INT_GET(head->oh_len, ARCH_CONVERT);
    }
    if (head && head->oh_flags & XLOG_CONTINUE_TRANS)
	skip++;
    return skip;
}	/* xlog_print_trans_dquot */


/******************************************************************************
 *
 *		Log print routines
 *
 ******************************************************************************
 */

void
xlog_print_lseek(xlog_t *log, int fd, xfs_daddr_t blkno, int whence)
{
#define BBTOOFF64(bbs)	(((xfs_off_t)(bbs)) << BBSHIFT)
	xfs_off_t offset;

	if (whence == SEEK_SET)
		offset = BBTOOFF64(blkno+log->l_logBBstart);
	else
		offset = BBTOOFF64(blkno);
	if (lseek64(fd, offset, whence) < 0) {
		fprintf(stderr, "%s: lseek64 to %lld failed: %s\n",
			progname, (long long)offset, strerror(errno));
		exit(1);
	}
}	/* xlog_print_lseek */


void
print_lsn(xfs_caddr_t	string,
	  xfs_lsn_t	*lsn,
	  xfs_arch_t    arch)
{
    printf("%s: %u,%u", string,
	    CYCLE_LSN(*lsn, arch), BLOCK_LSN(*lsn, arch));
}


int
xlog_print_record(int		  fd,
		 int		  num_ops,
		 int		  len,
		 int		  *read_type,
		 xfs_caddr_t	  *partial_buf,
		 xlog_rec_header_t *rhead)
{
    xlog_op_header_t	*op_head;
    xlog_rec_header_t	*rechead;
    xfs_caddr_t		buf, ptr;
    int			read_len, skip;
    int			ret, n, i;

    if (print_no_print)
	    return NO_ERROR;

    if (!len) {
	printf("\n");
	return NO_ERROR;
    }

    /* read_len must read up to some block boundary */
    read_len = (int) BBTOB(BTOBB(len));

    /* read_type => don't malloc() new buffer, use old one */
    if (*read_type == FULL_READ) {
	if ((ptr = buf = (xfs_caddr_t)malloc(read_len)) == NULL) {
	    fprintf(stderr, "xlog_print_record: malloc failed\n");
	    exit(1);
	}
    } else {
	read_len -= *read_type;
	buf = (xfs_caddr_t)((__psint_t)(*partial_buf) + (__psint_t)(*read_type));
	ptr = *partial_buf;
    }
    if ((ret = (int) read(fd, buf, read_len)) == -1) {
	fprintf(stderr, "xlog_print_record: read error\n");
	exit(1);
    }
    /* Did we overflow the end? */
    if (*read_type == FULL_READ &&
	BLOCK_LSN(rhead->h_lsn, ARCH_CONVERT)+BTOBB(read_len) >= logBBsize) {
	*read_type = BBTOB(logBBsize-BLOCK_LSN(rhead->h_lsn, ARCH_CONVERT)-1);
	*partial_buf = buf;
	return PARTIAL_READ;
    }

    /* Did we read everything? */
    if ((ret == 0 && read_len != 0) || ret != read_len) {
	*read_type = ret;
	*partial_buf = buf;
	return PARTIAL_READ;
    }
    if (*read_type != FULL_READ)
	read_len += *read_type;

    /* Everything read in.  Start from beginning of buffer */
    buf = ptr;
    for (i = 0; ptr < buf + read_len; ptr += BBSIZE, i++) {
	rechead = (xlog_rec_header_t *)ptr;
	if (INT_GET(rechead->h_magicno, ARCH_CONVERT) == XLOG_HEADER_MAGIC_NUM) {
	    free(buf);
	    return -1;
	} else {
	    if (INT_GET(rhead->h_cycle, ARCH_CONVERT) !=
			INT_GET(*(uint *)ptr, ARCH_CONVERT)) {
		if (*read_type == FULL_READ)
		    return -1;
		else if (INT_GET(rhead->h_cycle, ARCH_CONVERT) + 1 !=
			INT_GET(*(uint *)ptr, ARCH_CONVERT))
		    return -1;
	    }
	}
	INT_SET(*(uint *)ptr, ARCH_CONVERT,
		INT_GET(rhead->h_cycle_data[i], ARCH_CONVERT));
    }
    ptr = buf;
    for (i=0; i<num_ops; i++) {
	print_xlog_op_line();
	op_head = (xlog_op_header_t *)ptr;
	xlog_print_op_header(op_head, i, &ptr);

	/* print transaction data */
	if (print_no_data ||
	    ((XLOG_SET(op_head->oh_flags, XLOG_WAS_CONT_TRANS) ||
	      XLOG_SET(op_head->oh_flags, XLOG_CONTINUE_TRANS)) &&
	     INT_GET(op_head->oh_len, ARCH_CONVERT) == 0)) {
	    for (n = 0; n < INT_GET(op_head->oh_len, ARCH_CONVERT); n++) {
		printf("%c", *ptr);
		ptr++;
	    }
	    printf("\n");
	    continue;
	}
	if (xlog_print_find_tid(INT_GET(op_head->oh_tid, ARCH_CONVERT),
				op_head->oh_flags & XLOG_WAS_CONT_TRANS)) {
	    printf("Left over region from split log item\n");
	    ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
	    continue;
	}
	if (INT_GET(op_head->oh_len, ARCH_CONVERT) != 0) {
	    if (*(uint *)ptr == XFS_TRANS_HEADER_MAGIC) {
		skip = xlog_print_trans_header(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT));
	    } else {
		switch (*(unsigned short *)ptr) {
		    case XFS_LI_5_3_BUF:
		    case XFS_LI_6_1_BUF:
		    case XFS_LI_BUF: {
			skip = xlog_print_trans_buffer(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT),
					&i, num_ops);
			break;
		    }
		    case XFS_LI_5_3_INODE:
		    case XFS_LI_6_1_INODE:
		    case XFS_LI_INODE: {
			skip = xlog_print_trans_inode(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT),
					&i, num_ops);
			break;
		    }
		    case XFS_LI_DQUOT: {
			skip = xlog_print_trans_dquot(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT),
					&i, num_ops);
			break;
		    }
		    case XFS_LI_EFI: {
			skip = xlog_print_trans_efi(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT));
			break;
		    }
		    case XFS_LI_EFD: {
			skip = xlog_print_trans_efd(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT));
			break;
		    }
		    case XFS_LI_QUOTAOFF: {
			skip = xlog_print_trans_qoff(&ptr,
					INT_GET(op_head->oh_len, ARCH_CONVERT));
			break;
		    }
		    case XLOG_UNMOUNT_TYPE: {
			printf("Unmount filesystem\n");
			skip = 0;
			break;
		    }
		    default: {
			fprintf(stderr, "%s: unknown log operation type (%x)\n",
				progname, *(unsigned short *)ptr);
			skip = 0;
			ptr += INT_GET(op_head->oh_len, ARCH_CONVERT);
		    }
		} /* switch */
	    } /* else */
	    if (skip != 0)
		xlog_print_add_to_trans(INT_GET(op_head->oh_tid, ARCH_CONVERT), skip);
	}
    }
    printf("\n");
    free(buf);
    return NO_ERROR;
}	/* xlog_print_record */


int
xlog_print_rec_head(xlog_rec_header_t *head, int *len)
{
    int i;
    char uub[64];
    int datalen,bbs;

    if (print_no_print)
	    return INT_GET(head->h_num_logops, ARCH_CONVERT);

    if (INT_ISZERO(head->h_magicno, ARCH_CONVERT))
	return ZEROED_LOG;

    if (INT_GET(head->h_magicno, ARCH_CONVERT) != XLOG_HEADER_MAGIC_NUM) {
	printf("Header 0x%x wanted 0x%x\n",
		INT_GET(head->h_magicno, ARCH_CONVERT),
		XLOG_HEADER_MAGIC_NUM);
	return BAD_HEADER;
    }

    datalen=INT_GET(head->h_len, ARCH_CONVERT);
    bbs=(datalen/BBSIZE)+(datalen%BBSIZE)?1:0;

    printf("cycle: %d	version: %d	",
	    INT_GET(head->h_cycle, ARCH_CONVERT),
	    INT_GET(head->h_version, ARCH_CONVERT));
    print_lsn("	lsn", &head->h_lsn, ARCH_CONVERT);
    print_lsn("	tail_lsn", &head->h_tail_lsn, ARCH_CONVERT);
    printf("\n");
    printf("length of Log Record: %d	prev offset: %d		num ops: %d\n",
	   datalen,
	    INT_GET(head->h_prev_block, ARCH_CONVERT),
	    INT_GET(head->h_num_logops, ARCH_CONVERT));

    if (print_overwrite) {
	printf("cycle num overwrites: ");
	for (i=0; i< bbs; i++)
	    printf("%d - 0x%x  ",
		    i,
		    INT_GET(head->h_cycle_data[i], ARCH_CONVERT));
	printf("\n");
    }

    uuid_unparse(head->h_fs_uuid, uub);
    printf("uuid: %s   format: ", uub);
    switch (INT_GET(head->h_fmt, ARCH_CONVERT)) {
	case XLOG_FMT_UNKNOWN:
	    printf("unknown\n");
	    break;
	case XLOG_FMT_LINUX_LE:
	    printf("little endian linux\n");
	    break;
	case XLOG_FMT_LINUX_BE:
	    printf("big endian linux\n");
	    break;
	case XLOG_FMT_IRIX_BE:
	    printf("big endian irix\n");
	    break;
	default:
	    printf("? (%d)\n", INT_GET(head->h_fmt, ARCH_CONVERT));
	    break;
    }

    *len = INT_GET(head->h_len, ARCH_CONVERT);
    return(INT_GET(head->h_num_logops, ARCH_CONVERT));
}	/* xlog_print_rec_head */

static void
print_xlog_bad_zeroed(xfs_daddr_t blkno)
{
	print_stars();
	printf("* ERROR: found data after zeroed blocks block=%-21lld  *\n",
		(long long)blkno);
	print_stars();
	if (print_exit)
	    xlog_exit("Bad log - data after zeroed blocks");
}	/* print_xlog_bad_zeroed */

static void
print_xlog_bad_header(xfs_daddr_t blkno, xfs_caddr_t buf)
{
	print_stars();
	printf("* ERROR: header cycle=%-11d block=%-21lld        *\n",
		GET_CYCLE(buf, ARCH_CONVERT), (long long)blkno);
	print_stars();
	if (print_exit)
	    xlog_exit("Bad log record header");
}	/* print_xlog_bad_header */

void
print_xlog_bad_data(xfs_daddr_t blkno)
{
	print_stars();
	printf("* ERROR: data block=%-21lld                             *\n",
		(long long)blkno);
	print_stars();
	if (print_exit)
	    xlog_exit("Bad data in log");
}	/* print_xlog_bad_data */


/*
 * This code is gross and needs to be rewritten.
 */
void xfs_log_print(xlog_t       *log,
		   int          fd,
		   int		print_block_start)
{
    char	hbuf[XLOG_HEADER_SIZE];
    int		num_ops, len;
    xfs_daddr_t	block_end = 0, block_start, blkno, error;
    int		read_type = FULL_READ;
    xfs_caddr_t	partial_buf;
    int         zeroed = 0;

    logBBsize = log->l_logBBsize;

    /*
     * Normally, block_start and block_end are the same value since we
     * are printing the entire log.  However, if the start block is given,
     * we still end at the end of the logical log.
     */
    if ((error = xlog_print_find_oldest(log, &block_end))) {
	    fprintf(stderr, "%s: problem finding oldest LR\n", progname);
	    return;
    }
    if (print_block_start == -1)
	    block_start = block_end;
    else
	    block_start = print_block_start;
    xlog_print_lseek(log, fd, block_start, SEEK_SET);
    blkno    = block_start;

    for (;;) {
	if (read(fd, hbuf, 512) == 0) {
	    printf("%s: physical end of log\n", progname);
	    print_xlog_record_line();
	    break;
	}
	if (print_only_data) {
		printf("BLKNO: %lld\n", (long long)blkno);
		xlog_recover_print_data(hbuf, 512);
		blkno++;
		goto loop;
	}
	num_ops = xlog_print_rec_head((xlog_rec_header_t *)hbuf, &len);
	blkno++;

	if (zeroed && num_ops != ZEROED_LOG) {
	    printf("%s: after %d zeroed blocks\n", progname, zeroed);
	    /* once we find zeroed blocks - that's all we expect */
	    print_xlog_bad_zeroed(blkno-1);
	    /* reset count since we're assuming previous zeroed blocks
	     * were bad
	     */
	    zeroed = 0;
	}

	if (num_ops == ZEROED_LOG || num_ops == BAD_HEADER) {
	    if (num_ops == ZEROED_LOG) {
		zeroed++;
	    } else {
		print_xlog_bad_header(blkno-1, hbuf);
	    }

	    goto loop;
	}

	error =	xlog_print_record(fd, num_ops, len, &read_type, &partial_buf,
				  (xlog_rec_header_t *)hbuf);
	switch (error) {
	    case 0: {
		blkno += BTOBB(len);
		if (print_block_start != -1 &&
		    blkno >= block_end)		/* If start specified, we */
			goto end;		/* end early */
		break;
	    }
	    case -1: {
		print_xlog_bad_data(blkno-1);
		if (print_block_start != -1 &&
		    blkno >= block_end)		/* If start specified, */
			goto end;		/* we end early */
		xlog_print_lseek(log, fd, blkno, SEEK_SET);
		goto loop;
	    }
	    case PARTIAL_READ: {
		print_xlog_record_line();
		printf("%s: physical end of log\n", progname);
		print_xlog_record_line();
		blkno = 0;
		xlog_print_lseek(log, fd, 0, SEEK_SET);
		/*
		 * We may have hit the end of the log when we started at 0.
		 * In this case, just end.
		 */
		if (block_start == 0)
			goto end;
		goto partial_log_read;
	    }
	    default: xlog_panic("illegal value");
	}
	print_xlog_record_line();
loop:
	if (blkno >= logBBsize) {
		if (zeroed) {
		    printf("%s: skipped %d zeroed blocks\n", progname, zeroed);
		    if (zeroed == logBBsize)
			printf("%s: totally zeroed log\n", progname);

		    zeroed=0;
		}
		printf("%s: physical end of log\n", progname);
		print_xlog_record_line();
		break;
	}
    }

    /* Do we need to print the first part of physical log? */
    if (block_start != 0) {
	blkno = 0;
	xlog_print_lseek(log, fd, 0, SEEK_SET);
	for (;;) {
	    if (read(fd, hbuf, 512) == 0) {
		xlog_panic("xlog_find_head: bad read");
	    }
	    if (print_only_data) {
		printf("BLKNO: %lld\n", (long long)blkno);
		xlog_recover_print_data(hbuf, 512);
		blkno++;
		goto loop2;
	    }
	    num_ops = xlog_print_rec_head((xlog_rec_header_t *)hbuf, &len);
	    blkno++;

	    if (num_ops == ZEROED_LOG || num_ops == BAD_HEADER) {
		/* we only expect zeroed log entries at the end
		 * of the _physical_ log, so treat them the same
		 * as bad blocks here
		 */
		print_xlog_bad_header(blkno-1, hbuf);

		if (blkno >= block_end)
		    break;
		continue;
	    }
partial_log_read:
	    error= xlog_print_record(fd, num_ops, len, &read_type,
				    &partial_buf, (xlog_rec_header_t *)hbuf);
	    if (read_type != FULL_READ)
		len -= read_type;
	    read_type = FULL_READ;
	    if (!error)
		blkno += BTOBB(len);
	    else {
		print_xlog_bad_data(blkno-1);
		xlog_print_lseek(log, fd, blkno, SEEK_SET);
		goto loop2;
	    }
	    print_xlog_record_line();
loop2:
	    if (blkno >= block_end)
		break;
	}
    }

end:
    printf("%s: logical end of log\n", progname);
    print_xlog_record_line();
}
