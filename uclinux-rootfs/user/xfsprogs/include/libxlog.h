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
#ifndef LIBXLOG_H
#define LIBXLOG_H

#include <xfs/libxfs.h>

/*
 * define the userlevel xlog_t to be the subset of the kernel's
 * xlog_t that we actually need to get our work done, avoiding
 * the need to define any exotic kernel types in userland.
 */
typedef struct log {
	xfs_lsn_t	l_tail_lsn;     /* lsn of 1st LR w/ unflush buffers */
	xfs_lsn_t	l_last_sync_lsn;/* lsn of last LR on disk */
	xfs_mount_t	*l_mp;	        /* mount point */
	dev_t		l_dev;	        /* dev_t of log */
	xfs_daddr_t	l_logBBstart;   /* start block of log */
	int		l_logsize;      /* size of log in bytes */
	int		l_logBBsize;    /* size of log in 512 byte chunks */
	int		l_curr_cycle;   /* Cycle number of log writes */
	int		l_prev_cycle;   /* Cycle # b4 last block increment */
	int		l_curr_block;   /* current logical block of log */
	int		l_prev_block;   /* previous logical block of log */
	int		l_iclog_size;	 /* size of log in bytes */
	int		l_iclog_size_log;/* log power size of log */
	int		l_iclog_bufs;	 /* number of iclog buffers */
	int		l_grant_reserve_cycle;	/* */
	int		l_grant_reserve_bytes;	/* */
	int		l_grant_write_cycle;	/* */
	int		l_grant_write_bytes;	/* */
	uint		l_sectbb_log;   /* log2 of sector size in bbs */
} xlog_t;

#include <xfs/xfs_log_recover.h>
#include <xfs/xfs_buf_item.h>
#include <xfs/xfs_inode_item.h>
#include <xfs/xfs_extfree_item.h>

typedef union {
	xlog_rec_header_t       hic_header;
	xlog_rec_ext_header_t   hic_xheader;
	char                    hic_sector[XLOG_HEADER_SIZE];
} xlog_in_core_2_t;

/*
 * macros mapping kernel code to user code
 */
#ifndef EFSCORRUPTED
#define EFSCORRUPTED			 990
#endif
#define STATIC				static
#define XFS_ERROR(e)			(e)
#define XFS_ERROR_REPORT(e,l,mp)	((void) 0)
#define XFS_CORRUPTION_ERROR(e,l,mp,m)	((void) 0)
#define unlikely(x)			(x)
#define min(a,b)			((a) < (b) ? (a) : (b))

#if (__GNUC__ < 2) || ((__GNUC__ == 2) && (__GNUC_MINOR__ <= 95))
# define xlog_warn(fmt,args...) \
	( fprintf(stderr,fmt,## args), fputc('\n', stderr) )
# define cmn_err(sev,fmt,args...) \
	xlog_warn(fmt,## args)
# define xlog_exit(fmt,args...) \
	( xlog_warn(fmt,## args), exit(1) )
# define xlog_panic(fmt,args...) \
	xlog_exit(fmt,## args)
#else
# define xlog_warn(...) \
	( fprintf(stderr,__VA_ARGS__), fputc('\n', stderr) )
# define cmn_err(sev,...) \
	xlog_warn(__VA_ARGS__)
# define xlog_exit(...) \
	( xlog_warn(__VA_ARGS__), exit(1) )
# define xlog_panic(...) \
	xlog_exit(__VA_ARGS__)
#endif

#define xlog_get_bp(log,bbs)	libxfs_getbuf(x.logdev, 0, (bbs))
#define xlog_put_bp(bp)		libxfs_putbuf(bp)
#define xlog_bread(log,blkno,bbs,bp)	\
	(libxfs_readbufr(x.logdev,	\
			(log)->l_logBBstart+(blkno), bp, (bbs), 1), 0)
#define xlog_align(log,blkno,nbblks,bp)	XFS_BUF_PTR(bp)

#define kmem_zalloc(size, foo)			calloc(size,1)
#define kmem_alloc(size, foo)			calloc(size,1)
#define kmem_free(ptr, foo)			free(ptr)
#define kmem_realloc(ptr, len, old, foo)	realloc(ptr, len)

/* exports */
extern int	print_record_header;
extern int	print_exit;

/* libxfs parameters */
extern libxfs_init_t	x;

extern void xfs_log_print_trans(xlog_t          *log,
				int		print_block_start);

extern void xfs_log_print(      xlog_t          *log,
				int             fd,
				int		print_block_start);

extern int  xlog_find_zeroed(xlog_t *log, xfs_daddr_t *blk_no);
extern int  xlog_find_cycle_start(xlog_t *log, xfs_buf_t *bp,
		xfs_daddr_t first_blk, xfs_daddr_t *last_blk, uint cycle);
extern int  xlog_find_tail(xlog_t *log, xfs_daddr_t *head_blk,
		xfs_daddr_t *tail_blk, int readonly);

extern int  xlog_test_footer(xlog_t *log);
extern int  xlog_recover(xlog_t *log, int readonly);
extern void xlog_recover_print_data(xfs_caddr_t p, int len);
extern void xlog_recover_print_logitem(xlog_recover_item_t *item);
extern void xlog_recover_print_trans_head(xlog_recover_t *tr);
extern int  xlog_print_find_oldest(xlog_t *log, xfs_daddr_t *last_blk);

extern void print_xlog_op_line(void);
extern void print_xlog_record_line(void);
extern void print_stars(void);

/* for transactional view */
extern void xlog_recover_print_trans_head(xlog_recover_t *tr);

extern void xlog_recover_print_trans(	xlog_recover_t		*trans,
					xlog_recover_item_t	*itemq,
					int			print);

extern int  xlog_do_recovery_pass(	xlog_t		*log,
					xfs_daddr_t	head_blk,
					xfs_daddr_t	tail_blk,
					int		pass);
extern int  xlog_recover_do_trans(	xlog_t		*log,
					xlog_recover_t	*trans,
					int		pass);
extern int  xlog_header_check_recover(  xfs_mount_t         *mp,
					xlog_rec_header_t   *head);
extern int  xlog_header_check_mount(    xfs_mount_t         *mp,
					xlog_rec_header_t   *head);

#endif	/* LIBXLOG_H */
