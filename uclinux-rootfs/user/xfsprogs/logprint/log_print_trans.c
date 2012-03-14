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

#include "logprint.h"

void
xlog_recover_print_trans_head(
	xlog_recover_t	*tr)
{
	printf("TRANS: tid:0x%x  type:%s  #items:%d  trans:0x%x  q:0x%lx\n",
	       tr->r_log_tid, trans_type[tr->r_theader.th_type],
	       tr->r_theader.th_num_items,
	       tr->r_theader.th_tid, (long)tr->r_itemq);
}

int
xlog_recover_do_trans(
	xlog_t		*log,
	xlog_recover_t	*trans,
	int		pass)
{
	xlog_recover_print_trans(trans, trans->r_itemq, 3);
	return 0;
}

void
xfs_log_print_trans(
	xlog_t		*log,
	int		print_block_start)
{
	xfs_daddr_t	head_blk, tail_blk;

	if (xlog_find_tail(log, &head_blk, &tail_blk, 0))
		exit(1);

	printf("    log tail: %lld head: %lld state: %s\n",
		(long long)tail_blk,
		(long long)head_blk,
		(tail_blk == head_blk)?"<CLEAN>":"<DIRTY>");

	if (print_block_start != -1) {
		printf("    override tail: %d\n", print_block_start);
		tail_blk = print_block_start;
	}
	printf("\n");

	print_record_header = 1;

	if (head_blk == tail_blk)
		return;
	if (xlog_do_recovery_pass(log, head_blk, tail_blk, XLOG_RECOVER_PASS1))
		exit(1);
}
