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

#include <xfs/libxlog.h>

int print_exit;
int print_record_header;
libxfs_init_t x;

static int
header_check_uuid(xfs_mount_t *mp, xlog_rec_header_t *head)
{
    char uu_log[64], uu_sb[64];

    if (!uuid_compare(mp->m_sb.sb_uuid, head->h_fs_uuid)) return 0;

    uuid_unparse(mp->m_sb.sb_uuid, uu_sb);
    uuid_unparse(head->h_fs_uuid, uu_log);

    printf(_("* ERROR: mismatched uuid in log\n"
	     "*            SB : %s\n*            log: %s\n"),
	    uu_sb, uu_log);

    memcpy(&mp->m_sb.sb_uuid, &head->h_fs_uuid, sizeof(uuid_t));

    return 0;
}

int
xlog_header_check_recover(xfs_mount_t *mp, xlog_rec_header_t *head)
{
    if (print_record_header)
	printf(_("\nLOG REC AT LSN cycle %d block %d (0x%x, 0x%x)\n"),
	       CYCLE_LSN(head->h_lsn, ARCH_CONVERT),
	       BLOCK_LSN(head->h_lsn, ARCH_CONVERT),
	       CYCLE_LSN(head->h_lsn, ARCH_CONVERT),
	       BLOCK_LSN(head->h_lsn, ARCH_CONVERT));

    if (INT_GET(head->h_magicno, ARCH_CONVERT) != XLOG_HEADER_MAGIC_NUM) {

	printf(_("* ERROR: bad magic number in log header: 0x%x\n"),
		INT_GET(head->h_magicno, ARCH_CONVERT));

    } else if (header_check_uuid(mp, head)) {

	/* failed - fall through */

    } else if (INT_GET(head->h_fmt, ARCH_CONVERT) != XLOG_FMT) {

	printf(_("* ERROR: log format incompatible (log=%d, ours=%d)\n"),
		INT_GET(head->h_fmt, ARCH_CONVERT), XLOG_FMT);

    } else {
	/* everything is ok */
	return 0;
    }

    /* bail out now or just carry on regardless */
    if (print_exit)
	xlog_exit(_("Bad log"));

    return 0;
}

int
xlog_header_check_mount(xfs_mount_t *mp, xlog_rec_header_t *head)
{
    if (uuid_is_null(head->h_fs_uuid)) return 0;
    if (header_check_uuid(mp, head)) {
	/* bail out now or just carry on regardless */
	if (print_exit)
	    xlog_exit(_("Bad log"));
    }
    return 0;
}
