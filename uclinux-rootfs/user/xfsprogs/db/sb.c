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

#include <xfs/libxfs.h>
#include <xfs/libxlog.h>
#include "command.h"
#include "type.h"
#include "faddr.h"
#include "fprint.h"
#include "field.h"
#include "io.h"
#include "sb.h"
#include "bit.h"
#include "output.h"
#include "init.h"

static int	sb_f(int argc, char **argv);
static void     sb_help(void);
static int	uuid_f(int argc, char **argv);
static void     uuid_help(void);
static int	label_f(int argc, char **argv);
static void     label_help(void);
static int	version_f(int argc, char **argv);
static void     version_help(void);

static const cmdinfo_t	sb_cmd =
	{ "sb", NULL, sb_f, 0, 1, 1, "[agno]",
	  "set current address to sb header", sb_help };
static const cmdinfo_t	uuid_cmd =
	{ "uuid", NULL, uuid_f, 0, 1, 1, "[uuid]",
	  "write/print FS uuid", uuid_help };
static const cmdinfo_t	label_cmd =
	{ "label", NULL, label_f, 0, 1, 1, "[label]",
	  "write/print FS label", label_help };
static const cmdinfo_t	version_cmd =
	{ "version", NULL, version_f, 0, 1, 1, "[feature]",
	  "set feature bit(s) in the sb version field", version_help };

void
sb_init(void)
{
	add_command(&sb_cmd);
	add_command(&uuid_cmd);
	add_command(&label_cmd);
	add_command(&version_cmd);
}

#define	OFF(f)	bitize(offsetof(xfs_sb_t, sb_ ## f))
#define	SZC(f)	szcount(xfs_sb_t, sb_ ## f)
const field_t	sb_flds[] = {
	{ "magicnum", FLDT_UINT32X, OI(OFF(magicnum)), C1, 0, TYP_NONE },
	{ "blocksize", FLDT_UINT32D, OI(OFF(blocksize)), C1, 0, TYP_NONE },
	{ "dblocks", FLDT_DRFSBNO, OI(OFF(dblocks)), C1, 0, TYP_NONE },
	{ "rblocks", FLDT_DRFSBNO, OI(OFF(rblocks)), C1, 0, TYP_NONE },
	{ "rextents", FLDT_DRTBNO, OI(OFF(rextents)), C1, 0, TYP_NONE },
	{ "uuid", FLDT_UUID, OI(OFF(uuid)), C1, 0, TYP_NONE },
	{ "logstart", FLDT_DFSBNO, OI(OFF(logstart)), C1, 0, TYP_LOG },
	{ "rootino", FLDT_INO, OI(OFF(rootino)), C1, 0, TYP_INODE },
	{ "rbmino", FLDT_INO, OI(OFF(rbmino)), C1, 0, TYP_INODE },
	{ "rsumino", FLDT_INO, OI(OFF(rsumino)), C1, 0, TYP_INODE },
	{ "rextsize", FLDT_AGBLOCK, OI(OFF(rextsize)), C1, 0, TYP_NONE },
	{ "agblocks", FLDT_AGBLOCK, OI(OFF(agblocks)), C1, 0, TYP_NONE },
	{ "agcount", FLDT_AGNUMBER, OI(OFF(agcount)), C1, 0, TYP_NONE },
	{ "rbmblocks", FLDT_EXTLEN, OI(OFF(rbmblocks)), C1, 0, TYP_NONE },
	{ "logblocks", FLDT_EXTLEN, OI(OFF(logblocks)), C1, 0, TYP_NONE },
	{ "versionnum", FLDT_UINT16X, OI(OFF(versionnum)), C1, 0, TYP_NONE },
	{ "sectsize", FLDT_UINT16D, OI(OFF(sectsize)), C1, 0, TYP_NONE },
	{ "inodesize", FLDT_UINT16D, OI(OFF(inodesize)), C1, 0, TYP_NONE },
	{ "inopblock", FLDT_UINT16D, OI(OFF(inopblock)), C1, 0, TYP_NONE },
	{ "fname", FLDT_CHARNS, OI(OFF(fname)), CI(SZC(fname)), 0, TYP_NONE },
	{ "blocklog", FLDT_UINT8D, OI(OFF(blocklog)), C1, 0, TYP_NONE },
	{ "sectlog", FLDT_UINT8D, OI(OFF(sectlog)), C1, 0, TYP_NONE },
	{ "inodelog", FLDT_UINT8D, OI(OFF(inodelog)), C1, 0, TYP_NONE },
	{ "inopblog", FLDT_UINT8D, OI(OFF(inopblog)), C1, 0, TYP_NONE },
	{ "agblklog", FLDT_UINT8D, OI(OFF(agblklog)), C1, 0, TYP_NONE },
	{ "rextslog", FLDT_UINT8D, OI(OFF(rextslog)), C1, 0, TYP_NONE },
	{ "inprogress", FLDT_UINT8D, OI(OFF(inprogress)), C1, 0, TYP_NONE },
	{ "imax_pct", FLDT_UINT8D, OI(OFF(imax_pct)), C1, 0, TYP_NONE },
	{ "icount", FLDT_UINT64D, OI(OFF(icount)), C1, 0, TYP_NONE },
	{ "ifree", FLDT_UINT64D, OI(OFF(ifree)), C1, 0, TYP_NONE },
	{ "fdblocks", FLDT_UINT64D, OI(OFF(fdblocks)), C1, 0, TYP_NONE },
	{ "frextents", FLDT_UINT64D, OI(OFF(frextents)), C1, 0, TYP_NONE },
	{ "uquotino", FLDT_INO, OI(OFF(uquotino)), C1, 0, TYP_INODE },
	{ "gquotino", FLDT_INO, OI(OFF(gquotino)), C1, 0, TYP_INODE },
	{ "qflags", FLDT_UINT16X, OI(OFF(qflags)), C1, 0, TYP_NONE },
	{ "flags", FLDT_UINT8X, OI(OFF(flags)), C1, 0, TYP_NONE },
	{ "shared_vn", FLDT_UINT8D, OI(OFF(shared_vn)), C1, 0, TYP_NONE },
	{ "inoalignmt", FLDT_EXTLEN, OI(OFF(inoalignmt)), C1, 0, TYP_NONE },
	{ "unit", FLDT_UINT32D, OI(OFF(unit)), C1, 0, TYP_NONE },
	{ "width", FLDT_UINT32D, OI(OFF(width)), C1, 0, TYP_NONE },
	{ "dirblklog", FLDT_UINT8D, OI(OFF(dirblklog)), C1, 0, TYP_NONE },
	{ "logsectlog", FLDT_UINT8D, OI(OFF(logsectlog)), C1, 0, TYP_NONE },
	{ "logsectsize", FLDT_UINT16D, OI(OFF(logsectsize)), C1, 0, TYP_NONE },
	{ "logsunit", FLDT_UINT32D, OI(OFF(logsunit)), C1, 0, TYP_NONE },
	{ NULL }
};

const field_t	sb_hfld[] = {
	{ "", FLDT_SB, OI(0), C1, 0, TYP_NONE },
	{ NULL }
};

static void
sb_help(void)
{
	dbprintf(
"\n"
" set allocation group superblock\n"
"\n"
" Example:\n"
"\n"
" 'sb 7' - set location to 7th allocation group superblock, set type to 'sb'\n"
"\n"
" Located in the first sector of each allocation group, the superblock\n"
" contains the base information for the filesystem.\n"
" The superblock in allocation group 0 is the primary.  The copies in the\n"
" remaining allocation groups only serve as backup for filesystem recovery.\n"
" The icount/ifree/fdblocks/frextents are only updated in superblock 0.\n"
"\n"
);
}

static int
sb_f(
	int		argc,
	char		**argv)
{
	xfs_agnumber_t	agno;
	char		*p;

	if (argc > 1) {
		agno = (xfs_agnumber_t)strtoul(argv[1], &p, 0);
		if (*p != '\0' || agno >= mp->m_sb.sb_agcount) {
			dbprintf("bad allocation group number %s\n", argv[1]);
			return 0;
		}
		cur_agno = agno;
	} else if (cur_agno == NULLAGNUMBER)
		cur_agno = 0;
	ASSERT(typtab[TYP_SB].typnm == TYP_SB);
	set_cur(&typtab[TYP_SB],
		XFS_AG_DADDR(mp, cur_agno, XFS_SB_DADDR),
		XFS_FSS_TO_BB(mp, 1), DB_RING_ADD, NULL);
	return 0;
}

/*ARGSUSED*/
int
sb_size(
	void	*obj,
	int	startoff,
	int	idx)
{
	return bitize(mp->m_sb.sb_sectsize);
}

static int
get_sb(xfs_agnumber_t agno, xfs_sb_t *sb)
{
	push_cur();
	set_cur(&typtab[TYP_SB],
		XFS_AG_DADDR(mp, agno, XFS_SB_DADDR),
		XFS_FSS_TO_BB(mp, 1), DB_RING_IGN, NULL);

	if (!iocur_top->data) {
		dbprintf("can't read superblock for AG %u\n", agno);
		pop_cur();
		return 0;
	}

	libxfs_xlate_sb(iocur_top->data, sb, 1, ARCH_CONVERT, XFS_SB_ALL_BITS);

	if (sb->sb_magicnum != XFS_SB_MAGIC) {
		dbprintf("bad sb magic # %#x in AG %u\n",
			sb->sb_magicnum, agno);
		return 0;
	}
	if (!XFS_SB_GOOD_VERSION(sb)) {
		dbprintf("bad sb version # %#x in AG %u\n",
			sb->sb_versionnum, agno);
		return 0;
	}
	if (agno == 0 && sb->sb_inprogress != 0) {
		dbprintf("mkfs not completed successfully\n");
		return 0;
	}
	return 1;
}

/* workaround craziness in the xlog routines */
int xlog_recover_do_trans(xlog_t *log, xlog_recover_t *t, int p) { return 0; }

static int
zero_log(uuid_t *uuidp)
{
	xlog_t		log;
	xfs_daddr_t	head_blk, tail_blk;

	if (mp->m_sb.sb_logstart) {
		if (x.logdev && x.logdev != x.ddev) {
			dbprintf("aborting - external log specified for FS "
				 "with an internal log\n");
			return 0;
		}
	} else {
		if (!x.logdev || (x.logdev == x.ddev)) {
			dbprintf("aborting - no external log specified for FS "
				 "with an external log\n");
			return 0;
		}
	}

	memset(&log, 0, sizeof(log));
	if (!x.logdev)
		x.logdev = x.ddev;
	x.logBBsize = XFS_FSB_TO_BB(mp, mp->m_sb.sb_logblocks);
	x.logBBstart = XFS_FSB_TO_DADDR(mp, mp->m_sb.sb_logstart);
	log.l_dev = (mp->m_sb.sb_logstart == 0) ? x.logdev : x.ddev;
	log.l_logsize = BBTOB(log.l_logBBsize);
	log.l_logBBsize = x.logBBsize;
	log.l_logBBstart = x.logBBstart;
	log.l_mp = mp;

	if (xlog_find_tail(&log, &head_blk, &tail_blk, 0)) {
		dbprintf("ERROR: cannot find log head/tail, run xfs_repair\n");
		return 0;
	}
	if (head_blk != tail_blk) {
		dbprintf(
"ERROR: The filesystem has valuable metadata changes in a log which needs to\n"
"be replayed.  Mount the filesystem to replay the log, and unmount it before\n"
"re-running %s.  If you are unable to mount the filesystem, then use\n"
"the xfs_repair -L option to destroy the log and attempt a repair.\n"
"Note that destroying the log may cause corruption -- please attempt a mount\n"
"of the filesystem before doing this.\n", progname);
		return 0;
	}

	dbprintf("Clearing log and setting UUID\n");

	if (libxfs_log_clear(log.l_dev,
			XFS_FSB_TO_DADDR(mp, mp->m_sb.sb_logstart),
			(xfs_extlen_t)XFS_FSB_TO_BB(mp, mp->m_sb.sb_logblocks),
			uuidp,
			XFS_SB_VERSION_HASLOGV2(&mp->m_sb) ? 2 : 1,
			mp->m_sb.sb_logsunit, XLOG_FMT)) {
		dbprintf("ERROR: cannot clear the log\n");
		return 0;
	}
	return 1;
}


static void
uuid_help(void)
{
	dbprintf(
"\n"
" write/print FS uuid\n"
"\n"
" Example:\n"
"\n"
" 'uuid'                                      - print UUID\n"
" 'uuid 01234567-0123-0123-0123-0123456789ab' - write UUID\n"
" 'uuid generate'                             - generate and write\n"
" 'uuid rewrite'                              - copy UUID from SB 0\n"
"\n"
"The print function checks the UUID in each SB and will warn if the UUIDs\n"
"differ between AGs (the log is not checked). The write commands will\n"
"set the uuid in all AGs to either a specified value, a newly generated\n"
"value or the value found in the first superblock (SB 0) respectively.\n"
"As a side effect of writing the UUID, the log is cleared (which is fine\n"
"on a CLEANLY unmounted FS).\n"
"\n"
);
}

static uuid_t *
do_uuid(xfs_agnumber_t agno, uuid_t *uuid)
{
	xfs_sb_t	tsb;
	static uuid_t	uu;

	if (!get_sb(agno, &tsb))
		return NULL;

	if (!uuid) {	/* get uuid */
		memcpy(&uu, &tsb.sb_uuid, sizeof(uuid_t));
		pop_cur();
		return &uu;
	}
	/* set uuid */
	memcpy(&tsb.sb_uuid, uuid, sizeof(uuid_t));
	libxfs_xlate_sb(iocur_top->data, &tsb, -1, ARCH_CONVERT, XFS_SB_UUID);
	write_cur();
	return uuid;
}

static int
uuid_f(
	int		argc,
	char		**argv)
{
	char	        bp[40];
	xfs_agnumber_t	agno;
	uuid_t		uu;
	uuid_t		*uup = NULL;

	if (argc != 1 && argc != 2) {
		dbprintf("invalid parameters\n");
		return 0;
	}

	if (argc == 2) {	/* WRITE UUID */

		if ((x.isreadonly & LIBXFS_ISREADONLY) || !expert_mode) {
			dbprintf("%s: not in expert mode, writing disabled\n",
				progname);
			return 0;
		}

		if (!strcasecmp(argv[1], "generate")) {
			uuid_generate(uu);
		} else if (!strcasecmp(argv[1], "nil")) {
			uuid_clear(uu);
		} else if (!strcasecmp(argv[1], "rewrite")) {
			uup = do_uuid(0, NULL);
			if (!uup) {
				dbprintf("failed to read UUID from AG 0\n");
				return 0;
			}
			memcpy(&uu, uup, sizeof(uuid_t));
			uuid_unparse(uu, bp);
			dbprintf("old UUID = %s\n", bp);
		} else {
			if (uuid_parse(argv[1], uu)) {
				dbprintf("invalid UUID\n");
				return 0;
			}
		}

		/* clear the log (setting uuid) if its not dirty */
		if (!zero_log(&uu))
			return 0;

		dbprintf("writing all SBs\n");
		for (agno = 0; agno < mp->m_sb.sb_agcount; agno++)
			if (!do_uuid(agno, &uu)) {
				dbprintf("failed to set UUID in AG %d\n", agno);
				break;
			}

		uuid_unparse(uu, bp);
		dbprintf("new UUID = %s\n", bp);
		return 0;

	} else {	/* READ+CHECK UUID */

		for (agno = 0; agno < mp->m_sb.sb_agcount; agno++) {
			uup = do_uuid(agno, NULL);
			if (!uup) {
				dbprintf("failed to read UUID from AG %d\n",
					agno);
				return 0;
			}
			if (agno) {
				if (memcmp(&uu, uup, sizeof(uuid_t))) {
					dbprintf("warning: UUID in AG %d "
						 "differs to the primary SB\n",
						agno);
					break;
				}
			} else {
				memcpy(&uu, uup, sizeof(uuid_t));
			}
		}
		if (mp->m_sb.sb_logstart) {
			if (x.logdev && x.logdev != x.ddev)
				dbprintf("warning - external log specified "
					 "for FS with an internal log\n");
		} else if (!x.logdev || (x.logdev == x.ddev)) {
			dbprintf("warning - no external log specified "
				 "for FS with an external log\n");
		}

		uuid_unparse(uu, bp);
		dbprintf("UUID = %s\n", bp);
	}

	return 0;
}


static void
label_help(void)
{
	dbprintf(
"\n"
" write/print FS label\n"
"\n"
" Example:\n"
"\n"
" 'label'              - print label\n"
" 'label 123456789012' - write label\n"
" 'label --'           - write an empty label\n"
"\n"
"The print function checks the label in each SB and will warn if the labels\n"
"differ between AGs. The write commands will set the label in all AGs to the\n"
"specified value.  The maximum length of a label is 12 characters - use of a\n"
"longer label will result in truncation and a warning will be issued.\n"
"\n"
);
}

static char *
do_label(xfs_agnumber_t agno, char *label)
{
	size_t		len;
	xfs_sb_t	tsb;
	static char	lbl[sizeof(tsb.sb_fname) + 1];

	if (!get_sb(agno, &tsb))
		return NULL;

	memset(&lbl[0], 0, sizeof(lbl));

	if (!label) {	/* get label */
		pop_cur();
		memcpy(&lbl[0], &tsb.sb_fname, sizeof(tsb.sb_fname));
		return &lbl[0];
	}
	/* set label */
	if ((len = strlen(label)) > sizeof(tsb.sb_fname)) {
		if (agno == 0)
			dbprintf("%s: truncating label length from %d to %d\n",
				progname, (int)len, (int)sizeof(tsb.sb_fname));
		len = sizeof(tsb.sb_fname);
	}
	if ( len == 2 &&
	     (strcmp(label, "\"\"") == 0 ||
	      strcmp(label, "''")   == 0 ||
	      strcmp(label, "--")   == 0) )
		label[0] = label[1] = '\0';
	memset(&tsb.sb_fname, 0, sizeof(tsb.sb_fname));
	memcpy(&tsb.sb_fname, label, len);
	memcpy(&lbl[0], &tsb.sb_fname, sizeof(tsb.sb_fname));
	libxfs_xlate_sb(iocur_top->data, &tsb, -1, ARCH_CONVERT, XFS_SB_FNAME);
	write_cur();
	return &lbl[0];
}

static int
label_f(
	int		argc,
	char		**argv)
{
	char		*p = NULL;
	xfs_sb_t	sb;
	xfs_agnumber_t	ag;

	if (argc != 1 && argc != 2) {
		dbprintf("invalid parameters\n");
		return 0;
	}

	if (argc == 2) {	/* WRITE LABEL */

		if ((x.isreadonly & LIBXFS_ISREADONLY) || !expert_mode) {
			dbprintf("%s: not in expert mode, writing disabled\n",
				progname);
			return 0;
		}

		dbprintf("writing all SBs\n");
		for (ag = 0; ag < mp->m_sb.sb_agcount; ag++)
			if ((p = do_label(ag, argv[1])) == NULL) {
				dbprintf("failed to set label in AG %d\n", ag);
				break;
			}
		dbprintf("new label = \"%s\"\n", p);

	} else {	/* READ LABEL */

		for (ag = 0; ag < mp->m_sb.sb_agcount; ag++) {
			p = do_label(ag, NULL);
			if (!p) {
				dbprintf("failed to read label in AG %d\n", ag);
				return 0;
			}
			if (!ag)
				memcpy(&sb.sb_fname, p, sizeof(sb.sb_fname));
			else if (memcmp(&sb.sb_fname, p, sizeof(sb.sb_fname)))
				dbprintf("warning: AG %d label differs\n", ag);
		}
		dbprintf("label = \"%s\"\n", p);
	}
	return 0;
}


static void
version_help(void)
{
	dbprintf(
"\n"
" set/print feature bits in sb version\n"
"\n"
" Example:\n"
"\n"
" 'version'          - print current feature bits\n"
" 'version extflg'   - enable unwritten extents\n"
"\n"
"The version function prints currently enabled features for a filesystem\n"
"according to its the version field of the primary superblock.\n"
"It can also be used to enable selected features, such as support for\n"
"unwritten extents.  The upated version is written into to all AGs.\n"
"\n"
);
}

static int
do_version(xfs_agnumber_t agno, __uint16_t versionnum)
{
	xfs_sb_t	tsb;

	if (!get_sb(agno, &tsb))
		return 0;

	tsb.sb_versionnum = versionnum;
	libxfs_xlate_sb(iocur_top->data, &tsb,
			-1, ARCH_CONVERT, XFS_SB_VERSIONNUM);
	write_cur();
	return 1;
}

static char *
version_string(
	xfs_sb_t	*sbp)
{
	static char	s[1024];

	if (XFS_SB_VERSION_NUM(sbp) == XFS_SB_VERSION_1)
		strcpy(s, "V1");
	else if (XFS_SB_VERSION_NUM(sbp) == XFS_SB_VERSION_2)
		strcpy(s, "V2");
	else if (XFS_SB_VERSION_NUM(sbp) == XFS_SB_VERSION_3)
		strcpy(s, "V3");
	else if (XFS_SB_VERSION_NUM(sbp) == XFS_SB_VERSION_4)
		strcpy(s, "V4");

	if (XFS_SB_VERSION_HASATTR(sbp))
		strcat(s, ",ATTR");
	if (XFS_SB_VERSION_HASNLINK(sbp))
		strcat(s, ",NLINK");
	if (XFS_SB_VERSION_HASQUOTA(sbp))
		strcat(s, ",QUOTA");
	if (XFS_SB_VERSION_HASALIGN(sbp))
		strcat(s, ",ALIGN");
	if (XFS_SB_VERSION_HASDALIGN(sbp))
		strcat(s, ",DALIGN");
	if (XFS_SB_VERSION_HASSHARED(sbp))
		strcat(s, ",SHARED");
	if (XFS_SB_VERSION_HASDIRV2(sbp))
		strcat(s, ",DIRV2");
	if (XFS_SB_VERSION_HASLOGV2(sbp))
		strcat(s, ",LOGV2");
	if (XFS_SB_VERSION_HASEXTFLGBIT(sbp))
		strcat(s, ",EXTFLG");
	if (XFS_SB_VERSION_HASSECTOR(sbp))
		strcat(s, ",SECTOR");
	return s;
}

static int
version_f(
	int		argc,
	char		**argv)
{
	__uint16_t	version = 0;
	xfs_agnumber_t	ag;

	if (argc == 2) {	/* WRITE VERSION */

		if ((x.isreadonly & LIBXFS_ISREADONLY) || !expert_mode) {
			dbprintf("%s: not in expert mode, writing disabled\n",
				progname);
			return 0;
		}

		/* Logic here derived from the IRIX xfs_chver(1M) script. */
		if (!strcasecmp(argv[1], "extflg")) {
			switch (XFS_SB_VERSION_NUM(&mp->m_sb)) {
			case XFS_SB_VERSION_1:
				version = 0x0004 | XFS_SB_VERSION_EXTFLGBIT;
				break;
			case XFS_SB_VERSION_2:
				version = 0x0014 | XFS_SB_VERSION_EXTFLGBIT;
				break;
			case XFS_SB_VERSION_3:
				version = 0x0034 | XFS_SB_VERSION_EXTFLGBIT;
				break;
			case XFS_SB_VERSION_4:
				if (XFS_SB_VERSION_HASEXTFLGBIT(&mp->m_sb))
					dbprintf("unwritten extents flag"
						 " is already enabled\n");
				else
					version = mp->m_sb.sb_versionnum |
						  XFS_SB_VERSION_EXTFLGBIT;
				break;
			}
		} else {
			dbprintf("%s: invalid version change command \"%s\"\n",
				progname, argv[1]);
			return 0;
		}

		if (version) {
			dbprintf("writing all SBs\n");
			for (ag = 0; ag < mp->m_sb.sb_agcount; ag++)
				if (!do_version(ag, version)) {
					dbprintf("failed to set versionnum "
						 "in AG %d\n", ag);
					break;
				}
			mp->m_sb.sb_versionnum = version;
		}
	}
	dbprintf("versionnum [0x%x] = %s\n", mp->m_sb.sb_versionnum,
			version_string(&mp->m_sb));
	return 0;
}
