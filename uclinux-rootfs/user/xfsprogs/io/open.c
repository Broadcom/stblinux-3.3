/*
 * Copyright (c) 2003 Silicon Graphics, Inc.  All Rights Reserved.
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
#include "command.h"
#include "input.h"
#include "init.h"

static cmdinfo_t open_cmd;
static cmdinfo_t stat_cmd;
static cmdinfo_t setfl_cmd;
static cmdinfo_t statfs_cmd;
static cmdinfo_t extsize_cmd;
static int stat_f(int, char **);

int
openfile(
	char		*path,
	xfs_fsop_geom_t	*geom,
	int		aflag,
	int		cflag,
	int		dflag,
	int		rflag,
	int		sflag,
	int		tflag,
	int		xflag)
{
	int		fd;
	int		oflags;

	oflags = (rflag ? O_RDONLY : O_RDWR);
	if (aflag)
		oflags |= O_APPEND;
	if (cflag)
		oflags |= O_CREAT;
	if (dflag)
		oflags |= O_DIRECT;
	if (sflag)
		oflags |= O_SYNC;
	if (tflag)
		oflags |= O_TRUNC;

	fd = open(path, oflags, 0644);
	if (fd < 0) {
		perror(path);
		return -1;
	}
	if (!geom)
		return fd;

	if (!platform_test_xfs_fd(fd)) {
		fprintf(stderr, _("%s: specified file "
			"[\"%s\"] is not on an XFS filesystem\n"),
			progname, fname);
		close(fd);
		return -1;
	}

	if (xfsctl(path, fd, XFS_IOC_FSGEOMETRY, geom) < 0) {
		perror("XFS_IOC_FSGEOMETRY");
		close(fd);
		return -1;
	}

	if (!readonly && xflag) {	/* read/write and realtime */
		struct fsxattr	attr;

		if (xfsctl(path, fd, XFS_IOC_FSGETXATTR, &attr) < 0) {
			perror("XFS_IOC_FSGETXATTR");
			close(fd);
			return -1;
		}
		if (!(attr.fsx_xflags & XFS_XFLAG_REALTIME)) {
			attr.fsx_xflags |= XFS_XFLAG_REALTIME;
			if (xfsctl(path, fd, XFS_IOC_FSSETXATTR, &attr) < 0) {
				perror("XFS_IOC_FSSETXATTR");
				close(fd);
				return -1;
			}
		}
	}
	return fd;
}

static int
usage(void)
{
	printf("%s %s\n", open_cmd.name, open_cmd.oneline);
	return 0;
}

static void
open_help(void)
{
	printf(_(
"\n"
" opens a new file in the requested mode, after closing the current file\n"
"\n"
" Example:\n"
" 'open -d /tmp/data' - opens data file read-write for direct IO\n"
"\n"
" Opens a file for subsequent use by all of the other xfs_io commands.\n"
" With no arguments, open uses the stat command to show the current file.\n"
" -a -- open with the O_APPEND flag (append-only mode)\n"
" -c -- open with O_CREAT (create the file if it doesn't exist)\n"
" -d -- open with O_DIRECT (non-buffered IO, note alignment constraints)\n"
" -r -- open with O_RDONLY, the default is O_RDWR\n"
" -s -- open with O_SYNC\n"
" -t -- open with O_TRUNC (truncate the file to zero length if it exists)\n"
" -x -- mark the file as a realtime XFS file immediately after opening it\n"
" Note1: read/write direct IO requests must be blocksize aligned.\n"
" Note2: the bmap for non-regular files can be obtained provided the file\n"
" was opened appropriately (in particular, must be opened read-only).\n"
"\n"));
}

static int
open_f(
	int		argc,
	char		**argv)
{
	int		aflag = 0;
	int		cflag = 0;
	int		dflag = 0;
	int		rflag = 0;
	int		sflag = 0;
	int		tflag = 0;
	int		xflag = 0;
	char		*filename;
	xfs_fsop_geom_t	geometry;
	int		fd;
	int		c;

	if (argc == 1)
		return stat_f(argc, argv);

	while ((c = getopt(argc, argv, "acdrstx")) != EOF) {
		switch (c) {
		case 'a':
			aflag = 1;
			break;
		case 'c':
			cflag = 1;
			break;
		case 'd':
			dflag = 1;
			break;
		case 'r':
			rflag = 1;
			break;
		case 's':
			sflag = 1;
			break;
		case 't':
			tflag = 1;
			break;
		case 'x':
			xflag = 1;
			break;
		default:
			return usage();
		}
	}

	if (optind != argc - 1)
		return usage();

	fd = openfile(argv[optind], &geometry,
		      aflag, cflag, dflag, rflag, sflag, tflag, xflag);
	if (fd < 0)
		return 0;

	filename = strdup(argv[optind]);
	if (!filename) {
		perror("strdup");
		close(fd);
		return 0;
	}

	/*
	 * All OK, proceed to make this the new global open file
	 */
	osync = sflag;
	trunc = tflag;
	append = aflag;
	directio = dflag;
	readonly = rflag;
	realtime = xflag;
	if (fname) {
		close(fdesc);
		free(fname);
	}
	fgeom = geometry;
	fname = filename;
	fdesc = fd;
	return 0;
}

off64_t
filesize(void)
{
	struct stat64	st;

	if (fstat64(fdesc, &st) < 0) {
		perror("fstat64");
		return -1;
	}
	return st.st_size;
}

static char *
filetype(mode_t mode)
{
	switch (mode & S_IFMT) {
	case S_IFSOCK:
		return _("socket");
	case S_IFDIR:
		return _("directory");
	case S_IFCHR:
		return _("char device");
	case S_IFBLK:
		return _("block device");
	case S_IFREG:
		return _("regular file");
	case S_IFLNK:
		return _("symbolic link");
	case S_IFIFO:
		return _("fifo");
	}
	return NULL;
}

static int
stat_f(
	int		argc,
	char		**argv)
{
	struct fsxattr	fsx;
	struct stat64	st;
	char		fullname[PATH_MAX + 1];

	printf(_("fd.path = \"%s\"\n"),
		realpath(fname, fullname) ? fullname : fname);
	printf(_("fd.flags = %s,%s,%s%s%s\n"),
		osync ? _("sync") : _("non-sync"),
		directio ? _("direct") : _("non-direct"),
		readonly ? _("read-only") : _("read-write"),
		realtime ? _(",real-time") : "",
		append ? _(",append-only") : "");
	if (fstat64(fdesc, &st) < 0) {
		perror("fstat64");
	} else {
		printf(_("stat.ino = %lld\n"), (long long)st.st_ino);
		printf(_("stat.type = %s\n"), filetype(st.st_mode));
		printf(_("stat.size = %lld\n"), (long long)st.st_size);
		printf(_("stat.blocks = %lld\n"), (long long)st.st_blocks);
		if (argc == 2 && !strcmp(argv[1], "-v")) {
			printf(_("stat.atime = %s"), ctime(&st.st_atime));
			printf(_("stat.mtime = %s"), ctime(&st.st_mtime));
			printf(_("stat.ctime = %s"), ctime(&st.st_ctime));
		}
	}
	if ((xfsctl(fname, fdesc, XFS_IOC_FSGETXATTR, &fsx)) < 0) {
		perror("xfsctl(XFS_IOC_FSGETXATTR)");
	} else {
		printf(_("xattr.xflags = 0x%x\n"), fsx.fsx_xflags);
		printf(_("xattr.extsize = %u\n"), fsx.fsx_extsize);
		printf(_("xattr.nextents = %u\n"), fsx.fsx_nextents);
	}
	return 0;
}

static int
setfl_f(
	int			argc,
	char			**argv)
{
	int			c, flags;

	flags = fcntl(fdesc, F_GETFL, 0);
	if (flags < 0) {
		perror("fcntl(F_GETFL)");
		return 0;
	}

	while ((c = getopt(argc, argv, "ad")) != EOF) {
		switch (c) {
		case 'a':
			if (flags & O_APPEND)
				flags |= O_APPEND;
			else
				flags &= ~O_APPEND;
			break;
		case 'd':
			if (flags & O_DIRECT)
				flags |= O_DIRECT;
			else
				flags &= ~O_DIRECT;
			break;
		default:
			printf(_("invalid setfl argument -- '%c'\n"), c);
			return 0;
		}
	}

	if (fcntl(fdesc, F_SETFL, flags)  < 0)
		perror("fcntl(F_SETFL)");

	return 0;
}

static int
extsize_f(
	int			argc,
	char			**argv)
{
	struct fsxattr		fsx;
	long			extsize;

	extsize = (long)cvtnum(fgeom.blocksize, fgeom.sectsize, argv[1]);
	if (extsize < 0) {
		printf(_("non-numeric extsize argument -- %s\n"), argv[1]);
		return 0;
	}
	if ((xfsctl(fname, fdesc, XFS_IOC_FSGETXATTR, &fsx)) < 0) {
		perror("xfsctl(XFS_IOC_FSGETXATTR)");
		return 0;
	}
	fsx.fsx_extsize = extsize;
	if ((xfsctl(fname, fdesc, XFS_IOC_FSSETXATTR, &fsx)) < 0) {
		perror("xfsctl(XFS_IOC_FSSETXATTR)");
		return 0;
	}

	return 0;
}

static int
statfs_f(
	int			argc,
	char			**argv)
{
	struct xfs_fsop_geom_v1	fsgeo;
	struct statfs		st;
	char			fullname[PATH_MAX + 1];

	printf(_("fd.path = \"%s\"\n"),
		realpath(fname, fullname) ? fullname : fname);
	if (platform_fstatfs(fdesc, &st) < 0) {
		perror("fstatfs");
	} else {
		printf(_("statfs.f_bsize = %lld\n"), (long long) st.f_bsize);
		printf(_("statfs.f_blocks = %lld\n"), (long long) st.f_blocks);
#if !defined(__sgi__)
		printf(_("statfs.f_bavail = %lld\n"), (long long) st.f_bavail);
#endif
	}
	if ((xfsctl(fname, fdesc, XFS_IOC_FSGEOMETRY_V1, &fsgeo)) < 0) {
		perror("xfsctl(XFS_IOC_FSGEOMETRY_V1)");
	} else {
		printf(_("geom.bsize = %u\n"), fsgeo.blocksize);
		printf(_("geom.agcount = %u\n"), fsgeo.agcount);
		printf(_("geom.agblocks = %u\n"), fsgeo.agblocks);
		printf(_("geom.datablocks = %llu\n"),
			(unsigned long long) fsgeo.datablocks);
		printf(_("geom.rtblocks = %llu\n"),
			(unsigned long long) fsgeo.rtblocks);
		printf(_("geom.rtextents = %llu\n"),
			(unsigned long long) fsgeo.rtextents);
		printf(_("geom.rtextsize = %u\n"), fsgeo.rtextsize);
		printf(_("geom.sunit = %u\n"), fsgeo.sunit);
		printf(_("geom.swidth = %u\n"), fsgeo.swidth);
	}
	return 0;
}

void
open_init(void)
{
	open_cmd.name = _("open");
	open_cmd.altname = _("o");
	open_cmd.cfunc = open_f;
	open_cmd.argmin = 0;
	open_cmd.argmax = -1;
	open_cmd.args = _("[-acdrstx] [path]");
	open_cmd.oneline =
		_("close the current file, open file specified by path");
	open_cmd.help = open_help;

	stat_cmd.name = _("stat");
	stat_cmd.cfunc = stat_f;
	stat_cmd.argmin = 0;
	stat_cmd.argmax = 1;
	stat_cmd.args = _("[-v]");
	stat_cmd.oneline =
		_("statistics on the currently open file");

	setfl_cmd.name = _("setfl");
	setfl_cmd.cfunc = setfl_f;
	setfl_cmd.args = _("[-adx]");
	setfl_cmd.oneline =
		_("set/clear append/direct flags on the open file");

	statfs_cmd.name = _("statfs");
	statfs_cmd.cfunc = statfs_f;
	statfs_cmd.oneline =
		_("statistics on the filesystem of the currently open file");

	extsize_cmd.name = _("extsize");
	extsize_cmd.cfunc = extsize_f;
	extsize_cmd.argmin = 1;
	extsize_cmd.argmax = 1;
	extsize_cmd.oneline =
		_("set prefered extent size (in bytes) for the open file");

	add_command(&open_cmd);
	add_command(&stat_cmd);
	add_command(&setfl_cmd);
	add_command(&statfs_cmd);
	add_command(&extsize_cmd);
}
