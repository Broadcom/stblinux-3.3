/*
 *  Copyright (c) 2003 Silicon Graphics, Inc.  All Rights Reserved.
 *
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
 * or the like.	 Any license provided herein, whether implied or
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

#define ustat __kernel_ustat
#include <xfs/libxfs.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/diskslice.h>
#include <sys/disklabel.h>

extern char *progname;

int
platform_check_ismounted(char *name, char *block, struct stat64 *s, int verbose)
{
	struct stat	st;

	if (!s) {
		if (stat(block, &st) < 0)
			return 0;
		if ((st.st_mode & S_IFMT) != S_IFBLK)
			return 0;
		s = &st;
	}
	/* Remember, FreeBSD can now mount char devices! -- adrian */
	if (((st.st_mode & S_IFMT) != S_IFBLK) &&
	    ((st.st_mode & S_IFMT) != S_IFCHR))
		return 0;

	return 0;
}

int
platform_check_iswritable(char *name, char *block, struct stat64 *s, int fatal)
{
	return 0;
}

void
platform_set_blocksize(int fd, char *path, int blocksize)
{
}

void
platform_flush_device(int fd)
{
}

/*
 * Get disk slice and partition information.
 */
static __int64_t
getdisksize(int fd, const char *fname)
{
	//struct	diskslices ds;
	struct	disklabel dl, *lp;
	const	char *s1, *s2;
	char	*s;
	int	slice, part, fd1, i, e;
	__int64_t size = 0LL;

	slice = part = -1;
	s1 = fname;
	if ((s2 = strrchr(s1, '/')))
		s1 = s2 + 1;
	for (s2 = s1; *s2 && !isdigit(*s2); s2++);
	if (!*s2 || s2 == s1)
		s2 = NULL;
	else
		while (isdigit(*++s2));
	s1 = s2;
	if (s2 && *s2 == 's') {
		slice = strtol(s2 + 1, &s, 10);
		if (slice < 1 || slice > MAX_SLICES - BASE_SLICE)
			s2 = NULL;
		else {
			slice = BASE_SLICE + slice - 1;
			s2 = s;
		}
	}
	if (s2 && *s2 >= 'a' && *s2 <= 'a' + MAXPARTITIONS - 1) {
		if (slice == -1)
			slice = COMPATIBILITY_SLICE;
		part = *s2++ - 'a';
	}
	if (!s2 || (*s2 && *s2 != '.')) {
		fprintf(stderr, _("%s: can't figure out partition info\n"),
		    progname);
		exit(1);
	}

	if (slice == -1 || part != -1) {
		lp = &dl;
		i = ioctl(fd, DIOCGDINFO, lp);
		if (i == -1 && slice != -1 && part == -1) {
			e = errno;
			if (!(s = strdup(fname))) {
				fprintf(stderr, "%s: %s\n",
				    progname, strerror(errno));
				exit(1);
			}
			s[s1 - fname] = 0;
			if ((fd1 = open(s, O_RDONLY)) != -1) {
				i = ioctl(fd1, DIOCGDINFO, lp);
				close(fd1);
			}
			free(s);
			errno = e;
		}
		if (i == -1) {
			fprintf(stderr, _("%s: can't read disk label: %s\n"),
			    progname, strerror(errno));
			exit(1);
		}
		if (slice == -1 || part != -1) {
			if (part == -1)
				part = RAW_PART;
			if (part >= lp->d_npartitions ||
			    !lp->d_partitions[part].p_size) {
				fprintf(stderr,
					_("%s: partition %s is unavailable\n"),
					progname, fname);
				exit(1);
			}
			size = lp->d_partitions[part].p_size;
		}
	}
	return size;
}

void
platform_findsizes(char *path, int fd, long long *sz, int *bsz)
{
	struct stat	st;
	__int64_t	size;

	if (fstat(fd, &st) < 0) {
		fprintf(stderr, _("%s: "
			"cannot stat the device file \"%s\": %s\n"),
			progname, path, strerror(errno));
		exit(1);
	}
	if ((st.st_mode & S_IFMT) == S_IFREG) {
		*sz = (long long)(st.st_size >> 9);
		*bsz = BBSIZE;
		return;
	}

	*sz = (long long) getdisksize(fd, path);
	*bsz = BBSIZE;
}
