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
#include <ctype.h>
#include "command.h"
#include "input.h"
#include "init.h"

static cmdinfo_t pread_cmd;

static void
pread_help(void)
{
	printf(_(
"\n"
" reads a range of bytes in a specified block size from the given offset\n"
"\n"
" Example:\n"
" 'read -v 512 20' - dumps 20 bytes read from 512 bytes into the file\n"
"\n"
" Reads a segment of the currently open file, optionally dumping it to the\n"
" standard output stream (with -v option) for subsequent inspection.\n"
" The reads are performed in sequential blocks starting at offset, with the\n"
" blocksize tunable using the -b option (default blocksize is 4096 bytes).\n"
"\n"));
}

void	*buffer;
ssize_t	buffersize;

int
alloc_buffer(
	ssize_t		bsize,
	unsigned int	seed)
{
	if (bsize > buffersize) {
		if (buffer)
			free(buffer);
		buffer = memalign(getpagesize(), buffersize = bsize);
		if (!buffer) {
			perror("memalign");
			buffersize = 0;
			return 0;
		}
	}
	memset(buffer, seed, buffersize);
	return 1;
}

static void
dump_buffer(
	off64_t		offset,
	ssize_t		len)
{
	int		i, j;
	char		*p;

	for (i = 0, p = (char *)buffer; i < len; i += 16) {
		char	*s = p;

		printf("%08llx:  ", (unsigned long long)offset + i);
		for (j = 0; j < 16 && i + j < len; j++, p++)
			printf("%02x ", *p);
		printf(" ");
		for (j = 0; j < 16 && i + j < len; j++, s++) {
			if (isalnum((int)*s))
				printf("%c", *s);
			else
				printf(".");
		}
		printf("\n");
	}
}

int
read_buffer(
	int		fd,
	off64_t		offset,
	ssize_t		count,
	ssize_t		*total,
	int		verbose,
	int		onlyone)
{
	ssize_t		bytes;

	*total = 0;
	while (count > 0) {
		bytes = pread64(fd, buffer, min(count,buffersize), offset);
		if (bytes == 0)
			break;
		if (bytes < 0) {
			perror("pread64");
			return 0;
		}
		if (verbose)
			dump_buffer(offset, bytes);
		*total += bytes;
		if (onlyone || bytes < count)
			break;
		offset += bytes;
		count -= bytes;
	}
	return 1;
}

static int
pread_f(
	int		argc,
	char		**argv)
{
	off64_t		offset;
	ssize_t		count, total;
	unsigned int	bsize = 4096;
	char		*sp;
	int		vflag = 0;
	int		c;

	while ((c = getopt(argc, argv, "b:v")) != EOF) {
		switch (c) {
		case 'b':
			bsize = strtoul(optarg, &sp, 0);
			if (!sp || sp == optarg) {
				printf(_("non-numeric bsize -- %s\n"), optarg);
				return 0;
			}
			break;
		case 'v':
			vflag = 1;
			break;
		default:
			printf("%s %s\n", pread_cmd.name, pread_cmd.oneline);
			return 0;
		}
	}
	if (optind != argc - 2) {
		printf("%s %s\n", pread_cmd.name, pread_cmd.oneline);
		return 0;
	}
	offset = cvtnum(fgeom.blocksize, fgeom.sectsize, argv[optind]);
	if (offset < 0) {
		printf(_("non-numeric offset argument -- %s\n"), argv[optind]);
		return 0;
	}
	optind++;
	count = (ssize_t)cvtnum(fgeom.blocksize, fgeom.sectsize, argv[optind]);
	if (count < 0) {
		printf(_("non-numeric length argument -- %s\n"), argv[optind]);
		return 0;
	}

	if (!alloc_buffer(bsize, 0xabababab))
		return 0;

	if (!read_buffer(fdesc, offset, count, &total, vflag, 0))
		return 0;

	printf(_("read %ld/%ld bytes at offset %lld\n"),
		(long)total, (long)count, (long long)offset);
	return 0;
}

void
pread_init(void)
{
	pread_cmd.name = _("pread");
	pread_cmd.altname = _("r");
	pread_cmd.cfunc = pread_f;
	pread_cmd.argmin = 2;
	pread_cmd.argmax = -1;
	pread_cmd.args = _("[-b bs] [-v] off len");
	pread_cmd.oneline = _("reads a number of bytes at a specified offset");
	pread_cmd.help = pread_help;

	add_command(&pread_cmd);
}
