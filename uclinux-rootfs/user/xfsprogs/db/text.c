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
#include <ctype.h>
#include "block.h"
#include "bmap.h"
#include "command.h"
#include "type.h"
#include "faddr.h"
#include "fprint.h"
#include "field.h"
#include "inode.h"
#include "io.h"
#include "output.h"
#include "init.h"

static void     print_rawtext(void *data, int len);

void
print_text(
	const field_t   *fields,
	int             argc,
	char            **argv)
{
	print_rawtext(iocur_top->data, iocur_top->len);
}

static void
print_rawtext(
	void    *data,
	int     len)
{
	int     i;
	int     j;
	int     lastaddr;
	int     offchars;
	unsigned char   *p;

	lastaddr = (len - 1) & ~(16 - 1);
	if (lastaddr < 0x10)
		offchars = 1;
	else if (lastaddr < 0x100)
		offchars = 2;
	else if (lastaddr < 0x1000)
		offchars = 3;
	else
		offchars = 4;

	for (i = 0, p = data; i < len; i += 16) {
		unsigned char *s = p;

		dbprintf("%-0*.*x:  ", offchars, offchars, i);

		for (j = 0; j < 16 && i + j < len; j++, p++) {
			dbprintf("%02x ", *p);
		}

		dbprintf(" ");

		for (j = 0; j < 16 && i + j < len; j++, s++) {
			if (isalnum(*s))
				dbprintf("%c", *s);
			else
				dbprintf(".", *s);
		}

		dbprintf("\n");
	}
}
