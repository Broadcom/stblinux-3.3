/*
 SYSVish version:

Copyright (C) 1996, 2000 Free Software Foundation, Inc.
This file is part of the GNU C Library.
Contributed by Ulrich Drepper <drepper@cygnus.com>, 1996.

The GNU C Library is free software; you can redistribute it and/or
modify it under the terms of the GNU Library General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The GNU C Library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Library General Public License for more details.

You should have received a copy of the GNU Library General Public
License along with the GNU C Library; see the file COPYING.LIB.  If
not, write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.

 * BSDish version:
 *
 * Copyright (c) 1988, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#ifndef lint
static char sccsid[] = "@(#)logwtmp.c	8.1 (Berkeley) 6/4/93";
#endif /* not lint */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <utmp.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "extern.h"

#if !defined (_PATH_WTMP) && defined (WTMP_FILE)
#define _PATH_WTMP WTMP_FILE
#endif

/*
 * Modified version of logwtmp that holds wtmp file open
 * after first call, for use with ftp (which may chroot
 * after login, but before logout).
 */
void
logwtmp(const char *line, const char *name, const char *host)
{
  struct utmp ut;

  /* Set information in new entry.  */
  memset (&ut, 0, sizeof (ut));
#if _HAVE_UT_TYPE - 0
  ut.ut_type = USER_PROCESS;
#endif
  strncpy (ut.ut_line, line, sizeof ut.ut_line);
  strncpy (ut.ut_name, name, sizeof ut.ut_name);
#if _HAVE_UT_HOST - 0
  strncpy (ut.ut_host, host, sizeof ut.ut_host);
#endif

#if _HAVE_UT_TV - 0
  gettimeofday (&ut.ut_tv, NULL);
#else
  time (&ut.ut_time);
#endif

#if defined (HAVE_UTMPNAME) && defined (HAVE_SETUTENT_R)
  /* XXX I think frobbing the details of DATA is GNU libc specific.  */
  {
    static struct utmp_data data = { -1 };

    if (data.ut_fd < 0)
      /* Open the file.  */
      {
	/* Tell that we want to use the WTMP file.  */
	if (utmpname (_PATH_WTMP) == 0)
	  return;

	/* Open WTMP file.  */
	setutent_r (&data);
      }

    /* Position at end of file.  */
    data.loc_utmp = lseek (data.ut_fd, 0, SEEK_END);
    if (data.loc_utmp == -1)
      return;

    /* Write the entry.  */
    pututline_r (&ut, &data);
  }
#else
  /* Do things the old way.  */
  {
	struct utmp ut;
	struct stat buf;
	static int fd = -1;

	if (fd < 0 && (fd = open(_PATH_WTMP, O_WRONLY|O_APPEND, 0)) < 0)
		return;
	if (fstat(fd, &buf) == 0) {
		if (write(fd, (char *)&ut, sizeof(struct utmp)) !=
		    sizeof(struct utmp))
			(void)ftruncate(fd, buf.st_size);
	}
  }
#endif
}
