/*-
 * Copyright (c) 1991, 1993
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
 */

#ifndef lint
static char sccsid[] = "@(#)getent.c	8.2 (Berkeley) 12/15/93";
#endif /* not lint */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifdef HAVE_CGETENT
static char *area;
#endif

/*ARGSUSED*/
int
getent(char *cp, char *name)
{
#ifdef	HAVE_CGETENT
	char *dba[2];
#endif
	(void)cp; (void)name; /* shutup gcc */
#ifdef	HAVE_CGETENT
	dba[0] = "/etc/gettytab";
	dba[1] = 0;
	return((cgetent(&area, dba, name) == 0) ? 1 : 0);
#else
	return(0);
#endif
}

#ifndef	SOLARIS
/*ARGSUSED*/
char *
getstr(char *id, char **cpp)
{
# ifdef	HAVE_CGETENT
	char *answer;
# endif
	(void)id; (void)cpp;
# ifdef	HAVE_CGETENT
	return((cgetstr(area, id, &answer) > 0) ? answer : 0);
# else
	return(0);
# endif
}
#endif
