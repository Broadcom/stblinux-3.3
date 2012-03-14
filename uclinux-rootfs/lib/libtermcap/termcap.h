/* Declarations for termcap library.
   Copyright (C) 1991, 1992 Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.  */

#ifndef _TERMCAP_H
#define _TERMCAP_H 1

#include <features.h>
#include <termios.h>
#include <sys/types.h>

extern char PC;
extern char *UP;
extern char *BC;

#ifdef __linux__
extern speed_t ospeed;
#else
extern short ospeed;
#endif

__BEGIN_DECLS

#if defined(__PMT)	/* XXX glibc-2.1 wants this */
typedef int (*outfuntype) __PMT((int));
#else
typedef int (*outfuntype) __P((int));
#endif

extern int tgetent __P((void *__buffer, __const char *__termtype));
extern int tgetflag __P((__const char *__name));
extern int tgetnum __P((__const char *__name));
extern char *tgetstr __P((__const char *__name, char **__area));
extern char *tgoto __P((__const char *__cstring, int __hpos,
	int __vpos));
extern char *tparam __P((__const char *__ctlstring, void *__buffer,
	int __size, ...));

#if defined(__PMT)	/* XXX glibc-2.1 wants this */
extern void tputs __P((__const char *__string, int __nlines,
	int (*) __PMT((int))));
#else
extern void tputs __P((__const char *__string, int __nlines,
	int (*) __P((int))));
#endif

__END_DECLS

#endif /* not _TERMCAP_H */
