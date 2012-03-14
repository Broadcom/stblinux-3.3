/* Copyright (C) 1998,2001 Free Software Foundation, Inc.

   This file is part of GNU Inetutils.

   GNU Inetutils is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   GNU Inetutils is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GNU Inetutils; see the file COPYING.  If not, write to
   the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA. */

#include <intalkd.h>
#include <stdarg.h>
#include <sys/uio.h>

#undef MAX
#define MAX(a,b) ( (a) > (b) ? (a) : (b) )
#define N_LINES 5
#define N_CHARS 256

extern char * ttymsg (struct iovec *iov, int iovcnt, char *line, int tmout);

typedef struct
{
  int ind;
  int max_size;
  char line[N_LINES][N_CHARS];
  int size[N_LINES];
  char buf[N_LINES*N_CHARS+3];
} LINE;

static void
init_line (LINE *lp)
{
  memset (lp, 0, sizeof *lp);
}

static void
format_line (LINE *lp, const char *fmt, ...)
{
  va_list ap;
  int i = lp->ind;

  if (lp->ind >= N_LINES)
    return;
  lp->ind++;
  va_start (ap, fmt);
  lp->size[i] = vsnprintf (lp->line[i], sizeof lp->line[i], fmt, ap);
  lp->max_size = MAX (lp->max_size, lp->size[i]);
  va_end (ap);
}

static char *
finish_line (LINE *lp)
{
  int i;
  char *p;

  p = lp->buf;
  *p++ = '\a';
  *p++ = '\r';
  *p++ = '\n';
  for (i = 0; i < lp->ind; i++)
    {
      char *q;
      int j;

      for (q = lp->line[i]; *q; q++)
	*p++ = *q;
      for (j = lp->size[i]; j < lp->max_size + 2; j++)
	*p++ = ' ';
      *p++ = '\r';
      *p++ = '\n';
    }
  *p = 0;
  return lp->buf;
}

static int
print_mesg (char *tty, CTL_MSG *request, char *remote_machine)
{
  time_t t;
  LINE ln;
  char *buf;
  struct tm *tm;
  struct iovec iovec;
  char *cp;

  time (&t);
  tm = localtime (&t);
  init_line (&ln);
  format_line (&ln, "");
  format_line (&ln, "Message from Talk_Daemon@%s at %d:%02d ...",
	       hostname, tm->tm_hour , tm->tm_min);
  format_line (&ln, "talk: connection requested by %s@%s",
	       request->l_name, remote_machine);
  format_line (&ln, "talk: respond with:  talk %s@%s",
	       request->l_name, remote_machine);
  format_line (&ln, "");
  format_line (&ln, "");
  buf = finish_line (&ln);

  iovec.iov_base = buf;
  iovec.iov_len = strlen (buf);

  if ((cp = ttymsg (&iovec, 1, tty, RING_WAIT - 5)) != NULL)
    {
      syslog(LOG_CRIT, "%s", cp);
      return FAILED;
    }
  return SUCCESS;
}

/* See if the user is accepting messages. If so, announce that
   a talk is requested. */
int
announce (CTL_MSG *request, char *remote_machine)
{
  char *ttypath;
  int len;
  struct stat st;
  int rc;

  len = sizeof (PATH_DEV) + strlen (request->r_tty) + 2;
  ttypath = malloc (len);
  if (!ttypath)
    {
      syslog (LOG_CRIT, "out of memory");
      exit (1);
    }
  sprintf (ttypath, "%s/%s", PATH_DEV, request->r_tty);
  rc = stat (ttypath, &st);
  free (ttypath);
  if (rc < 0 || (st.st_mode & S_IWGRP) == 0)
    return PERMISSION_DENIED;
  return print_mesg (request->r_tty, request, remote_machine);
}
