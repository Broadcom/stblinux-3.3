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

#include "telnetd.h"
#include <sys/wait.h>

#ifdef AUTHENTICATION
# include <libtelnet/auth.h>
#endif


void
setup_utmp (char *line)
{
  char *ut_id = utmp_ptsid (line, "tn");
  utmp_init (line + sizeof ("/dev/") - 1, ".telnet", ut_id);
}


int
startslave (char *host, int autologin, char *autoname)
{
  pid_t pid;
  int master;
  
#ifdef AUTHENTICATION
  if (!autoname || !autoname[0])
    autologin = 0;

  if (autologin < auth_level)
    {
      fatal (net, "Authorization failed");
      exit (1);
    }
#endif
  pid = forkpty (&master, line, NULL, NULL);
  if (pid < 0)
    {
      if (errno == ENOENT)
	{
	  syslog (LOG_ERR, "Out of ptys");
	  fatal (net, "Out of ptys");
	}
      else
	{
	  syslog (LOG_ERR, "forkpty: %m");
	  fatal (net, "Forkpty");
	}
    }

  if (pid == 0)
      {
	/* Child */
	if (net > 2)
	  close (net);

#ifdef UTMPX
	setup_utmp (line);
#endif
	start_login (host, autologin, line);
      }

  /* Master */
  return master;
}
  
extern char **environ;
/*
 * scrub_env()
 *
 * Remove a few things from the environment that
 * don't need to be there.
 *
 * Security fix included in telnet-95.10.23.NE of David Borman <deb@cray.com>.
 */
static void
scrub_env ()
{
  register char **cpp, **cpp2;

  for (cpp2 = cpp = environ; *cpp; cpp++)
    {
      if (strncmp (*cpp, "LD_", 3) 
	  && strncmp (*cpp, "_RLD_", 5) 
	  && strncmp (*cpp, "LIBPATH=", 8) 
	  && strncmp (*cpp, "IFS=", 4))
	*cpp2++ = *cpp;
    }
  *cpp2 = 0;
}

void
start_login (char *host, int autologin, char *name)
{
  char *cmd;
  int argc;
  char **argv;
  
  scrub_env ();

  /* Set the environment variable "LINEMODE" to indicate our linemode */
  if (lmodetype == REAL_LINEMODE)
    setenv ("LINEMODE", "real", 1);
  else if (lmodetype == KLUDGE_LINEMODE || lmodetype == KLUDGE_OK)
    setenv ("LINEMODE", "kludge", 1);

  cmd = expand_line (login_invocation);
  if (!cmd)
    fatal (net, "can't expand login command line");
  argcv_get (cmd, "", &argc, &argv);
  execv (argv[0], argv);
  syslog (LOG_ERR, "%s: %m\n", cmd);
  fatalperror (net, cmd);
}

void
cleanup (int sig)
{
  char *p;

  if (sig)
    {
      int status;
      pid_t pid = waitpid((pid_t)-1, &status, WNOHANG);
      syslog (LOG_INFO, "child process %ld exited: %d",
	      (long) pid, WEXITSTATUS(status));
    }
  
  p = line + sizeof (PATH_DEV) - 1;
  utmp_logout (p);
  chmod (line, 0644);
  chown (line, 0, 0);
  shutdown (net, 2);
  exit (1);
}

  
