/*
 * Copyright (c) 1983, 1993, 2002
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

#if 0
static char sccsid[] = "@(#)logger.c	8.1 (Berkeley) 6/6/93";
#endif

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <getopt.h>

#define	SYSLOG_NAMES
#include <syslog.h>
#ifndef HAVE_SYSLOG_INTERNAL
# include <syslog-int.h>
#endif

int decode __P((char *, CODE *));
int pencode __P((char *));
static void usage __P((int));

extern char *__progname;

static const char *short_options = "isf:p:t:";
static struct option long_options[] =
{
  { "file", required_argument, 0, 'f' },
  { "priority", required_argument, 0, 'p' },
  { "tag", required_argument, 0, 't' },
  { "help", no_argument, 0, '&' },
  { "version", no_argument, 0, 'V' },
  { 0, 0, 0, 0 }
};

static void
usage (int err)
{
  if (err != 0)
    {
      fprintf (stderr, "Usage: %s [OPTION] ...\n", __progname);
      fprintf (stderr, "Try `%s --help' for more information.\n", __progname);
    }
  else
    {
      fprintf (stdout, "Usage: %s [OPTION] ...\n", __progname);
      fprintf (stdout, "       %s [OPTION] ... MESSAGE\n", __progname);
      puts ("Make entries in the system log.\n\n\
  -i                  Log the process id with every line");
#ifdef LOG_PERROR
      puts ("\
  -s                  Copy the message to stderr");
#endif
      puts ("\
  -f, --file=FILE     Log the content of FILE\n\
  -p, --priority=PRI  Log with priority PRI\n\
  -t, --tag=TAG       Prepend every line with TAG\n\
      --help          Display this help and exit\n\
      --version       Output version information and exit");

      fprintf (stdout, "\nSubmit bug reports to %s.\n", PACKAGE_BUGREPORT);
    }
  exit (err);
}

/* syslog reads from an input and arranges to write the result on the
   system log.  */
int
main (int argc, char *argv[])
{
  int option, logflags, pri;
  char *tag, buf[1024];

#ifndef HAVE___PROGNAME
  __progname = argv[0];
#endif

  tag = NULL;
  pri = LOG_NOTICE;
  logflags = 0;
  while ((option = getopt_long (argc, argv, short_options,
				long_options, 0)) != EOF)
    {
      switch(option)
	{
	case 'f': /* Log from file.  */
	  if (freopen (optarg, "r", stdin) == NULL)
	    {
	      fprintf (stderr, "%s: %s: %s\n", __progname, optarg,
		       strerror (errno));
	      exit(1);
	    }
	  break;

	case 'i': /* Log process id also.  */
	  logflags |= LOG_PID;
	  break;

	case 'p': /* Set priority to log.  */
	  pri = pencode (optarg);
	  break;

	case 's': /* Log to standard error as well.  */
#ifdef LOG_PERROR
	  logflags |= LOG_PERROR;
#else
	  fprintf (stderr, "%s: -s: option not implemented\n", __progname);
	  exit (1);
#endif
	  break;

	case 't': /* Tag message.  */
	  tag = optarg;
	  break;

	case '&': /* Usage.  */
	  usage (0);
	  /* Not reached.  */

	case 'V': /* Version.  */
	  printf ("syslog (%s) %s\n", PACKAGE_NAME, PACKAGE_VERSION);
          exit (0);

	case '?':
	default:
	  usage (1);
	}
    }
  
  argc -= optind;
  argv += optind;
  
  /* Setup for logging.  */
  openlog (tag ? tag : getlogin(), logflags, 0);
  (void) fclose (stdout);
  
  /* Log input line if appropriate.  */
  if (argc > 0)
    {
      char *p, *endp;
      int len;
      
      for (p = buf, endp = buf + sizeof (buf) - 2; *argv;)
	{
	  len = strlen (*argv);
	  if (p + len > endp && p > buf)
	    {
	      syslog (pri, "%s", buf);
	      p = buf;
	    }
	  if (len > sizeof (buf) - 1)
	    syslog (pri, "%s", *argv++);
	  else
	    {
	      if (p != buf)
		*p++ = ' ';
	      memcpy (p, *argv++, len);
	      *(p += len) = '\0';
	    }
	}
      if (p != buf)
	syslog (pri, "%s", buf);
    }
  else
    while (fgets (buf, sizeof (buf), stdin) != NULL)
      syslog (pri, "%s", buf);
  exit (0);
}

/* Decode a symbolic name to a numeric value.  */
int
pencode (char *s)
{
  char *save;
  int fac, lev;
  
  for (save = s; *s && *s != '.'; ++s);
  if (*s)
    {
      *s = '\0';
      fac = decode (save, facilitynames);
      if (fac < 0)
	{
	  fprintf (stderr, "%s: unknown facility name: %s\n", __progname,
		   save);
	  exit (1);
	}
      *s++ = '.';
    }
  else
    {
      fac = 0;
      s = save;
    }
  lev = decode (s, prioritynames);
  if (lev < 0)
    {
      fprintf (stderr, "%s: unknown priority name: %s\n", __progname, save);
      exit(1);
    }
  return ((lev & LOG_PRIMASK) | (fac & LOG_FACMASK));
}

int
decode (char *name, CODE *codetab)
{
  CODE *c;

  if (isdigit (*name))
    return (atoi (name));

  for (c = codetab; c->c_name; c++)
    if (!strcasecmp (name, c->c_name))
      return (c->c_val);

  return (-1);
}
