/* syslogd - log system messages
 *
 * Copyright (c) 1983, 1988, 1993, 1994, 2002
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
static char sccsid[] = "@(#)syslogd.c	8.3 (Berkeley) 4/4/94";
#endif

/*
 *  syslogd -- log system messages
 *
 * This program implements a system log. It takes a series of lines.
 * Each line may have a priority, signified as "<n>" as the first
 * characters of the line.  If this is not present, a default priority
 * is used.
 *
 * To kill syslogd, send a signal 15 (terminate).  A signal 1 (hup)
 * will cause it to reread its configuration file.
 *
 * Defined Constants:
 *
 * MAXLINE -- the maximum line length that can be handled.
 * DEFUPRI -- the default priority for user messages
 * DEFSPRI -- the default priority for kernel messages
 *
 * Author: Eric Allman
 * extensive changes by Ralph Campbell
 * more extensive changes by Eric Allman (again)
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#define IOVCNT          6               /* size of the iovec array */
#define	MAXLINE		1024		/* Maximum line length.  */
#define	MAXSVLINE	240		/* Maximum saved line length.  */
#define DEFUPRI		(LOG_USER|LOG_NOTICE)
#define DEFSPRI		(LOG_KERN|LOG_CRIT)
#define TIMERINTVL	30		/* Interval for checking flush, mark.  */
#define TTYMSGTIME      10              /* Time out passed to ttymsg.  */

#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/un.h>
#ifdef TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h>
#else
# ifdef HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif
#endif
#include <sys/resource.h>
#include <poll.h>
#ifdef HAVE_SYS_TYPES_H
# include <sys/types.h>
#endif

#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

#ifdef HAVE_UTMP_H
# include <utmp.h>
#endif

#ifdef UTMPX
# ifdef HAVE_UTMPX_H
#  include <utmpx.h>
# endif
typedef struct utmpx UTMP;
# define SETUTENT() setutxent()
# define GETUTENT() getutxent()
# define ENDUTENT() endutxent()
#else
typedef struct utmp UTMP;
# define SETUTENT() setutent()
# define GETUTENT() getutent()
# define ENDUTENT() endutent()
#endif

#ifdef HAVE_STDARG_H
# include <stdarg.h>
#else
# include <varargs.h>
#endif

#define SYSLOG_NAMES
#include <syslog.h>
#ifndef HAVE_SYSLOG_INTERNAL
# include <syslog-int.h>
#endif

/* A mask of all facilities mentioned explicitly in the configuration file
 *
 * This is used to support a virtual facility "**" that covers all the rest,
 * so that messages to unexpected facilities won't be lost when "*" is 
 * not logged to a file.
 */
int facilities_seen;

const char *ConfFile = PATH_LOGCONF; /* Default Configuration file.  */
const char *PidFile = PATH_LOGPID; /* Default path to tuck pid.  */
char ctty[] = PATH_CONSOLE; /* Default console to send message info.  */

static int dbg_output;   /* If true, print debug output in debug mode.  */
static int restart;  /* If 1, indicates SIGHUP was dropped.  */

/* Unix socket family to listen.  */
struct funix
{
  const char *name;
  int fd;
} *funix;

size_t nfunix; /* Number of unix sockets in the funix array.  */

/*
 * Flags to logmsg().
 */

#define IGN_CONS	0x001	/* Don't print on console.  */
#define SYNC_FILE	0x002	/* Do fsync on file after printing.  */
#define ADDDATE		0x004	/* Add a date to the message.  */
#define MARK		0x008	/* This message is a mark.  */

/* This structure represents the files that will have log copies
   printed.  */

struct filed
{
  struct filed *f_next;			/* Next in linked list.  */
  short f_type;				/* Entry type, see below.  */
  short f_file;				/* File descriptor.  */
  time_t f_time;			/* Time this was last written.  */
  u_char f_pmask[LOG_NFACILITIES+1];	/* Priority mask.  */
  union
  {
    struct
    {
      int f_nusers;
      char **f_unames;
    } f_user;                           /* Send a message to a user.  */
    struct
    {
      char *f_hname;
      struct sockaddr_in f_addr;
    } f_forw;				/* Forwarding address.  */
    char *f_fname;                      /* Name use for Files|Pipes|TTYs.  */
  } f_un;
  char f_prevline[MAXSVLINE];		/* Last message logged.  */
  char f_lasttime[16];			/* Time of last occurrence.  */
  char *f_prevhost;			/* Host from which recd.  */
  int f_prevpri;			/* Pri of f_prevline.  */
  int f_prevlen;			/* Length of f_prevline.  */
  int f_prevcount;			/* Repetition cnt of prevline.  */
  size_t f_repeatcount;			/* Number of "repeated" msgs.  */
  int f_flags;				/* Additional flags see below.  */
};

struct filed *Files;	/* Linked list of files to log to.  */
struct filed consfile;	/* Console `file'.  */

/* Values for f_type.  */
#define F_UNUSED	0	/* Unused entry.  */
#define F_FILE		1	/* Regular file.  */
#define F_TTY		2	/* Terminal.  */
#define F_CONSOLE	3	/* Console terminal.  */
#define F_FORW		4	/* Remote machine.  */
#define F_USERS		5	/* List of users.  */
#define F_WALL		6	/* Everyone logged on.  */
#define F_FORW_SUSP	7	/* Suspended host forwarding.  */
#define F_FORW_UNKN	8	/* Unknown host forwarding.  */
#define F_PIPE		9	/* Named pipe.  */

const char *TypeNames[] =
{
  "UNUSED",
  "FILE",
  "TTY",
  "CONSOLE",
  "FORW",
  "USERS",
  "WALL",
  "FORW(SUSPENDED)",
  "FORW(UNKNOWN)",
  "PIPE"
};

/* Flags in filed.f_flags.  */
#define OMIT_SYNC	0x001   /* Omit fsync after printing.  */

/* Constants for the F_FORW_UNKN retry feature.  */
#define INET_SUSPEND_TIME 180	/* Number of seconds between attempts.  */
#define INET_RETRY_MAX	10	/* Number of times to try gethostbyname().  */

/* Intervals at which we flush out "message repeated" messages, in
 seconds after previous message is logged.  After each flush, we move
 to the next interval until we reach the largest.  */

int repeatinterval[] = { 30, 60 };	/* Number of seconds before flush.  */
#define	MAXREPEAT ((sizeof(repeatinterval) / sizeof(repeatinterval[0])) - 1)
#define	REPEATTIME(f)	((f)->f_time + repeatinterval[(f)->f_repeatcount])
#define	BACKOFF(f)	{ if (++(f)->f_repeatcount > MAXREPEAT) \
				 (f)->f_repeatcount = MAXREPEAT; \
			}

/* Delimiter in arguments to command line options `-s' and `-l'.  */
#define LIST_DELIMITER	':'

char *LocalHostName;            /* Our hostname.  */
char *LocalDomain;              /* Our local domain name.  */
int finet = -1;                 /* Internet datagram socket fd.  */
int fklog = -1;                 /* Kernel log device fd.  */
int LogPort;                    /* Port number for INET connections.  */
int Initialized;                /* True when we are initialized. */
int MarkInterval = 20 * 60;     /* Interval between marks in seconds.  */
int MarkSeq;                    /* Mark sequence number.  */

int Debug;                      /* True if in debug mode.  */
int AcceptRemote;               /* Receive messages that come via UDP.  */
char **StripDomains;            /* Domains to be stripped before logging.  */
char **LocalHosts;              /* Hosts to be logged by their hostname.  */
int NoDetach;                   /* Don't run in background and detach
				   from ctty. */
int NoHops = 1;	                /* Bounce syslog messages for other hosts.  */
int NoKLog;                     /* Don't attempt to log kernel device.  */
int NoUnixAF;                   /* Don't listen to unix sockets. */
int NoForward;                  /* Don't forward messages.  */
time_t  now;                    /* Time use for mark and forward supending.  */
int force_sync;                 /* GNU/Linux behaviour to sync on every line.
				   This off by default. Set to 1 to enable.  */

extern char *localhost __P ((void));
extern int waitdaemon __P ((int nochdir, int noclose, int maxwait));
extern char *__progname;

void cfline __P ((const char *, struct filed *));
const char *cvthname __P ((struct sockaddr_in *));
int decode __P ((const char *, CODE *));
void die __P ((int));
void domark __P ((int));
void fprintlog __P ((struct filed *, const char *, int, const char *));
void init __P ((int));
void logerror __P ((const char *));
void logmsg __P ((int, const char *, const char *, int));
void printline __P ((const char *, const char *));
void printsys __P ((const char *));
char *ttymsg __P ((struct iovec *, int, char *, int));
static void usage __P ((int));
void wallmsg __P ((struct filed *, struct iovec *));
char **crunch_list __P ((char **oldlist, char *list));
char *textpri __P ((int pri));
void dbg_toggle __P ((int));
static void dbg_printf __P ((const char *, ...));
void trigger_restart __P ((int));
static void add_funix __P ((const char *path));
static int create_unix_socket __P ((const char *path));
static int create_inet_socket __P ((void));

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
      puts ("Log system messages.\n\n\
  -f, --rcfile=FILE  Override configuration file (default: " PATH_LOGCONF ")\n\
      --pidfile=FILE Override pidfile (default: " PATH_LOGPID ")\n\
  -n, --no-detach    Don't enter daemon mode\n\
  -d, --debug        Print debug information (implies -n)\n\
  -p, --socket FILE  Override default unix domain socket " PATH_LOG "\n\
  -a SOCKET          Add unix socket to listen to (up to 19)\n\
  -r, --inet         Receive remote messages via internet domain socket\n\
      --no-unixaf    Do not listen on unix domain sockets (overrides -a and -p)\n\
  -S, --sync         Force a file sync on every line");
#ifdef PATH_KLOG
      puts ("\
      --no-klog      Do not listen to kernel log device " PATH_KLOG);
#endif
      puts ("\
      --no-forward   Do not forward any messages (overrides -h)\n\
  -h, --hop          Forward messages from remote hosts\n\
  -m, --mark=INTVL   Specify timestamp interval in logs (0 for no timestamps)\n\
  -l HOSTLIST        Log hosts in HOSTLIST by their hostname\n\
  -s DOMAINLIST      List of domains which should be stripped from the FQDN\n\
                     of hosts before logging their name.\n\
      --help         Display this help and exit\n\
  -V, --version      Output version information and exit");

      fprintf (stdout, "\nSubmit bug reports to %s.\n", PACKAGE_BUGREPORT);
    }
  exit (err);
}

static const char *short_options = "a:dhf:Kl:m:np:rs:VS";
static struct option long_options[] =
{
  { "debug", no_argument, 0, 'd' },
  { "rcfile", required_argument, 0, 'f' },
  { "pidfile", required_argument, 0, 'P' },
  { "hop", no_argument, 0, 'h' },
  { "mark", required_argument, 0, 'm' },
  { "no-detach", no_argument, 0, 'n' },
  { "socket", required_argument, 0, 'p' },
  { "inet", no_argument, 0, 'r' },
  { "no-klog", no_argument, 0, 'K' },
  { "no-forward", no_argument, 0, 'F' },
  { "no-unixaf", no_argument, 0, 'U' },
  { "sync", no_argument, 0, 'S' },
  { "help", no_argument, 0, '&' },
  { "version", no_argument, 0, 'V' },
#if 0 /* Not sure about the long name. Maybe move into conffile even.  */
  { "", required_argument, 0, 'a' },
  { "", required_argument, 0, 's' },
  { "", required_argument, 0, 'l' },
#endif
  { 0, 0, 0, 0 }
};

int
main (int argc, char *argv[])
{
  int option;
  size_t i;
  FILE *fp;
  char *p;
  char line[MAXLINE + 1];
  char kline[MAXLINE + 1];
  int kline_len = 0;
  pid_t ppid = 0;	/* We run in debug mode and didn't fork.  */
  struct pollfd *fdarray;
  unsigned long nfds = 0;

#ifndef HAVE___PROGNAME
  __progname = argv[0];
#endif

  /* Initiliaze PATH_LOG as the first element of the unix sockets array.  */
  add_funix (PATH_LOG);

  while ((option = getopt_long (argc, argv, short_options,
				long_options, 0)) != EOF)
    {
      switch(option)
	{
	case 'a': /* Add to unix socket array.  */
	  add_funix (optarg);
	  break;

	case 'd': /* Debug mode.  */
	  Debug = 1;
	  NoDetach = 1;
	  break;

	case 'f': /* Override the default config file.  */
	  ConfFile = optarg;
	  break;

	case 'h': /* Disable forwarding.  */
	  NoHops = 0;
	  break;

	case 'l': /* Add host to be log whithout the FQDN.  */
	  LocalHosts = crunch_list (LocalHosts, optarg);
	  break;

	case 'm': /* Set the timestamp interval mark.  */
	  MarkInterval = atoi (optarg) * 60;
	  break;

	case 'n': /* Don't run in background.  */
	  NoDetach = 1;
	  break;

	case 'p': /* Overide PATH_LOG name.  */
	  funix[0].name = optarg;
	  funix[0].fd = -1;
	  break;

	case 'r': /* Enable remote message via inet socket.  */
	  AcceptRemote = 1;
	  break;

	case 's': /* List of domain names to strip.  */
	  StripDomains = crunch_list (StripDomains, optarg);
	  break;

	case 'P': /* Override pidfile.  */
	  PidFile = optarg;
	  break;

	case 'K': /* Disable kernel logging.  */
	  NoKLog = 1;
	  break;

	case 'F': /* Disable forwarding.  */
	  NoForward = 1;
	  break;

	case 'U': /* Disable  unix sockets.  */
	  NoUnixAF = 1;
	  break;

	case 'S': /* Sync on every line.  */
	  force_sync = 1;
	  break;

	case '&': /* Usage.  */
	  usage (0);
	  /* Not reached.  */

	case 'V': /* Version.  */
	  printf ("syslogd (%s) %s\n", PACKAGE_NAME, PACKAGE_VERSION);
	  exit (0);

	case '?':
	default:
	  usage (1);
	  /* Not reached.  */
	}
    }

  /* Bail out, wrong usage */
  argc -= optind;
  if (argc != 0)
    usage (1);

  /* Daemonise, if not, set the buffering for line buffer.  */
  if (!NoDetach)
    {
      /* History:  According to the GNU/Linux sysklogd ChangeLogs
	 "Wed Feb 14 12:42:09 CST 1996:  Dr. Wettstein
	 Parent process of syslogd does not exit until child process has
	 finished initialization process.  This allows rc.* startup to
	 pause until syslogd facility is up and operating."

	 IMO, the GNU/Linux distributors should fix there booting
	 sequence. But we still keep the approach.  */
      ppid = waitdaemon (0, 0, 30);
      if (ppid < 0)
	{
	  fprintf (stderr, "%s: could not become daemon: %s\n",
		   __progname, strerror (errno));
	  exit (1);
	}
    }
  else
    {
      dbg_output = 1;
      setvbuf (stdout, 0, _IOLBF, 0);
    }

  /* Get our hostname.  */
  LocalHostName = localhost ();
  if (LocalHostName == NULL)
    {
      fprintf (stderr, "%s: can't get local host name: %s\n",
	       __progname, strerror (errno));
      exit (2);
    }
  /* Get the domainname.  */
  p = strchr (LocalHostName, '.');
  if (p != NULL)
    {
      *p++ = '\0';
      LocalDomain = p;
    }
  else
    {
      struct hostent *host_ent;

      /* Try to resolve the domainname by calling DNS. */
      host_ent = gethostbyname (LocalHostName);
      if (host_ent)
	{
	  /* Override what we had */
	  if (LocalHostName)
	    free (LocalHostName);
	  LocalHostName = strdup (host_ent->h_name);
	  p = strchr (LocalHostName, '.');
	  if (p != NULL)
	    {
	      *p++ = '\0';
	      LocalDomain = p;
	    }
	}
      if (LocalDomain == NULL)
	LocalDomain = strdup ("");
    }

  consfile.f_type = F_CONSOLE;
  consfile.f_un.f_fname = strdup (ctty);

  (void) signal (SIGTERM, die);
  (void) signal (SIGINT, Debug ? die : SIG_IGN);
  (void) signal (SIGQUIT, Debug ? die : SIG_IGN);
  (void) signal (SIGALRM, domark);
  (void) signal (SIGUSR1, Debug ? dbg_toggle : SIG_IGN);
  (void) alarm (TIMERINTVL);

  /* We add  2 = 1(klog) + 1(inet), even if they may be not use.  */
  fdarray = (struct pollfd *) malloc ((nfunix + 2) * sizeof (*fdarray));
  if (fdarray == NULL)
    {
      fprintf (stderr, "%s: can't allocate fd table: %s\n",
	       __progname, strerror (errno));
      exit (2);
    }

#ifdef PATH_KLOG
  /* Initialize kernel logging and add to the list.  */
  if (!NoKLog)
    {
      fklog = open (PATH_KLOG, O_RDONLY, 0);
      if (fklog >= 0)
	{
	  fdarray[nfds].fd = fklog;
	  fdarray[nfds].events = POLLIN | POLLPRI;
	  nfds++;
	  dbg_printf ("Klog open %s\n", PATH_KLOG);
	}
      else
	dbg_printf ("Can't open %s: %s\n", PATH_KLOG, strerror (errno));
    }
#endif

  /* Initialize unix sockets.  */
  if (!NoUnixAF)
    {
      for (i = 0; i < nfunix; i++)
	{
	  funix[i].fd = create_unix_socket (funix[i].name);
	  if (funix[i].fd >= 0)
	    {
	      fdarray[nfds].fd = funix[i].fd;
	      fdarray[nfds].events = POLLIN | POLLPRI;
	      nfds++;
	      dbg_printf ("Opened UNIX socket `%s'.\n", funix[i].name);
	    }
	  else
	      dbg_printf ("Can't open %s: %s\n", funix[i].name,
			  strerror (errno));
	}
    }

  /* Initialize inet socket and add them to the list.  */
  if (AcceptRemote || (!NoForward))
    {
      finet = create_inet_socket ();
      if (finet >= 0)
	{
	  fdarray[nfds].fd = finet;
	  fdarray[nfds].events = POLLIN | POLLPRI;
	  nfds++;
	  dbg_printf ("Opened syslog UDP port.\n");
	}
      else
	dbg_printf ("Can't open UDP port: %s\n", strerror (errno));
    }

  /* read configuration file */
  init (0);

  /* Tuck my process id away.  */
  fp = fopen (PidFile, "w");
  if (fp != NULL)
    {
      fprintf (fp, "%d\n", getpid ());
      (void) fclose (fp);
    }

  dbg_printf ("off & running....\n");

  signal (SIGHUP, trigger_restart);

  if (Debug)
    {
      dbg_printf ("Debugging disabled, send SIGUSR1 to turn on debugging.\n");
      dbg_output = 0;
    }

  /* If we're doing waitdaemon(), tell the parent to exit,
     we are ready to roll.  */
  if (ppid)
    kill (ppid, SIGALRM);

  for (;;)
    {
      int nready;
      nready = poll (fdarray, nfds, -1);
      if (nready == 0) /* ??  noop */
	continue;

      /* Sighup was dropped.  */
      if (restart)
	{
	  dbg_printf ("\nReceived SIGHUP, restarting syslogd.\n");
	  init (0);
	  restart = 0;
	  continue;
	}

      if (nready < 0)
	{
	  if (errno != EINTR)
	    logerror ("poll");
	  continue;
	}

      /*dbg_printf ("got a message (%d)\n", nready);*/

      for (i = 0; i < nfds; i++)
	if (fdarray[i].revents & (POLLIN | POLLPRI))
	  {
	    int result;
	    size_t len;
	    if (fdarray[i].fd == -1)
		continue;
	    else if (fdarray[i].fd == fklog)
	      {
		result = read (fdarray[i].fd, &kline[kline_len],
			       sizeof (kline) - kline_len - 1);

		if (result > 0)
		  {
		    kline_len += result;
		  }
		else if (result < 0 && errno != EINTR)
		  {
		    logerror ("klog");
		    fdarray[i].fd = fklog = -1;
		  }

		while (1)
		  {
		    char *bol, *eol;

		    kline[kline_len] = '\0';

		    for (bol = kline, eol = strchr (kline, '\n'); eol;
			 bol = eol, eol = strchr (bol, '\n'))
		      {
			*(eol++) = '\0';
			kline_len -= (eol - bol);
			printsys (bol);
		      }

		    /* This loop makes sure the daemon won't lock up
		     * on null bytes in the klog stream.  They still hurt 
		     * efficiency, acting like a message separator that 
		     * forces a shift-and-reiterate when the buffer was
		     * never full.
		     */
		    while (kline_len && !*bol)
		      {
			bol++;
			kline_len--;
		      }

		    if (!kline_len)
		      break;

		    if (bol != kline)
		      {
			/* shift the partial line to start of buffer, so 
			 * we can re-iterate.
			 */
			memmove (kline, bol, kline_len);
		      }
		    else
		      {
			if (kline_len < MAXLINE)
			  break;

			/* The pathological case of a single message that
			 * overfills our buffer.  The best we can do is
			 * log it in pieces.
			 */
			printsys (kline);

			/* Clone priority signal if present 
			 * We merely shift the kline_len pointer after
			 * it so the next chunk is written after it.
			 *
			 * strchr(kline,'>') is not used as it would allow 
			 * a pathological line ending in '>' to cause an
			 * endless loop.
			 */
			if (kline[0] == '<'
			    && isdigit (kline[1]) && kline[2] == '>')
			  kline_len = 3;
			else
			  kline_len = 0;
		      }
		  }
	      }
	    else if (fdarray[i].fd == finet)
	      {
		struct sockaddr_in frominet;
		/*dbg_printf ("inet message\n");*/
		len = sizeof (frominet);
		memset (line, '\0', sizeof (line));
		result = recvfrom (fdarray[i].fd, line, MAXLINE, 0,
				   (struct sockaddr *) &frominet, &len);
		if (result > 0)
		  {
		    line[result] = '\0';
		    printline (cvthname (&frominet), line);
		  }
		else if (result < 0 && errno != EINTR)
		  logerror ("recvfrom inet");
	      }
	    else
	      {
		struct sockaddr_un fromunix;
		/*dbg_printf ("unix message\n");*/
		len = sizeof (fromunix);
		result = recvfrom (fdarray[i].fd, line, MAXLINE, 0,
				   (struct sockaddr *) &fromunix, &len);
		if (result > 0)
		  {
		    line[result] = '\0';
		    printline (LocalHostName, line);
		  }
		else if (result < 0 && errno != EINTR)
		  logerror ("recvfrom unix");
	      }
	  }
	else if (fdarray[i].revents & POLLNVAL)
	  {
	    logerror ("poll nval\n");
	    fdarray[i].fd = -1;
	  }
	else if (fdarray[i].revents & POLLERR)
	  logerror ("poll err\n");
	else if (fdarray[i].revents & POLLHUP)
	  logerror ("poll hup\n");
    } /* for (;;) */
} /* main */

#ifndef SUN_LEN
#define SUN_LEN(unp) (strlen((unp)->sun_path) + 3)
#endif

static void
add_funix (const char *name)
{
  funix = realloc (funix, (nfunix + 1)*sizeof(*funix));
  if (funix == NULL)
    {
      fprintf (stderr, "%s: cannot allocate space for unix sockets: %s\n",
	       __progname, strerror (errno));
      exit (1);
    }
  funix[nfunix].name = name;
  funix[nfunix].fd = -1;
  nfunix++;
}

static int
create_unix_socket (const char *path)
{
  int fd;
  struct sockaddr_un sunx;
  char line[MAXLINE + 1];

  if (path[0] == '\0')
    return -1;

  (void) unlink (path);

  memset (&sunx, 0, sizeof (sunx));
  sunx.sun_family = AF_UNIX;
  (void) strncpy (sunx.sun_path, path, sizeof (sunx.sun_path));
  fd = socket (AF_UNIX, SOCK_DGRAM, 0);
  if (fd < 0 || bind(fd, (struct sockaddr *) &sunx, SUN_LEN (&sunx)) < 0
      || chmod(path, 0666) < 0)
    {
      snprintf (line, sizeof (line), "cannot create %s", path);
      logerror (line);
      dbg_printf ("cannot create %s: %s\n", path, strerror (errno));
      close (fd);
    }
  return fd;
}

static int
create_inet_socket ()
{
  int fd;
  struct sockaddr_in sin;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    {
      logerror ("unknown protocol, suspending inet service");
      return fd;
    }

  memset (&sin, 0, sizeof (sin));
  sin.sin_family = AF_INET;
  sin.sin_port = LogPort;
  if (bind(fd, (struct sockaddr *) &sin, sizeof (sin)) < 0)
    {
      logerror("bind, suspending inet");
      close(fd);
      return -1;
    }
  return fd;
}

char **
crunch_list (char **oldlist, char *list)
{
  int count, i;
  char *p, *q;

  p = list;

  /* Strip off trailing delimiters.  */
  while (p[strlen (p) - 1] == LIST_DELIMITER)
    {
      p[strlen (p) - 1] = '\0';
    }
  /* Cut off leading delimiters.  */
  while (p[0] == LIST_DELIMITER)
    {
      p++;
    }

  /* Bailout early the string is empty.  */
  if (*p == '\0')
    return oldlist;

  /* Count delimiters to calculate elements.  */
  for (count = 1, i = 0; p[i]; i++)
    if (p[i] == LIST_DELIMITER) count++;

  /* Count how many we add in the old list.  */
  for (i = 0; oldlist && oldlist[i]; i++)
    ;

  /* allocate enough space */
  oldlist = (char **) realloc (oldlist, (i + count + 1) * sizeof (*oldlist));
  if (oldlist == NULL)
    {
      fprintf (stderr, "%s: can't allocate memory: %s",
	       __progname, strerror (errno));
      exit (1);
    }

  /*
    We now can assume that the first and last
    characters are different from any delimiters,
    so we don't have to care about it anymore.  */

  /* Start from where we left last time.  */
  for (count = i; (q = strchr (p, LIST_DELIMITER)) != NULL;
       count++, p = q, p++)
    {
      oldlist[count] = (char *) malloc ((q - p + 1) * sizeof(char));
      if (oldlist[count] == NULL)
	{
	  fprintf (stderr, "%s: can't allocate memory: %s",
		   __progname, strerror (errno));
	  exit (1);
	}
      strncpy (oldlist[count], p, q - p);
      oldlist[count][q - p] = '\0';
    }

  /* take the last one */
  oldlist[count] = (char *) malloc ((strlen (p) + 1) * sizeof (char));
  if (oldlist[count] == NULL)
    {
      fprintf (stderr, "%s: can't allocate memory: %s",
	       __progname, strerror (errno));
      exit(1);
    }
  strcpy (oldlist[count], p);

  oldlist[++count] = NULL; /* terminate the array with a NULL */

  if (Debug)
    {
      for (count = 0; oldlist[count]; count++)
	printf ("#%d: %s\n", count, oldlist[count]);
    }
  return oldlist;
}

/* Take a raw input line, decode the message, and print the message on
   the appropriate log files.  */
void
printline (const char *hname, const char *msg)
{
  int c, pri;
  const char *p;
  char *q, line[MAXLINE + 1];

  /* test for special codes */
  pri = DEFUPRI;
  p = msg;
  if (*p == '<')
    {
      pri = 0;
      while (isdigit (*++p))
	pri = 10 * pri + (*p - '0');
      if (*p == '>')
	++p;
    }
  if (pri &~ (LOG_FACMASK|LOG_PRIMASK))
    pri = DEFUPRI;

  /* don't allow users to log kernel messages */
  if (LOG_FAC (pri) == LOG_KERN)
    pri = LOG_MAKEPRI (LOG_USER, LOG_PRI (pri));

  q = line;
  while ((c = *p++) != '\0' &&
	 q < &line[sizeof (line) - 1])
    if (iscntrl(c))
      if (c == '\n')
	*q++ = ' ';
      else if (c == '\t')
	*q++ = '\t';
      else if (c >= 0177)
	*q++ = c;
      else
	{
	  *q++ = '^';
	  *q++ = c ^ 0100;
	}
    else
      *q++ = c;
  *q = '\0';

  /* This for the default behaviour on GNU/Linux syslogd who
     sync on every line.  */
  if (force_sync)
    logmsg (pri, line, hname, SYNC_FILE);
  logmsg (pri, line, hname, 0);
}

/* Take a raw input line from /dev/klog, split and format similar to
   syslog().  */
void
printsys (const char *msg)
{
  int c, pri, flags;
  char *lp, *q, line[MAXLINE + 1];
  const char *p;

  strcpy (line, "vmunix: ");
  lp = line + strlen (line);
  for (p = msg; *p != '\0'; )
    {
      flags = SYNC_FILE | ADDDATE;	/* Fsync after write.  */
      pri = DEFSPRI;
      if (*p == '<')
	{
	  pri = 0;
	  while (isdigit (*++p))
	    pri = 10 * pri + (*p - '0');
	  if (*p == '>')
	    ++p;
	}
      else
	{
	  /* kernel printf's come out on console */
	  flags |= IGN_CONS;
	}
      if (pri &~ (LOG_FACMASK|LOG_PRIMASK))
	pri = DEFSPRI;
      q = lp;
      while (*p != '\0' && (c = *p++) != '\n' &&
	     q < &line[MAXLINE])
	*q++ = c;
      *q = '\0';
      logmsg (pri, line, LocalHostName, flags);
    }
}

/* Decode a priority into textual information like auth.emerg.  */
char *
textpri (int pri)
{
  static char res[20];
  CODE *c_pri, *c_fac;

  for (c_fac = facilitynames; c_fac->c_name
	 && !(c_fac->c_val == LOG_FAC (pri) << 3); c_fac++);
  for (c_pri = prioritynames; c_pri->c_name
	 && !(c_pri->c_val == LOG_PRI (pri)); c_pri++);

  snprintf (res, sizeof (res), "%s.%s", c_fac->c_name, c_pri->c_name);

  return res;
}

/* Log a message to the appropriate log files, users, etc. based on
   the priority.  */
void
logmsg (int pri, const char *msg, const char *from, int flags)
{
  struct filed *f;
  int fac, msglen, prilev;
#ifdef HAVE_SIGACTION
  sigset_t sigs, osigs;
#else
  int omask;
#endif

  const char *timestamp;

  dbg_printf ("(logmsg): %s (%d), flags %x, from %s, msg %s\n",
	      textpri (pri), pri, flags, from, msg);

#ifdef HAVE_SIGACTION
  sigemptyset (&sigs);
  sigaddset (&sigs, SIGHUP);
  sigaddset (&sigs, SIGALRM);
  sigprocmask (SIG_BLOCK, &sigs, &osigs);
#else
  omask = sigblock (sigmask (SIGHUP) | sigmask (SIGALRM));
#endif

  /* Check to see if msg looks non-standard.  */
  msglen = strlen (msg);
  if (msglen < 16 || msg[3] != ' ' || msg[6] != ' ' ||
      msg[9] != ':' || msg[12] != ':' || msg[15] != ' ')
    flags |= ADDDATE;

  (void) time (&now);
  if (flags & ADDDATE)
    timestamp = ctime (&now) + 4;
  else
    {
      timestamp = msg;
      msg += 16;
      msglen -= 16;
    }

  /* Extract facility and priority level.  */
  if (flags & MARK)
    fac = LOG_NFACILITIES;
  else
    fac = LOG_FAC (pri);
  prilev = LOG_PRI (pri);

  /* Log the message to the particular outputs. */
  if (!Initialized)
    {
      f = &consfile;
      f->f_file = open (ctty, O_WRONLY, 0);
      f->f_prevhost = strdup (LocalHostName);
      if (f->f_file >= 0)
	{
	  fprintlog (f, from, flags, msg);
	  (void) close (f->f_file);
	}
#ifdef HAVE_SIGACTION
      sigprocmask (SIG_SETMASK, &osigs, 0);
#else
      (void) sigsetmask (omask);
#endif
      return;
    }
  for (f = Files; f; f = f->f_next)
    {
      /* Skip messages that are incorrect priority. */
      if (!(f->f_pmask[fac] & LOG_MASK (prilev)) ||
	  f->f_pmask[fac] == INTERNAL_NOPRI)
	continue;

      if (f->f_type == F_CONSOLE && (flags & IGN_CONS))
	continue;

      /* Don't output marks to recently written files.  */
      if ((flags & MARK) && (now - f->f_time) < MarkInterval / 2)
	continue;

      /* Suppress duplicate lines to this file.  */
      if ((flags & MARK) == 0 && msglen == f->f_prevlen && f->f_prevhost
	  && !strcmp (msg, f->f_prevline) && !strcmp (from, f->f_prevhost))
	{
	  (void) strncpy (f->f_lasttime, timestamp,
			  sizeof (f->f_lasttime) - 1);
	  f->f_prevcount++;
	  dbg_printf ("msg repeated %d times, %ld sec of %d\n",
		      f->f_prevcount, now - f->f_time,
		      repeatinterval[f->f_repeatcount]);
	  /* If domark would have logged this by now, flush it now (so we
	     don't hold isolated messages), but back off so we'll flush
	     less often in the future.  */
	  if (now > REPEATTIME (f))
	    {
	      fprintlog (f, from, flags, (char *)NULL);
	      BACKOFF (f);
	    }
	}
      else
	{
	  /* New line, save it.  */
	  if (f->f_prevcount)
	    fprintlog (f, from, 0, (char *)NULL);
	  f->f_repeatcount = 0;
	  (void) strncpy (f->f_lasttime, timestamp,
			  sizeof (f->f_lasttime) - 1);
	  if (f->f_prevhost)
	    free (f->f_prevhost);
	  f->f_prevhost = strdup (from);
	  if (msglen < MAXSVLINE)
	    {
	      f->f_prevlen = msglen;
	      f->f_prevpri = pri;
	      (void) strcpy (f->f_prevline, msg);
	      fprintlog (f, from, flags, (char *)NULL);
	    }
	  else
	    {
	      f->f_prevline[0] = 0;
	      f->f_prevlen = 0;
	      fprintlog (f, from, flags, msg);
	    }
	}
    }
#ifdef HAVE_SIGACTION
  sigprocmask (SIG_SETMASK, &osigs, 0);
#else
  (void) sigsetmask (omask);
#endif
}

void
fprintlog (struct filed *f, const char *from, int flags, const char *msg)
{
  struct iovec iov[IOVCNT];
  struct iovec *v;
  int l;
  char line[MAXLINE + 1], repbuf[80], greetings[200];
  time_t fwd_suspend;
  struct hostent *hp;

  v = iov;
  /* Be paranoid.  */
  memset (v, 0, sizeof (struct iovec) * IOVCNT );
  if (f->f_type == F_WALL)
    {
      v->iov_base = greetings;
      snprintf (greetings, sizeof (greetings),
		"\r\n\7Message from syslogd@%s at %.24s ...\r\n",
		f->f_prevhost, ctime (&now));
      v->iov_len = strlen (greetings);
      v++;
      v->iov_base = (char *)"";
      v->iov_len = 0;
      v++;
    }
  else
    {
      v->iov_base = f->f_lasttime;
      v->iov_len = sizeof (f->f_lasttime) - 1;
      v++;
      v->iov_base = (char *)" ";
      v->iov_len = 1;
      v++;
    }
  v->iov_base = f->f_prevhost;
  v->iov_len = strlen (v->iov_base);
  v++;
  v->iov_base = (char *)" ";
  v->iov_len = 1;
  v++;

  if (msg)
    {
      v->iov_base = (char *)msg;
      v->iov_len = strlen (msg);
    }
  else if (f->f_prevcount > 1)
    {
      v->iov_base = repbuf;
      snprintf (repbuf, sizeof (repbuf), "last message repeated %d times",
		f->f_prevcount);
      v->iov_len = strlen (repbuf);
    }
  else
    {
      v->iov_base = f->f_prevline;
      v->iov_len = f->f_prevlen;
    }
  v++;

  dbg_printf ("Logging to %s", TypeNames[f->f_type]);

  switch (f->f_type)
    {
    case F_UNUSED:
      f->f_time = now;
      dbg_printf ("\n");
      break;

    case F_FORW_SUSP:
      fwd_suspend = time((time_t *) 0) - f->f_time;
      if ( fwd_suspend >= INET_SUSPEND_TIME )
	{
	  dbg_printf ("\nForwarding suspension over, retrying FORW ");
	  f->f_type = F_FORW;
	  goto f_forw;
	}
      else
	{
	  dbg_printf (" %s\n", f->f_un.f_forw.f_hname);
	  dbg_printf ("Forwarding suspension not over, time left: %d.\n",
		      INET_SUSPEND_TIME - fwd_suspend);
	}
      break;

    case F_FORW_UNKN:
      dbg_printf (" %s\n", f->f_un.f_forw.f_hname);
      fwd_suspend = time ((time_t *) 0) - f->f_time;
      if (fwd_suspend >= INET_SUSPEND_TIME)
	{
	  hp = gethostbyname (f->f_un.f_forw.f_hname);
	  if (hp == NULL)
	    {
	      extern int h_errno;
	      dbg_printf ("Failure: %s\n", hstrerror (h_errno));
	      dbg_printf ("Retries: %d\n", f->f_prevcount);
	      if ( --f->f_prevcount < 0 )
		f->f_type = F_UNUSED;
	    }
	  else
	    {
	      dbg_printf ("%s found, resuming.\n", f->f_un.f_forw.f_hname);
	      memcpy (&f->f_un.f_forw.f_addr.sin_addr,
		      hp->h_addr, hp->h_length);
	      f->f_prevcount = 0;
	      f->f_type = F_FORW;
	      goto f_forw;
	    }
	}
      else
	dbg_printf ("Forwarding suspension not over, time left: %d\n",
		    INET_SUSPEND_TIME - fwd_suspend);
      break;

    case F_FORW:
    f_forw:
    dbg_printf (" %s\n", f->f_un.f_forw.f_hname);
    if (strcasecmp (from, LocalHostName) && NoHops)
      dbg_printf ("Not forwarding remote message.\n");
    else if (!NoForward)
      dbg_printf ("Not forwarding because forwarding is disabled.\n");
    else if (finet < 0)
      dbg_printf ("Not forwarding because of invalid inet fd.\n");
    else
      {
	f->f_time = now;
	snprintf (line, sizeof (line), "<%d>%.15s %s",
		  f->f_prevpri, (char *)iov[0].iov_base,
		  (char *)iov[4].iov_base);
	l = strlen (line);
	if (l > MAXLINE)
	  l = MAXLINE;
	if (sendto (finet, line, l, 0,
		    (struct sockaddr *)&f->f_un.f_forw.f_addr,
		    sizeof(f->f_un.f_forw.f_addr)) != l)
	  {
	    int e = errno;
	    dbg_printf ("INET sendto error: %d = %s.\n", e, strerror(e));
	    f->f_type = F_FORW_SUSP;
	    errno = e;
	    logerror ("sendto");
	  }
      }
    break;

    case F_CONSOLE:
      f->f_time = now;
      if (flags & IGN_CONS)
	{
	  dbg_printf (" (ignored)\n");
	  break;
	}
      /* FALLTHROUGH */

    case F_TTY:
    case F_FILE:
    case F_PIPE:
      f->f_time = now;
      dbg_printf (" %s\n", f->f_un.f_fname);
      if (f->f_type == F_TTY || f->f_type == F_CONSOLE)
	{
	  v->iov_base = (char *)"\r\n";
	  v->iov_len = 2;
	}
      else
	{
	  v->iov_base = (char *)"\n";
	  v->iov_len = 1;
	}
    again:
      if (writev (f->f_file, iov, IOVCNT) < 0)
	{
	  int e = errno;

	  /* XXX: If a named pipe is full, ignore it.  */
	  if (f->f_type == F_PIPE && e == EAGAIN)
	    break;

	  (void) close (f->f_file);
	  /* Check for errors on TTY's due to loss of tty. */
	  if ((e == EIO || e == EBADF)
	      && (f->f_type == F_TTY || f->f_type == F_CONSOLE))
	    {
	      f->f_file = open (f->f_un.f_fname, O_WRONLY|O_APPEND, 0);
	      if (f->f_file < 0)
		{
		  f->f_type = F_UNUSED;
		  logerror (f->f_un.f_fname);
		}
	      else
		goto again;
	    }
	  else
	    {
	      f->f_type = F_UNUSED;
	      errno = e;
	      logerror (f->f_un.f_fname);
	    }
	}
      else if ((flags & SYNC_FILE) && !(f->f_flags & OMIT_SYNC))
	(void) fsync (f->f_file);
      break;

    case F_USERS:
    case F_WALL:
      f->f_time = now;
      dbg_printf ("\n");
      v->iov_base = (char *)"\r\n";
      v->iov_len = 2;
      wallmsg (f, iov);
      break;
    }

  if (f->f_type != F_FORW_UNKN)
    f->f_prevcount = 0;
}

/* Write the specified message to either the entire world, or a list
   of approved users.  */
void
wallmsg (struct filed *f, struct iovec *iov)
{
  static int reenter;	/* avoid calling ourselves */
  UTMP *utp;
  int i;
  char *p;
  char line[sizeof(utp->ut_line) + 1];

  if (reenter++)
    return;

  SETUTENT();

  while ((utp = GETUTENT ()) != NULL)
    {
      /* We only want interrested to send to actual
	 process, USER_PROCESS where somebody might listen. */
      if (utp->ut_user[0] == '\0'
	  || utp->ut_line[0] == '\0'
	  || utp->ut_type != USER_PROCESS)
	continue;

      strncpy (line, utp->ut_line, sizeof (utp->ut_line));
      line[sizeof (utp->ut_line)] = '\0';
      if (f->f_type == F_WALL)
	{
	  /* Note we're using our own version of ttymsg
	     which does a double fork () to not have
	     zombies.  No need to waitpid().  */
	  p = ttymsg (iov, IOVCNT, line, TTYMSGTIME);
	  if (p != NULL)
	    {
	      errno = 0;	/* Already in message. */
	      logerror (p);
	    }
	  continue;
	}
      /* Should we send the message to this user? */
      for (i = 0; i < f->f_un.f_user.f_nusers; i++)
	if (!strncmp (f->f_un.f_user.f_unames[i], utp->ut_user,
		      sizeof (utp->ut_user)))
	  {
	    p = ttymsg (iov, IOVCNT, line, TTYMSGTIME);
	    if (p != NULL)
	      {
		errno = 0;	/* Already in message. */
		logerror (p);
	      }
	    break;
	  }
    }
  ENDUTENT ();
  reenter = 0;
}

/* Return a printable representation of a host address.  */
const char *
cvthname (struct sockaddr_in *f)
{
  struct hostent *hp;
  char *p;

  dbg_printf ("cvthname(%s)\n", inet_ntoa (f->sin_addr));

  if (f->sin_family != AF_INET)
    {
      dbg_printf ("Malformed from address.\n");
      return "???";
    }
  hp = gethostbyaddr ((char *) &f->sin_addr,
		      sizeof (struct in_addr), f->sin_family);
  if (hp == 0)
    {
      dbg_printf ("Host name for your address (%s) unknown.\n",
		  inet_ntoa (f->sin_addr));
      return inet_ntoa (f->sin_addr);
    }

  p = strchr (hp->h_name, '.');
  if (p != NULL)
    {
      if (strcasecmp (p + 1, LocalDomain) == 0)
	*p = '\0';
      else
	{
	  int count;

	  if (StripDomains)
	    {
	      count=0;
	      while (StripDomains[count])
		{
		  if (strcasecmp (p + 1, StripDomains[count]) == 0)
		    {
		      *p = '\0';
		      return hp->h_name;
		    }
		  count++;
		}
	    }
	  if (LocalHosts)
	    {
	      count=0;
	      while (LocalHosts[count])
		{
		  if (strcasecmp (hp->h_name, LocalHosts[count]) == 0)
		    {
		      *p = '\0';
		      return hp->h_name;
		    }
		  count++;
		}
	    }
	}
    }
  return hp->h_name;
}

void
domark (int signo)
{
  struct filed *f;

  (void)signo; /* Ignored.  */
  now = time ((time_t *) NULL);
  if (MarkInterval > 0)
    {
      MarkSeq += TIMERINTVL;
      if (MarkSeq >= MarkInterval)
	{
	  logmsg (LOG_INFO, "-- MARK --", LocalHostName, ADDDATE|MARK);
	  MarkSeq = 0;
	}
    }

  for (f = Files; f; f = f->f_next)
    {
      if (f->f_prevcount && now >= REPEATTIME(f))
	{
	  dbg_printf ("flush %s: repeated %d times, %d sec.\n",
		      TypeNames[f->f_type], f->f_prevcount,
		      repeatinterval[f->f_repeatcount]);
	  fprintlog (f, LocalHostName, 0, (char *)NULL);
	  BACKOFF (f);
	}
    }
  (void) alarm (TIMERINTVL);
}

/* Print syslogd errors some place.  */
void
logerror (const char *type)
{
  char buf[100];

  if (errno)
    (void) snprintf (buf, sizeof (buf),
		     "syslogd: %s: %s", type, strerror(errno));
  else
    (void) snprintf (buf, sizeof (buf), "syslogd: %s", type);
  errno = 0;
  dbg_printf ("%s\n", buf);
  logmsg (LOG_SYSLOG | LOG_ERR, buf, LocalHostName, ADDDATE);
}

void
die (int signo)
{
  struct filed *f;
  int was_initialized = Initialized;
  char buf[100];
  size_t i;

  Initialized = 0;	/* Don't log SIGCHLDs. */
  for (f = Files; f != NULL; f = f->f_next)
    {
      /* Flush any pending output.  */
      if (f->f_prevcount)
	fprintlog (f, LocalHostName, 0, (char *) NULL);
    }
  Initialized = was_initialized;
  if (signo)
    {
      dbg_printf ("%s: exiting on signal %d\n", __progname, signo);
      snprintf (buf, sizeof (buf), "exiting on signal %d", signo);
      errno = 0;
      logerror (buf);
    }

  if (fklog >= 0)
    close (fklog);

  for (i = 0; i < nfunix; i++)
    if (funix[i].fd >= 0)
      {
	close (funix[i].fd);
	if (funix[i].name)
	  (void) unlink (funix[i].name);
      }

  if (finet >= 0)
    close (finet);

  exit (0);
}

/* INIT -- Initialize syslogd from configuration table.  */
void
init (int signo)
{
  FILE *cf;
  struct filed *f, *next, **nextp;
  char *p;
#ifndef LINE_MAX
#define LINE_MAX 2048
#endif
  size_t line_max = LINE_MAX;
  char *cbuf;
  char *cline;
  struct servent *sp;

  (void) signo;  /* Ignored.  */
  dbg_printf ("init\n");
  sp = getservbyname ("syslog", "udp");
  if (sp == NULL)
    {
      errno = 0;
      logerror ("network logging disabled (syslog/udp service unknown).");
      logerror ("see syslogd(8) for details of whether and how to enable it.");
      return;
    }
  LogPort = sp->s_port;

  /* Close all open log files.  */
  Initialized = 0;
  for (f = Files; f != NULL; f = next)
    {
      /* Flush any pending output.  */
      if (f->f_prevcount)
	fprintlog (f, LocalHostName, 0, (char *)NULL);

      switch (f->f_type)
	{
	case F_FILE:
	case F_TTY:
	case F_CONSOLE:
	case F_PIPE:
	  (void) close (f->f_file);
	  break;
	}
      next = f->f_next;
      free (f);
    }
  Files = NULL;
  nextp = &Files;

  facilities_seen = 0;

  /* Open the configuration file.  */
  cf = fopen (ConfFile, "r");
  if (cf == NULL)
    {
      dbg_printf ("cannot open %s\n", ConfFile);
      *nextp = (struct filed *) calloc (1, sizeof(*f));
      cfline ("*.ERR\t" PATH_CONSOLE, *nextp);
      (*nextp)->f_next = (struct filed *) calloc (1, sizeof(*f));
      cfline ("*.PANIC\t*", (*nextp)->f_next);
      Initialized = 1;
      return;
    }

  /* Foreach line in the conf table, open that file.  */
  f = NULL;

  /* Allocate a buffer for line parsing.  */
  cbuf = malloc (line_max);
  if (cbuf == NULL)
    {
      /* There is no gracefull recovery here.  */
      dbg_printf ("cannot allocate space for configuration\n");
      return;
    }
  cline = cbuf;

  /* Line parsing :
     - skip comments,
     - strip off trailing spaces,
     - skip empty lines,
     - glob leading spaces,
     - readjust buffer if line is too big,
     - deal with continuation lines, last char is '\' .  */
  while (fgets (cline, line_max - (cline - cbuf), cf) != NULL)
    {
      size_t len = strlen (cline);

      /* No newline ? then line is too big for the buffer readjust.  */
      if (memchr (cline, '\n', len) == NULL)
	{
	  char *tmp;
	  tmp = realloc (cbuf, line_max * 2);
	  if (tmp == NULL)
	    {
	      /* Sigh ...  */
	      dbg_printf ("cannot allocate space configuration\n");
	      free (cbuf);
	      return;
	    }
	  else
	    cbuf = tmp;
	  line_max *= 2;
	  strcpy (cbuf, cline);
	  cline += len;
	  continue;
	}
      else
	cline = cbuf;

      /* Glob the leading spaces.  */
      for (p = cline; isspace (*p); ++p)
	;

      /* Skip comments and empty line.  */
      if (*p == '\0' || *p == '#')
	continue;

      strcpy (cline, p);

      /* Cut the trailing spaces.  */
      for (p = strchr (cline, '\0'); isspace (*--p);)
	;

      /* if '\', indicates continuation on the next line.  */
      if (*p == '\\')
	{
	  *p = '\0';
	  cline = p;
	  continue;
	}

      *++p = '\0';

      /* Send the line for more parsing.  */
      f = (struct filed *) calloc (1, sizeof (*f));
      *nextp = f;
      nextp = &f->f_next;
      cfline (cbuf, f);
    }

  /* Close the configuration file.  */
  (void) fclose (cf);
  free (cbuf);

  Initialized = 1;

  if (Debug)
    {
      for (f = Files; f; f = f->f_next)
	{
	  int i;
	  for (i = 0; i <= LOG_NFACILITIES; i++)
	    if (f->f_pmask[i] == INTERNAL_NOPRI)
	      dbg_printf(" X ");
	    else
	      dbg_printf("%2x ", f->f_pmask[i]);
	  dbg_printf("%s: ", TypeNames[f->f_type]);
	  switch (f->f_type)
	    {
	    case F_FILE:
	    case F_TTY:
	    case F_CONSOLE:
	    case F_PIPE:
	      dbg_printf ("%s", f->f_un.f_fname);
	      break;

	    case F_FORW:
	    case F_FORW_SUSP:
	    case F_FORW_UNKN:
	      dbg_printf ("%s", f->f_un.f_forw.f_hname);
	      break;

	    case F_USERS:
	      for (i = 0; i < f->f_un.f_user.f_nusers; i++)
		dbg_printf ("%s, ", f->f_un.f_user.f_unames[i]);
	      break;
	    }
	  dbg_printf ("\n");
	}
    }

  if (AcceptRemote)
    logmsg (LOG_SYSLOG | LOG_INFO, "syslogd (" PACKAGE_NAME \
	    " " PACKAGE_VERSION "): restart (remote reception)",
	    LocalHostName, ADDDATE);
  else
    logmsg (LOG_SYSLOG | LOG_INFO, "syslogd (" PACKAGE_NAME \
	    " " PACKAGE_VERSION "): restart", LocalHostName, ADDDATE);
  dbg_printf ("syslogd: restarted\n");
}

/* Crack a configuration file line.  */
void
cfline (const char *line, struct filed *f)
{
  struct hostent *hp;
  int i, pri, negate_pri, excl_pri;
  unsigned int pri_set, pri_clear;
  char *bp;
  const char *p, *q;
  char buf[MAXLINE], ebuf[200];

  dbg_printf ("cfline(%s)\n", line);

  errno = 0;	/* keep strerror() stuff out of logerror messages */

  /* Clear out file entry.  */
  memset (f, 0, sizeof (*f));
  for (i = 0; i <= LOG_NFACILITIES; i++)
    {
      f->f_pmask[i] = INTERNAL_NOPRI;
      f->f_flags = 0;
    }

  /* Scan through the list of selectors.  */
  for (p = line; *p && *p != '\t' && *p != ' ';)
    {

      /* Find the end of this facility name list.  */
      for (q = p; *q && *q != '\t' && *q++ != '.'; )
	continue;

      /* Collect priority name.  */
      for (bp = buf; *q && ! strchr ("\t ,;", *q); )
	*bp++ = *q++;
      *bp = '\0';

      /* Skip cruft.  */
      while (strchr(",;", *q))
	q++;

      bp = buf;
      negate_pri = excl_pri = 0;

      while (*bp == '!' || *bp == '=')
	switch (*bp++)
	  {
	  case '!':
	    negate_pri = 1;
	    break;
	  case '=':
	    excl_pri = 1;
	    break;
	  }

      /* Decode priority name and set up bit masks.  */
      if (*bp == '*')
	{
	  pri_clear = INTERNAL_NOPRI;
	  pri_set = LOG_UPTO (LOG_PRIMASK);
	}
      else
	{
	  pri = decode (bp, prioritynames);
	  if (pri < 0)
	    {
	      snprintf (ebuf, sizeof (ebuf),
			"unknown priority name \"%s\"", bp);
	      logerror (ebuf);
	      return;
	    }
	  pri_clear = 0;
	  pri_set = excl_pri ? LOG_MASK (pri) : LOG_UPTO (pri);
	}
      if (negate_pri)
	{
	  unsigned int exchange = pri_set;
	  pri_set = pri_clear;
	  pri_clear = exchange;
	}

      /* Scan facilities.  */
      while (*p && !strchr ("\t .;", *p))
	{
	  for (bp = buf; *p && ! strchr ("\t ,;.", *p); )
	    *bp++ = *p++;
	  *bp = '\0';
	  if (*buf == '*')
	    for (i = 0; i <= LOG_NFACILITIES; i++)
	      {
		/* make "**" act as a wildcard only for facilities not 
		 * specified elsewhere
		 */
		if (buf[1] == '*' && ((1 << i) & facilities_seen))
		  continue;
		f->f_pmask[i] &= ~pri_clear;
		f->f_pmask[i] |= pri_set;
	      }
	  else
	    {
	      i = decode (buf, facilitynames);
	      facilities_seen |= (1 << LOG_FAC(i));

	      if (i < 0)
		{
		  snprintf (ebuf, sizeof (ebuf),
			    "unknown facility name \"%s\"", buf);
		  logerror (ebuf);
		  return;
		}
	      f->f_pmask[LOG_FAC(i)] &= ~pri_clear;
	      f->f_pmask[LOG_FAC(i)] |= pri_set;
	    }
	  while (*p == ',' || *p == ' ')
	    p++;
	}
      p = q;
    }

  /* Skip to action part.  */
  while (*p == '\t' || *p == ' ')
    p++;

  if (*p == '-')
    {
      f->f_flags |= OMIT_SYNC;
      p++;
    }

  switch (*p)
    {
    case '@':
      f->f_un.f_forw.f_hname = strdup (++p);
      hp = gethostbyname (p);
      if (hp == NULL)
	{
	  f->f_type = F_FORW_UNKN;
	  f->f_prevcount = INET_RETRY_MAX;
	  f->f_time = time ( (time_t *)0 );
	}
      else
	f->f_type = F_FORW;
      memset (&f->f_un.f_forw.f_addr, 0,
	      sizeof(f->f_un.f_forw.f_addr));
      f->f_un.f_forw.f_addr.sin_family = AF_INET;
      f->f_un.f_forw.f_addr.sin_port = LogPort;
      if (f->f_type == F_FORW)
	memcpy (&f->f_un.f_forw.f_addr.sin_addr,
		hp->h_addr, hp->h_length);
      break;

    case '|':
      f->f_un.f_fname = strdup (p);
      if ((f->f_file = open (++p, O_RDWR|O_NONBLOCK)) < 0)
	{
	  f->f_type = F_UNUSED;
	  logerror (p);
	  break;
	}
      if (strcmp (p, ctty) == 0)
	f->f_type = F_CONSOLE;
      else if (isatty (f->f_file))
	f->f_type = F_TTY;
      else
	f->f_type = F_PIPE;
      break;

    case '/':
      f->f_un.f_fname = strdup (p);
      if ((f->f_file = open (p, O_WRONLY|O_APPEND|O_CREAT, 0644)) < 0)
	{
	  f->f_type = F_UNUSED;
	  logerror (p);
	  break;
	}
      if (strcmp (p, ctty) == 0)
	f->f_type = F_CONSOLE;
      else if (isatty (f->f_file))
	f->f_type = F_TTY;
      else
	f->f_type = F_FILE;
      break;

    case '*':
      f->f_type = F_WALL;
      break;

    default:
      f->f_un.f_user.f_nusers = 1;
      for (q = p; *q; q++)
	if (*q == ',')
	  f->f_un.f_user.f_nusers++;
      f->f_un.f_user.f_unames =
	(char **) malloc (f->f_un.f_user.f_nusers * sizeof (char *));
      for (i = 0; *p; i++)
	{
	  for (q = p; *q && *q != ','; )
	    q++;
	  f->f_un.f_user.f_unames[i] = malloc (q - p + 1);
	  if (f->f_un.f_user.f_unames[i])
	    {
	      strncpy (f->f_un.f_user.f_unames[i], p, q - p);
	      f->f_un.f_user.f_unames[i][q - p] = '\0';
	    }
	  while (*q == ',' || *q == ' ')
	    q++;
	  p = q;
	}
      f->f_type = F_USERS;
      break;
    }
}

/* Decode a symbolic name to a numeric value.  */
int
decode (const char *name, CODE *codetab)
{
  CODE *c;

  if (isdigit (*name))
    return atoi (name);

  for (c = codetab; c->c_name; c++)
    if (!strcasecmp (name, c->c_name))
      return c->c_val;

  return -1;
}

void
dbg_toggle(int signo)
{
  int dbg_save = dbg_output;

  (void) signo;  /* Ignored.  */
  dbg_output = 1;
  dbg_printf ("Switching dbg_output to %s.\n",
	      dbg_save == 0 ? "true" : "false");
  dbg_output = (dbg_save == 0) ? 1 : 0;
}

/* Ansi2knr will always change ... to va_list va_dcl */
static void
dbg_printf (const char *fmt, ...)
{
  va_list ap;

  if (!(Debug && dbg_output))
    return;

#if defined(HAVE_STDARG_H) && defined(__STDC__) && __STDC__
  va_start (ap, fmt);
#else
  va_start (ap);
#endif

  va_start (ap, fmt);
  vfprintf (stdout, fmt, ap);
  va_end (ap);

  fflush (stdout);
}

/* The following function is resposible for handling a SIGHUP signal.
   Since we are now doing mallocs/free as part of init we had better
   not being doing this during a signal handler.  Instead we simply
   set a flag variable which will tell the main loop to go through a
   restart.  */
void
trigger_restart (int signo)
{
  (void) signo;  /* Ignored.  */
  restart = 1;
}

