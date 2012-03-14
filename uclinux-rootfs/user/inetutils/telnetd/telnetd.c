/* Copyright (C) 1998,2001, 2002 Free Software Foundation, Inc.

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
#include <getopt.h>
#ifdef HAVE_SYS_UTSNAME_H
# include <sys/utsname.h>
#endif

static char short_options[] = "a:D::d:E:HhLl::nS:Uu:VX:";

static struct option long_options[] =
{
  /* Help options */
  {"version", no_argument, NULL, 'V'},
  {"license", no_argument, NULL, 'L'},
  {"help",    no_argument, NULL, 'H'},
  /* Common options */
  {"authmode", required_argument, NULL, 'a'},
  {"debug", optional_argument, NULL, 'D'},
  {"exec-login", required_argument, NULL, 'E'},
  {"no-hostinfo", no_argument, NULL, 'h'},
  {"linemode", optional_argument, NULL, 'l'},
  {"no-keepalive", no_argument, NULL, 'n'},
  {"reverse-lookup", no_argument, NULL, 'U'},
  {"disable-auth-type", required_argument, NULL, 'X'},
  {NULL, 0, NULL, 0}
};

static void telnetd_version P((void));
static void telnetd_license P((void));
static void telnetd_help P((void));
static void parse_authmode P((char *str));
static void parse_linemode P((char *str));
static void parse_debug_level P((char *str));
static void telnetd_setup P((int fd));
static int telnetd_run P((void));
static void print_hostinfo P((void));

/* Template command line for invoking login program */

char *login_invocation =
#ifdef SOLARIS
"/bin/login -h %h %?T{TERM=%T}{-} %?u{%?a{-f }-- %u}"
#else
"/bin/login -p -h %h %?u{-f %u}"
#endif
;

int keepalive = 1;      /* Should the TCP keepalive bit be set */
int reverse_lookup = 0; /* Reject connects from hosts which IP numbers
			   cannot be reverse mapped to their hostnames */
int alwayslinemode;     /* Always set the linemode (1) */ 
int lmodetype;          /* Type of linemode (2) */
int hostinfo = 1;       /* Print the host-specific information before
			   login */

int auth_level = 0;     /* Authentication level */

int debug_level[debug_max_mode]; /* Debugging levels */
int debug_tcp = 0;               /* Should the SO_DEBUG be set? */

int net;      /* Network connection socket */
int pty;      /* PTY master descriptor */
char *remote_hostname;
char *local_hostname;
char *user_name;
char line[256];

char	options[256];
char	do_dont_resp[256];
char	will_wont_resp[256];
int	linemode;	/* linemode on/off */
int	uselinemode;	/* what linemode to use (on/off) */
int	editmode;	/* edit modes in use */
int	useeditmode;	/* edit modes to use */
int	alwayslinemode;	/* command line option */
int	lmodetype;	/* Client support for linemode */
int	flowmode;	/* current flow control state */
int	restartany;	/* restart output on any character state */
int	diagnostic;	/* telnet diagnostic capabilities */
#if defined(AUTHENTICATION)
int	auth_level;
int     autologin;
#endif

slcfun	slctab[NSLC + 1];	/* slc mapping table */

char	*terminaltype;

int	SYNCHing;		/* we are in TELNET SYNCH mode */
struct telnetd_clocks clocks;

int
main (int argc, char **argv)
{
  int c;
  while ((c = getopt_long (argc, argv, short_options, long_options, NULL))
	 != EOF)
    {
      switch (c)
	{
	case 'V': 
	  telnetd_version ();
	  exit (0);
	  
	case 'L':
	  telnetd_license ();
	  exit (0);
	  
	case 'H':
	  telnetd_help ();
	  exit (0);
	  
	case 'a':
	  parse_authmode (optarg);
	  break;
	  
	case 'D':
	  parse_debug_level (optarg);
	  break;
	  
	case 'E':
	  login_invocation = optarg;
	  break;
	  
	case 'h':
	  hostinfo = 0;
	  break;
	  
	case 'l':
	  parse_linemode (optarg);
	  break;
	  
	case 'n':
	  keepalive = 0;
	  break;
	  
	case 'U':
	  reverse_lookup = 1;
	  break;
	  
#ifdef	AUTHENTICATION
	case 'X':
	  auth_disable_name (optarg);
	  break;
#endif
	default:
	  fprintf (stderr, "telnetd: unknown command line option: %c\n", c);
	  exit (1);
	}
    }

  if (argc != optind)
    {
      fprintf (stderr, "telnetd: junk arguments in the command line\n");
      exit (1);
    }
  
  openlog ("telnetd", LOG_PID | LOG_ODELAY, LOG_DAEMON);
  telnetd_setup (0);
  return telnetd_run ();
}

void
parse_linemode (char *str)
{
  if (!str)
    alwayslinemode = 1;
  else if (strcmp (str, "nokludge") == 0)
    lmodetype = NO_AUTOKLUDGE;
  else
    fprintf (stderr,
	     "telnetd: invalid argument to --linemode\n");
}

void
parse_authmode (char *str)
{
  if (strcasecmp (str, "none") == 0) 
    auth_level = 0;
  else if (strcasecmp (str, "other") == 0) 
    auth_level = AUTH_OTHER;
  else if (strcasecmp (str, "user") == 0) 
    auth_level = AUTH_USER;
  else if (strcasecmp (str, "valid") == 0) 
    auth_level = AUTH_VALID;
  else if (strcasecmp (str, "off") == 0) 
    auth_level = -1;
  else 
    fprintf (stderr,
	     "telnetd: unknown authorization level for -a\n");
}

static struct {
  char *name;
  int modnum;
} debug_mode[debug_max_mode] = {
  "options", debug_options,
  "report",  debug_report,
  "netdata", debug_net_data,
  "ptydata", debug_pty_data,
  "auth", debug_auth,
};

void
parse_debug_level (char *str)
{
  int i;
  char *tok;

  if (!str)
    {
      for (i = 0; i < debug_max_mode; i++) 
	debug_level[ debug_mode[i].modnum ] = MAX_DEBUG_LEVEL;
      return;
    }
  
  for (tok = strtok (str, ","); tok; tok = strtok (NULL, ","))
    {
      int length, level;
      char *p;

      if (strcmp (tok, "tcp") == 0)
	{
	  debug_tcp = 1;
	  continue;
	}

      p = strchr (tok, '=');
      if (p)
	{
	  length = p - tok;
	  level  = strtoul (p+1, NULL, 0);
	}
      else
	{
	  length = strlen (tok);
	  level  = MAX_DEBUG_LEVEL;
	}

      for (i = 0; i < debug_max_mode; i++) 
	if (strncmp (debug_mode[i].name, tok, length) == 0)
	  {
	    debug_level[ debug_mode[i].modnum ] = level;
	    break;
	  }

      if (i == debug_max_mode)
	fprintf (stderr, "telnetd: unknown debug mode: %s", tok);
    }
}


typedef unsigned int ip_addr_t; /*FIXME*/

void
telnetd_setup (int fd)
{
  struct sockaddr_in saddr;
  int true = 1;
  int len;
  struct hostent *hp;
  char uname[256]; /*FIXME*/
  int level;
 
  len = sizeof (saddr);
  if (getpeername(fd, (struct sockaddr *)&saddr, &len) < 0)
    {
      syslog (LOG_ERR, "getpeername: %m");
      exit (1);
    }

  syslog (LOG_INFO, "Connect from %s", inet_ntoa (saddr.sin_addr));
  
  hp = gethostbyaddr ((char*)&saddr.sin_addr.s_addr,
		      sizeof (saddr.sin_addr.s_addr), AF_INET);
  if (reverse_lookup)
    {
      char **ap;
      
      if (!hp)
	{
	  syslog (LOG_AUTH|LOG_NOTICE,
		  "Can't resolve %s: %s",
		  inet_ntoa (saddr.sin_addr),
		  hstrerror (h_errno));
	  fatal (fd, "Cannot resolve address.");
	}

      remote_hostname = xstrdup (hp->h_name);

      hp = gethostbyname (remote_hostname);
      if (!hp)
	{
	  syslog (LOG_AUTH|LOG_NOTICE,
		  "Forward resolve of %s failed: %s",
		  remote_hostname, hstrerror (h_errno));
	  fatal (fd, "Cannot resolve address.");
	}

      for (ap = hp->h_addr_list; *ap; ap++)
	if (*(ip_addr_t*)ap == saddr.sin_addr.s_addr)
	  break;

      if (ap == NULL)
	{
	  syslog (LOG_AUTH|LOG_NOTICE,
		  "None of addresses of %s matched %s",
		  remote_hostname,
		  inet_ntoa (saddr.sin_addr));
	  exit (0);
	}
    }
  else
    {
      if (hp)
	remote_hostname = xstrdup (hp->h_name);
      else
	remote_hostname = xstrdup (inet_ntoa (saddr.sin_addr));
    }
  
  /* Set socket options */

  if (keepalive
      && setsockopt (fd, SOL_SOCKET, SO_KEEPALIVE,
		     (char *)&true, sizeof (true)) < 0)
    syslog (LOG_WARNING, "setsockopt (SO_KEEPALIVE): %m");

  if (debug_tcp
      && setsockopt (fd, SOL_SOCKET, SO_DEBUG,
		     (char *)&true, sizeof (true)) < 0)
    syslog (LOG_WARNING, "setsockopt (SO_DEBUG): %m");

  net = fd;

  local_hostname = localhost ();
#if defined(AUTHENTICATION) || defined(ENCRYPTION)
  auth_encrypt_init (remote_hostname, local_hostname, "TELNETD", 1);
#endif

  io_setup ();
  
  /* get terminal type. */
  uname[0] = 0;
  level = getterminaltype (uname);
  setenv ("TERM", terminaltype ? terminaltype : "network", 1);
  if (uname[0])
    user_name = xstrdup (uname);
  pty = startslave (remote_hostname, level, user_name);

#ifndef	HAVE_STREAMSPTY
  /* Turn on packet mode */
  ioctl (pty, TIOCPKT, (char *)&true);
#endif
  ioctl (pty, FIONBIO, (char *)&true);
  ioctl (net, FIONBIO, (char *)&true);
  
#if defined(SO_OOBINLINE)
  setsockopt (net, SOL_SOCKET, SO_OOBINLINE, (char *)&true, sizeof true);
#endif	

#ifdef SIGTSTP
  signal (SIGTSTP, SIG_IGN);
#endif
#ifdef	SIGTTOU
  signal (SIGTTOU, SIG_IGN);
#endif

  signal (SIGCHLD, cleanup);
}

int
telnetd_run ()
{
  int nfd;
  
  get_slc_defaults ();

  if (my_state_is_wont(TELOPT_SGA))
    send_will(TELOPT_SGA, 1);

  /* Old BSD 4.2 clients are unable to deal with TCP out-of-band data.
     To find out, we send out a "DO ECHO". If the remote side is
     a BSD 4.2 it will answer "WILL ECHO". See the response processing
     below. */
  send_do(TELOPT_ECHO, 1);
  if (his_state_is_wont (TELOPT_LINEMODE))
    {
      /* Query the peer for linemode support by trying to negotiate
	 the linemode option. */
      linemode = 0;
      editmode = 0;
      send_do(TELOPT_LINEMODE, 1);  /* send do linemode */
    }

  send_do (TELOPT_NAWS, 1);
  send_will (TELOPT_STATUS, 1);
  flowmode = 1;		/* default flow control state */
  restartany = -1;	/* uninitialized... */
  send_do (TELOPT_LFLOW, 1);

  /* Wait for a response from the DO ECHO. Reportedly, some broken
     clients might not respond to it. To work around this, we wait
     for a response to NAWS, which should have been processed after
     DO ECHO (most dumb telnets respond with WONT for a DO that
     they don't understand).  
       On the other hand, the client might have sent WILL NAWS as
     part of its startup code, in this case it surely should have
     answered our DO ECHO, so the second loop is waiting for
     the ECHO to settle down.  */
  ttloop (his_will_wont_is_changing (TELOPT_NAWS));

  if (his_want_state_is_will (TELOPT_ECHO)
      && his_state_is_will (TELOPT_NAWS)) 
    ttloop (his_will_wont_is_changing (TELOPT_ECHO));

  /* If the remote client is badly broken and did not respond to our
     DO ECHO, we simulate the receipt of a will echo. This will also
     send a WONT ECHO to the client, since we assume that the client
     failed to respond because it believes that it is already in DO ECHO
     mode, which we do not want. */
  
  if (his_want_state_is_will (TELOPT_ECHO))
    {
      DEBUG(debug_options, 1, debug_output_data ("td: simulating recv\r\n"));
      willoption (TELOPT_ECHO);
    }

  /* Turn on our echo */
  if (my_state_is_wont (TELOPT_ECHO))
    send_will (TELOPT_ECHO, 1);

  /* Continuing line mode support.  If client does not support
     real linemode, attempt to negotiate kludge linemode by sending
     the do timing mark sequence. */
  if (lmodetype < REAL_LINEMODE)
    send_do (TELOPT_TM, 1);

  /* Pick up anything received during the negotiations */
  telrcv ();

  if (hostinfo)
    print_hostinfo ();
    
  init_termbuf ();
  localstat ();

  DEBUG(debug_report, 1,
	debug_output_data ("td: Entering processing loop\r\n"));

  nfd = ((net > pty) ? net : pty) + 1;

  for (;;)
    {
      fd_set ibits, obits, xbits;
      register int c;

      if (net_input_level () < 0 && pty_input_level () < 0)
	break;

      FD_ZERO(&ibits);
      FD_ZERO(&obits);
      FD_ZERO(&xbits);

      /* Never look for input if there's still stuff in the corresponding
	 output buffer */
      if (net_output_level () || pty_input_level () > 0)  
	FD_SET (net, &obits);
      else 
	FD_SET(pty, &ibits);

      if (pty_output_level () || net_input_level () > 0) 
	FD_SET(pty, &obits);
      else 
	FD_SET(net, &ibits);
	
      if (!SYNCHing) 
	FD_SET(net, &xbits);

      if ((c = select (nfd, &ibits, &obits, &xbits, NULL)) <= 0)
	{
	  if (c == -1 && errno == EINTR) 
	    continue;
	  sleep(5);
	  continue;
	}

      if (FD_ISSET(net, &xbits)) 
	SYNCHing = 1;

      if (FD_ISSET(net, &ibits))
	{
	  /* Something to read from the network... */
	  /*FIXME: handle  !defined(SO_OOBINLINE) */
	  net_read ();
	}

      if (FD_ISSET(pty, &ibits))
	{
	  /* Something to read from the pty... */
	  if (pty_read () < 0)
	    break;
	  c = pty_get_char (1); 
#if defined(TIOCPKT_IOCTL)
	  if (c & TIOCPKT_IOCTL)
	    {
	      pty_get_char (0);
	      copy_termbuf ();
	      localstat ();
	    }
#endif
	  if (c & TIOCPKT_FLUSHWRITE)
	    {
	      static char flushdata[] = { IAC, DM };
	      pty_get_char (0);
	      netclear();	/* clear buffer back */
	      net_output_datalen (flushdata, sizeof (flushdata));
	      set_neturg ();
	      DEBUG(debug_options, 1, printoption("td: send IAC", DM));
	    }

	  if (his_state_is_will(TELOPT_LFLOW)
	      && (c & (TIOCPKT_NOSTOP|TIOCPKT_DOSTOP)))
	    {
	      int newflow = c & TIOCPKT_DOSTOP ? 1 : 0;
	      if (newflow != flowmode)
		{
		  net_output_data ("%c%c%c%c%c%c",
				   IAC, SB, TELOPT_LFLOW,
				   flowmode ? LFLOW_ON
				   : LFLOW_OFF,
				   IAC, SE);
		}
	      pty_get_char (0);
	    }

	}

      while (pty_input_level () > 0)
	{
	  if (net_buffer_is_full ())
	    break;
	  c = pty_get_char (0);
	  if (c == IAC)
	    net_output_byte (c);
	  net_output_byte (c);
	  if (c == '\r' && my_state_is_wont (TELOPT_BINARY))
	    {
	      if (pty_input_level () > 0 && pty_get_char (1) == '\n')
		net_output_byte (pty_get_char (0));
	      else
		net_output_byte (0);
	    }
	}

      if (FD_ISSET(net, &obits) && net_output_level () > 0)
	netflush ();
      if (net_input_level () > 0)
	telrcv ();
      
      if (FD_ISSET(pty, &obits) && pty_output_level () > 0)
	ptyflush ();
    }
  cleanup (0);
}

void
print_hostinfo ()
{
  char *im = NULL;
  char *str;
#ifdef HAVE_UNAME
  struct utsname u;
  
  if (uname (&u) == 0)
    {
      im = malloc (strlen (UNAME_IM_PREFIX)
		   + strlen (u.sysname)
		   + 1 + strlen (u.release)
		   + strlen (UNAME_IM_SUFFIX) + 1);
      if (im)
	sprintf (im, "%s%s %s%s",
		 UNAME_IM_PREFIX,
		 u.sysname, u.release,
		 UNAME_IM_SUFFIX);
    }
#endif /* HAVE_UNAME */
  if (!im)
    im = xstrdup ("\r\n\nUNIX (%l) (%t)\r\n\n");

  str = expand_line (im);
  free (im);
  
  DEBUG(debug_pty_data, 1, debug_output_data ("sending %s", str)); 
  pty_input_putback (str, strlen (str));
  free (str);
}

void
telnetd_version ()
{
  printf ("telnetd - %s %s\n", PACKAGE_NAME, PACKAGE_VERSION);
  printf ("Copyright (C) 1998,2001,2002 Free Software Foundation, Inc.\n");
  printf ("%s comes with ABSOLUTELY NO WARRANTY.\n", PACKAGE_NAME);
  printf ("You may redistribute copies of %s\n", PACKAGE_NAME);
  printf ("under the terms of the GNU General Public License.\n");
  printf ("For more information about these matters, ");
  printf ("see the files named COPYING.\n");
}

void
telnetd_license ()
{
  static char license_text[] =
"   This program is free software; you can redistribute it and/or modify\n"
"   it under the terms of the GNU General Public License as published by\n"
"   the Free Software Foundation; either version 2, or (at your option)\n"
"   any later version.\n"
"\n"
"   This program is distributed in the hope that it will be useful,\n"
"   but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
"   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
"   GNU General Public License for more details.\n"
"\n"
"   You should have received a copy of the GNU General Public License\n"
"   along with this program; if not, write to the Free Software\n"
"   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.\n";
  printf ("%s", license_text);
}

void
telnetd_help ()
{
  printf ("\
Usage: telnetd [OPTION]\n\
\n\
Options are:\n\
    -a, --authmode AUTHMODE  specify what mode to use for authentication\n\
    -D, --debug[=LEVEL]      set debugging level\n\
    -E, --exec-login STRING  set program to be executed instead of /bin/login\n\
    -h, --no-hostinfo        do not print host information before login has\n\
                             been completed\n\
    -l, --linemode[=MODE]    set line mode\n\
    -n, --no-keepalive       disable TCP keep-alives\n\
    -U, --reverse-lookup     refuse  connections from addresses that\n\
                             cannot be mapped back into a symbolic name\n\
    -X, --disable-auth-type AUTHTYPE\n\
                             disable the use of given authentication option\n\
Informational options:\n\
    -V, --version         display this help and exit\n\
    -L, --license	  display license and exit\n\
    -H. --help		  output version information and exit\n");
}

int
stop()
{
  int volatile _s=1;

  while (_s)
    _s=_s;
  return 0;
}
