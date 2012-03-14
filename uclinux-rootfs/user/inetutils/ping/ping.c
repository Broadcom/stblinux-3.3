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

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <sys/param.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <sys/time.h>
#include <signal.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
/*#include <netinet/ip_icmp.h>  -- deliberately not including this */
#ifdef HAVE_NETINET_IP_VAR_H
# include <netinet/ip_var.h>
#endif

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>

#include "getopt.h"
#include <icmp.h>
#include <ping.h>
#include <ping_impl.h>

static char short_options[] = "VLhc:dfi:l:np:qRrs:t:v";
static struct option long_options[] = 
{
  /* Help options */
  {"version", no_argument, NULL, 'V'},
  {"license", no_argument, NULL, 'L'},
  {"help",    no_argument, NULL, 'h'},
  /* Common options */
  {"count",   required_argument, NULL, 'c'},
  {"debug",   no_argument, NULL, 'd'},
  {"ignore-routing", no_argument, NULL, 'r'},
  {"size",    required_argument, NULL, 's'},
  {"interval",required_argument, NULL, 'i'},
  {"numeric", no_argument, NULL, 'n'},
  {"verbose", no_argument, NULL, 'v'},
  /* Packet types */
  {"type",    required_argument, NULL, 't'},
  {"echo",    no_argument, NULL, ICMP_ECHO},
  {"timestamp",no_argument, NULL, ICMP_TIMESTAMP},
  {"address", no_argument, NULL, ICMP_ADDRESS},
  {"router",  no_argument, NULL, ICMP_ROUTERDISCOVERY},
  /* echo-specific options */
  {"flood",   no_argument, NULL, 'f'},
  {"preload", required_argument, NULL, 'l'},
  {"pattern", required_argument, NULL, 'p'},
  {"quiet",   no_argument, NULL, 'q'},
  {"route",   no_argument, NULL, 'R'},
  {NULL,      no_argument, NULL, 0}
};

extern int ping_echo __P ((int argc, char **argv));
extern int ping_timestamp __P ((int argc, char **argv));
extern int ping_address __P ((int argc, char **argv));
extern int ping_router __P ((int argc, char **argv));

PING *ping;
u_char *data_buffer;
size_t data_length = PING_DATALEN;
unsigned options;
unsigned long preload = 0;
int (*ping_type) __P ((int argc, char **argv)) = ping_echo;


static void show_usage (void);
static void show_license (void);
static void decode_pattern (const char *text, int *pattern_len, 
		           u_char *pattern_data);
static void decode_type (const char *optarg);
static void init_data_buffer (u_char *pat, int len);
static int send_echo (PING *ping);

int
main (int argc, char **argv)
{
  int c;
  char *p;
  int one = 1;
  u_char pattern[16];
  int pattern_len = 16;
  u_char *patptr = NULL;
  int is_root = getuid () == 0;

  if ((ping = ping_init (ICMP_ECHO, getpid ())) == NULL)
    {
      fprintf (stderr, "can't init ping: %s\n", strerror (errno));
      exit (1);
    }
  ping_set_sockopt (ping, SO_BROADCAST, (char *)&one, sizeof (one));

  /* Reset root privileges */
  setuid (getuid ());

  /* Parse command line */
  while ((c = getopt_long (argc, argv, short_options, long_options, NULL))
	 != EOF)
    {
      switch (c)
	{
	case 'V':
	  printf ("ping - %s %s\n", PACKAGE_NAME, PACKAGE_VERSION);
	  printf ("Copyright (C) 1998,2001 Free Software Foundation, Inc.\n");
	  printf ("%s comes with ABSOLUTELY NO WARRANTY.\n", PACKAGE_NAME);
	  printf ("You may redistribute copies of %s\n", PACKAGE_NAME);
	  printf ("under the terms of the GNU General Public License.\n");
	  printf ("For more information about these matters, ");
	  printf ("see the files named COPYING.\n");
	  exit (0);
	  break;
	  
	case 'L':
	  show_license ();
	  exit (0);
	  
	case 'h':
	  show_usage ();
	  exit (0);
	  break;
	  
	case 'c':
	  ping_set_count (ping, atoi (optarg));
	  break;
	  
	case 'd':
	  ping_set_sockopt (ping, SO_DEBUG, &one, sizeof (one));
	  break;
	  
	case 'r':
	  ping_set_sockopt (ping, SO_DONTROUTE, &one, sizeof (one));
	  break;
	  
	case 'i':
	  options |= OPT_INTERVAL;
	  ping_set_interval (ping, atoi (optarg));
	  break;
	  
	case 'p':
	  decode_pattern (optarg, &pattern_len, pattern);
	  patptr = pattern;
	  break;
	  
 	case 's':
	  data_length = atoi (optarg);
 	  break;
	  
	case 'n':
	  options |= OPT_NUMERIC;
	  break;
	  
	case 'q':
	  options |= OPT_QUIET;
	  break;
	  
	case 'R':
	  options |= OPT_RROUTE;
	  break;
	  
        case 'v':
	  options |= OPT_VERBOSE;
	  break;
	  
	case 'l':
	  if (!is_root)
	    {
	      fprintf (stderr, "ping: option not allowed: --preload\n");
	      exit (1);
	    }
	  preload = strtoul (optarg, &p, 0);
	  if (*p || preload > INT_MAX)
	    {
	      fprintf (stderr, "ping: invalid preload value (%s)\n", optarg);
	      exit (1);
	    }
	  break;

	case 'f':
	  if (!is_root)
	    {
	      fprintf (stderr, "ping: option not allowed: --flood\n");
	      exit (1);
	    }
	  options |= OPT_FLOOD;
	  setbuf (stdout, (char *)NULL);
	  break;

	case 't':
	  decode_type (optarg);
	  break;

	case ICMP_ECHO:
	  decode_type ("echo");
	  break;
	  
	case ICMP_TIMESTAMP:
	  decode_type ("timestamp");
	  break;
	  
	case ICMP_ADDRESS:
	  decode_type ("address");
	  break;
	  
	case ICMP_ROUTERDISCOVERY:
	  decode_type ("router");
	  break;
	  
	default:
	  fprintf (stderr, "%c: not implemented\n", c);
	  exit (1);
	}
    }

  argc -= optind;
  argv += optind;
  if (argc == 0) 
    {
      show_usage ();
      exit (0);
    }

  init_data_buffer (patptr, pattern_len);

  return (*ping_type)(argc, argv);
}

void
init_data_buffer (u_char *pat, int len)
{
  int i = 0;
  u_char *p;

  if (data_length == 0)
    return;
  data_buffer = malloc (data_length);
  if (!data_buffer)
    {
      fprintf (stderr, "ping: out of memory\n");
      exit (1);
    }
  if (pat)
    {
      for (p = data_buffer; p < data_buffer + data_length; p++)
	{
	  *p = pat[i];
	  if (i++ >= len)
	    i = 0;
	}
    }
  else
    {
      for (i = 0; i < data_length; i++)
	data_buffer[i] = i;
    }
}
  

void
decode_type (const char *optarg)
{
  if (strcasecmp (optarg, "echo") == 0)
    ping_type = ping_echo;
  else if (strcasecmp (optarg, "timestamp") == 0)
    ping_type = ping_timestamp;
  else if (strcasecmp (optarg, "address") == 0)
    ping_type = ping_address;
#if 0  
  else if (strcasecmp (optarg, "router") == 0)
    ping_type = ping_router;
#endif  
  else
    {
      fprintf (stderr, "unsupported packet type: %s\n", optarg);
      exit (1);
    }
}

void
decode_pattern (const char *text, int *pattern_len, u_char *pattern_data)
{
  int i, c, off;

  for (i = 0; *text && i < *pattern_len; i++)
    {
      if (sscanf (text, "%2x%n", &c, &off) != 1)
	{
	  fprintf (stderr, "ping: error in pattern near %s\n", text);
	  exit (1);
	}
      text += off;
    }
  *pattern_len = i;
}

int volatile stop = 0;

RETSIGTYPE
sig_int (int signal)
{
  stop = 1;
}

int
ping_run (PING *ping, int (*finish)())
{
  fd_set fdset;
  int fdmax;
  struct timeval timeout;
  struct timeval last, intvl, now;
  struct timeval *t = NULL;
  int finishing = 0;
  
  signal (SIGINT, sig_int);
  
  fdmax = ping->ping_fd+1;

  while (preload--)      
    send_echo (ping);

  if (options & OPT_FLOOD)
    {
      intvl.tv_sec = 0;
      intvl.tv_usec = 10000;
    }
  else
    {
      intvl.tv_sec = ping->ping_interval;
      intvl.tv_usec = 0;
    }
  
  gettimeofday (&last, NULL);
  send_echo (ping);

  while (!stop)
    {
      int n, len;
      
      FD_ZERO (&fdset);
      FD_SET (ping->ping_fd, &fdset);
      gettimeofday (&now, NULL);
      timeout.tv_sec = last.tv_sec + intvl.tv_sec - now.tv_sec;
      timeout.tv_usec = last.tv_usec + intvl.tv_usec - now.tv_usec;

      while (timeout.tv_usec < 0)
	{
	  timeout.tv_usec += 1000000;
	  timeout.tv_sec--;
	}
      while (timeout.tv_usec >= 1000000)
	{
	  timeout.tv_usec -= 1000000;
	  timeout.tv_sec++;
	}

      if (timeout.tv_sec < 0)
	timeout.tv_sec = timeout.tv_usec = 0;
      
      if ((n = select (fdmax, &fdset, NULL, NULL, &timeout)) < 0)
	{
	  if (errno != EINTR)
	    perror ("ping: select");
	  continue;
	}
      else if (n == 1)
	{
	  len = ping_recv (ping);
	  if (t == 0)
	    {
	      gettimeofday (&now, NULL);
	      t = &now;
	    }
	  if (ping->ping_count && ping->ping_num_recv >= ping->ping_count)
	    break;
	}
      else
	{
	  if (!ping->ping_count || ping->ping_num_recv < ping->ping_count)
	    {
	      send_echo (ping);
	      if (!(options & OPT_QUIET) && options & OPT_FLOOD)
		{
		  putchar ('.');
		}
	    }
	  else if (finishing)
	    break;
	  else
	    {
	      finishing = 1;

	      intvl.tv_sec = MAXWAIT;
	    }
	  gettimeofday (&last, NULL);
	}
    }
  if (finish)
    return (*finish)();
  return 0;
}

int
send_echo (PING *ping)
{
  int off = 0;
  
  if (PING_TIMING (data_length))
    {
      struct timeval tv;
      gettimeofday (&tv, NULL);
      ping_set_data (ping, &tv, 0, sizeof (tv));
      off += sizeof (tv);
    }
  if (data_buffer)
    ping_set_data (ping, data_buffer, off,
		  data_length > PING_HEADER_LEN ?
		    data_length-PING_HEADER_LEN : data_length);
  return ping_xmit (ping);
}

int
ping_finish ()
{
  fflush (stdout);
  printf ("--- %s ping statistics ---\n", ping->ping_hostname);
  printf ("%ld packets transmitted, ", ping->ping_num_xmit);
  printf ("%ld packets received, ", ping->ping_num_recv);
  if (ping->ping_num_rept)
    printf ("+%ld duplicates, ", ping->ping_num_rept);  
  if (ping->ping_num_xmit)
    {
      if (ping->ping_num_recv > ping->ping_num_xmit)
	printf ("-- somebody's printing up packets!");
      else
	printf ("%d%% packet loss",
	       (int) (((ping->ping_num_xmit - ping->ping_num_recv) * 100) /
		      ping->ping_num_xmit));

    }
  printf ("\n");
  return 0;
}

void
show_usage (void)
{
  printf ("\
Usage: ping [OPTION]... [ADDRESS]...\n\
\n\
Informational options:\n\
  -h, --help         display this help and exit\n\
  -L, --license      display license and exit\n\
  -V, --version      output version information and exit\n\
Options controlling ICMP request types:\n\
  --echo             Send ICMP_ECHO requests (default)\n\
  --address          Send ICMP_ADDRESS packets\n\
  --timestamp        Send ICMP_TIMESTAMP packets\n\
  --router           Send ICMP_ROUTERDISCOVERY packets\n\
Options valid for all request types:\n\
  -c, --count N      stop after sending N packets\n\
  -d, --debug        set the SO_DEBUG option\n\
  -i, --interval N   wait N seconds between sending each packet\n\
  -n, --numeric      do not resolve host addresses\n\
  -r, --ignore-routing  send directly to a host on an attached network\n\
  -v, --verbose      verbose output\n\
Options valid for --echo requests:\n\
* -f, --flood        flood ping \n\
* -l, --preload N    send N packets as fast as possible before falling into\n\
                     normal mode of behavior\n\
  -p, --pattern PAT  fill ICMP packet with given pattern (hex)\n\
  -q, --quiet        quiet output\n\
  -R, --route        record route\n\
  -s, --size N       set number of data octets to send\n\
\n\
Options marked with an * are available only to super-user\n\
\n\
report bugs to " PACKAGE_BUGREPORT ".\n\
");
}

void
show_license (void)
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

