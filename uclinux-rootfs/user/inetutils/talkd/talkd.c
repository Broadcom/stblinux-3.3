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

#include <intalkd.h>
#include <signal.h>
#include <getopt.h>

#ifndef LOG_FACILITY
# define LOG_FACILITY LOG_DAEMON
#endif

static void show_usage (void);
static void show_version (void);
static void show_license (void);
void talkd_init (void);
void talkd_run (int fd);

static char short_options[] = "VLha:di:r:t:";
static struct option long_options[] =
{
  /* Help options */
  {"version", no_argument, NULL, 'V'},
  {"license", no_argument, NULL, 'L'},
  {"help", no_argument, NULL, 'h'},
  {"acl", required_argument, NULL, 'a'},
  {"debug", no_argument, NULL, 'd'},
  {"idle-timeout", required_argument, NULL, 'i'},
  {"timeout", required_argument, NULL, 't'},
  {"request-ttl", required_argument, NULL, 'r'},
  {NULL, no_argument, NULL, 0}
};

/* Configurable parameters: */
int debug;
unsigned int timeout = 30;
time_t max_idle_time = 120;
time_t max_request_ttl = MAX_LIFE;

char *hostname;

int
main(int argc, char *argv[])
{
  int c;
  char *acl_file = NULL;

  while ((c = getopt_long (argc, argv, short_options, long_options, NULL))
	 != EOF)
    {
      switch (c)
	{
	case 'V':
	  show_version ();
	  exit (0);
	  break;

	case 'L':
	  show_license ();
	  exit (0);

	case 'h':
	  show_usage ();
	  exit (0);
	  break;

	case 'a':
	  acl_file = optarg;
	  break;

	case 'd':
	  debug++;
	  break;

	case 't':
	  timeout = strtoul (optarg, NULL, 0);
	  break;

	case 'i':
	  max_idle_time = strtoul (optarg, NULL, 0);
	  break;

	case 'r':
	  max_request_ttl = strtoul (optarg, NULL, 0);
	  break;

	default:
	  fprintf (stderr, "talkd: %c: not implemented\n", c);
	  exit (1);
	}
    }

  read_acl (acl_file);
  talkd_init ();
  talkd_run (0);
  return 0;
}

void
talkd_init ()
{
  extern char * localhost(void);
  openlog ("talkd", LOG_PID, LOG_FACILITY);
  hostname = localhost ();
  if (!hostname)
    {
      syslog (LOG_ERR, "can't determine my hostname: %m");
      exit (1);
    }
}

time_t last_msg_time;

static void
alarm_handler (int err)
{
  (void)err;
  if ((time (NULL) - last_msg_time) >= max_idle_time)
    exit(0);
  alarm (timeout);
}

void
talkd_run (int fd)
{
  signal (SIGALRM, alarm_handler);
  alarm (timeout);
  while (1)
    {
      int rc;
      struct sockaddr_in sa_in;
      CTL_MSG msg;
      CTL_RESPONSE resp;
      int len;

      len = sizeof sa_in;
      rc = recvfrom (fd, &msg, sizeof msg, 0, (struct sockaddr *)&sa_in, &len);
      if (rc != sizeof msg)
	{
	  if (rc < 0 && errno != EINTR)
	    syslog (LOG_WARNING, "recvfrom: %m");
	  continue;
	}
      last_msg_time = time (NULL);
      if (process_request (&msg, &sa_in, &resp) == 0)
	{
	  rc = sendto(fd, &resp, sizeof resp, 0,
		      (struct sockaddr *)&msg.ctl_addr, sizeof (msg.ctl_addr));
	  if (rc != sizeof resp)
	    syslog (LOG_WARNING, "sendto: %m");
	}
    }
}


void
show_version ()
{
  printf ("talkd - %s %s\n", PACKAGE_NAME, PACKAGE_VERSION);
  printf ("Copyright (C) 2001 Free Software Foundation, Inc.\n");
  printf ("%s comes with ABSOLUTELY NO WARRANTY.\n", PACKAGE_NAME);
  printf ("You may redistribute copies of %s\n", PACKAGE_NAME);
  printf ("under the terms of the GNU General Public License.\n");
  printf ("For more information about these matters, ");
  printf ("see the files named COPYING.\n");
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

void
show_usage (void)
{
  printf ("\
Usage: talkd [OPTION]...\n\
\n\
Options are:\n\
  -a, --acl FILE         read site-wide ACLs from FILE\n\
  -d, --debug            enable debugging\n\
  -i, --idle-timeout SECONDS  set idle timeout value\n\
  -r, --request-ttl SECONDS   set request time-to-live value\n\
  -t, --timeout SECONDS       set timeout value\n\
Informational options:\n\
  -h, --help             display this help and exit\n\
  -L, --license          display license and exit\n\
  -V, --version          output version information and exit\n");
}
