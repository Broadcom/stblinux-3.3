/* Copyright (C) 2001, 2002 Free Software Foundation, Inc.

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

/* NOTE: most of existing routers simply discard ICMP_ADDRESS requests. */
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

#include "getopt.h"
#include <icmp.h>
#include <ping.h>
#include <ping_impl.h>

static int recv_address (int code, void *closure,
			struct sockaddr_in *dest, struct sockaddr_in *from,
			struct ip *ip, icmphdr_t *icmp, int datalen);
static void print_address (int dupflag, void *closure,
			  struct sockaddr_in *dest, struct sockaddr_in *from,
			  struct ip *ip, icmphdr_t *icmp, int datalen);
static void print_address (int dupflag, void *closure,
			  struct sockaddr_in *dest, struct sockaddr_in *from,
			  struct ip *ip, icmphdr_t *icmp, int datalen);
static int address_finish ();

int
ping_address (int argc, char **argv)
{
  ping_set_type (ping, ICMP_ADDRESS);
  ping_set_event_handler (ping, recv_address, NULL);
  ping_set_packetsize (ping, 12); /* FIXME: constant */
  ping_set_count (ping, 1);

  if (ping_set_dest (ping, *argv))
    {
      fprintf (stderr, "ping: unknown host\n");
      exit (1);
    }

  printf ("PING %s (%s): sending address mask request\n",
	 ping->ping_hostname,
	 inet_ntoa (ping->ping_dest.sin_addr));

  return ping_run (ping, address_finish);
}


int
recv_address (int code, void *closure,
	     struct sockaddr_in *dest, struct sockaddr_in *from,
	     struct ip *ip, icmphdr_t *icmp, int datalen)
{
  switch (code)
    {
    case PEV_RESPONSE:
    case PEV_DUPLICATE:
      print_address (code == PEV_DUPLICATE,
		    closure, dest, from, ip, icmp, datalen);
      break;
    case PEV_NOECHO:;
      print_icmp_header (from, ip, icmp, datalen);
    }
  return 0;
}

void
print_address (int dupflag, void *closure,
	      struct sockaddr_in *dest, struct sockaddr_in *from,
	      struct ip *ip, icmphdr_t *icmp, int datalen)
{
  struct in_addr addr;

  printf ("%d bytes from %s: icmp_seq=%u", datalen,
	 inet_ntoa (*(struct in_addr *)&from->sin_addr.s_addr),
	 icmp->icmp_seq);
  if (dupflag)
    printf (" (DUP!)");
  addr.s_addr = icmp->icmp_mask;
  printf ("icmp_mask = %s", inet_ntoa (addr));
  printf ("\n");
  return;
}


int
address_finish ()
{
  return 0;
}
