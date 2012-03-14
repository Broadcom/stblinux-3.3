/* Copyright (C) 1998, 2001 Free Software Foundation, Inc.

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

#include <sys/socket.h>
#include <sys/time.h>
#include <signal.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
/*#include <netinet/ip_icmp.h> -- deliberately not including this */
#include <arpa/inet.h>
#include <icmp.h>

int
icmp_generic_encode (u_char *buffer, size_t bufsize, int type, int ident, int seqno)
{
  icmphdr_t *icmp;

  if (bufsize < 8)
    return -1;
  icmp = (icmphdr_t *)buffer;
  icmp->icmp_type = type;
  icmp->icmp_code = 0;
  icmp->icmp_cksum = 0;
  icmp->icmp_seq = seqno;
  icmp->icmp_id = ident;

  icmp->icmp_cksum = icmp_cksum (buffer, bufsize);
  return 0;
}

int
icmp_generic_decode (u_char *buffer, size_t bufsize,
		    struct ip **ipp, icmphdr_t **icmpp)
{
  size_t hlen;
  u_short cksum;
  struct ip *ip;
  icmphdr_t *icmp;

  /* IP header */
  ip = (struct ip*) buffer;
  hlen = ip->ip_hl << 2;
  if (bufsize < hlen + ICMP_MINLEN)
    return -1;

  /* ICMP header */
  icmp = (icmphdr_t*)(buffer + hlen);

  /* Prepare return values */
  *ipp = ip;
  *icmpp = icmp;

  /* Recompute checksum */
  cksum = icmp->icmp_cksum;
  icmp->icmp_cksum = 0;
  icmp->icmp_cksum = icmp_cksum ((u_char*)icmp, bufsize-hlen);
  if (icmp->icmp_cksum != cksum)
    return 1;
  return 0;
}

int
icmp_echo_encode (u_char *buffer, size_t bufsize, int ident, int seqno)
{
  return icmp_generic_encode (buffer, bufsize, ICMP_ECHO, ident, seqno);
}

int
icmp_echo_decode (u_char *buffer, size_t bufsize,
		 struct ip **ipp, icmphdr_t **icmpp)
{
  return icmp_generic_decode (buffer, bufsize, ipp, icmpp);
}


