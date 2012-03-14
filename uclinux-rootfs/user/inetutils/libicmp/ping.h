/* Copyright (C) 1998 Free Software Foundation, Inc.

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

typedef struct ping_data PING;
typedef int (*ping_efp) __P ((int code,
			     void *closure,
			     struct sockaddr_in *dest, 
			     struct sockaddr_in *from,
			     struct ip *ip, icmphdr_t *icmp, int datalen));


struct ping_data
{
  int    ping_fd;        /* Raw socket descriptor */
  int    ping_type;      /* Type of packets to send */
  int    ping_count;     /* Number of packets to send */
  int    ping_interval;  /* Number of seconds to wait between sending pkts */
  struct sockaddr_in ping_dest; /* whom to ping */
  char   *ping_hostname;     /* Printable hostname */
  size_t ping_datalen;   /* Length of data */
  int    ping_ident;     /* Our identifier */
                          
  ping_efp ping_event;   /* User-defined handler */
  void *ping_closure;    /* User-defined data */

  /* Runtime info */
  int ping_cktab_size;
  char   *ping_cktab;
  
  u_char *ping_buffer;   /* I/O buffer */
  struct sockaddr_in ping_from;
  long   ping_num_xmit;  /* Number of packets transmitted */
  long   ping_num_recv;  /* Number of packets received */
  long   ping_num_rept;  /* Number of duplicates received */
};

#define PEV_RESPONSE 0
#define PEV_DUPLICATE 1
#define PEV_NOECHO  2

#define PING_INTERVAL 1
#define	PING_CKTABSIZE 128

#define _PING_BUFLEN(p) ((p)->ping_datalen + sizeof (icmphdr_t))

#define _C_BIT(p,bit)    (p)->ping_cktab[(bit)>>3]  /* byte in ck array */
#define _C_MASK(bit)     (1 << ((bit) & 0x07))

#define _PING_SET(p,bit) (_C_BIT (p,bit) |= _C_MASK (bit))
#define _PING_CLR(p,bit) (_C_BIT (p,bit) &= (~_C_MASK (bit)))
#define _PING_TST(p,bit) (_C_BIT (p,bit) & _C_MASK (bit))

PING *ping_init (int type, int ident);
void ping_set_type (PING *p, int type);
void ping_set_count (PING *ping, int count);
void ping_set_sockopt (PING *ping, int opt, void *val, int valsize);
void ping_set_interval (PING *ping, int interval);
void ping_set_packetsize (PING *ping, int size);
int ping_set_dest (PING *ping, char *host);
int ping_set_pattern (PING *p, int len, u_char *pat);
void ping_set_event_handler (PING *ping, ping_efp fp, void *closure);
int ping_set_data (PING *p, void *data, size_t off, size_t len);
void ping_set_datalen (PING *p, size_t len);
int ping_recv (PING *p);
int ping_xmit (PING *p);
