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

/*
 * Structure of an icmp header.
 */
typedef struct icmp_header icmphdr_t;

struct icmp_header
{
  u_char  icmp_type;		/* type of message, see below */
  u_char  icmp_code;		/* type sub code */
  u_short icmp_cksum;		/* ones complement cksum of struct */
  union
  {
    u_char ih_pptr;		/* ICMP_PARAMPROB */
    struct in_addr ih_gwaddr;	/* ICMP_REDIRECT */
    struct ih_idseq
    {
      u_short	icd_id;
      u_short	icd_seq;
    } ih_idseq;
    int ih_void;

    /* ICMP_UNREACH_NEEDFRAG -- Path MTU discovery as per rfc 1191 */
    struct ih_pmtu
    {
      u_short ipm_void;
      u_short ipm_nextmtu;
    } ih_pmtu;

    /* ICMP_ROUTERADV -- RFC 1256 */
    struct ih_rtradv
    {
      u_char irt_num_addrs;     /* Number of addresses following the msg */
      u_char irt_wpa;           /* Address Entry Size (32-bit words) */ 
      u_short irt_lifetime;     /* Lifetime */
    } ih_rtradv;
    
  } icmp_hun;
#define	icmp_pptr	icmp_hun.ih_pptr
#define	icmp_gwaddr	icmp_hun.ih_gwaddr
#define	icmp_id		icmp_hun.ih_idseq.icd_id
#define	icmp_seq	icmp_hun.ih_idseq.icd_seq
#define	icmp_void	icmp_hun.ih_void
#define icmp_pmvoid     icmp_hun.ih_pmtu.ipm_void
#define icmp_nextmtu    icmp_hun.ih_pmtu.ipm_nextmtu
#define icmp_num_addrs  icmp_hun.ih_rtradv.irt_num_addrs
#define icmp_wpa        icmp_hun.ih_rtradv.irt_wpa
#define icmp_lifetime   icmp_hun.ih_rtradv.irt_lifetime
  
  union
  {
    struct id_ts          /* ICMP_TIMESTAMP, ICMP_TIMESTAMP_REPLY */
    {
      n_time its_otime;   /* Originate timestamp */
      n_time its_rtime;   /* Recieve timestamp */
      n_time its_ttime;   /* Transmit timestamp */
    } id_ts;
    struct id_ip          /* Original IP header */
    {
      struct ip idi_ip;
      /* options and then 64 bits of data */
    } id_ip;
    u_long	id_mask;  /* ICMP_ADDRESS, ICMP_ADDRESSREPLY */
    char	id_data[1];
  } icmp_dun;
#define	icmp_otime	icmp_dun.id_ts.its_otime
#define	icmp_rtime	icmp_dun.id_ts.its_rtime
#define	icmp_ttime	icmp_dun.id_ts.its_ttime
#define	icmp_ip		icmp_dun.id_ip.idi_ip
#define	icmp_mask	icmp_dun.id_mask
#define	icmp_data	icmp_dun.id_data
};

#define ICMP_ECHOREPLY		0	/* Echo Reply			*/
#define ICMP_DEST_UNREACH	3	/* Destination Unreachable	*/
/* Codes for ICMP_DEST_UNREACH. */
#define   ICMP_NET_UNREACH	0	/* Network Unreachable		*/
#define   ICMP_HOST_UNREACH	1	/* Host Unreachable		*/
#define   ICMP_PROT_UNREACH	2	/* Protocol Unreachable		*/
#define   ICMP_PORT_UNREACH	3	/* Port Unreachable		*/
#define   ICMP_FRAG_NEEDED	4	/* Fragmentation Needed/DF set	*/
#define   ICMP_SR_FAILED       	5	/* Source Route failed		*/
#define   ICMP_NET_UNKNOWN	6
#define   ICMP_HOST_UNKNOWN	7
#define   ICMP_HOST_ISOLATED	8
#define   ICMP_NET_ANO		9
#define   ICMP_HOST_ANO		10
#define   ICMP_NET_UNR_TOS	11
#define   ICMP_HOST_UNR_TOS	12
#define   ICMP_PKT_FILTERED	13	/* Packet filtered */
#define   ICMP_PREC_VIOLATION	14	/* Precedence violation */
#define   ICMP_PREC_CUTOFF	15	/* Precedence cut off */
#define  NR_ICMP_UNREACH	15	/* total subcodes */

#define ICMP_SOURCE_QUENCH	4	/* Source Quench		*/
#define ICMP_REDIRECT		5	/* Redirect (change route)	*/
/* Codes for ICMP_REDIRECT. */
#define   ICMP_REDIR_NET	0	/* Redirect Net			*/
#define   ICMP_REDIR_HOST	1	/* Redirect Host		*/
#define   ICMP_REDIR_NETTOS	2	/* Redirect Net for TOS		*/
#define   ICMP_REDIR_HOSTTOS	3	/* Redirect Host for TOS	*/

#define ICMP_ECHO		8	/* Echo Request			*/
#define ICMP_ROUTERADV          9       /* Router Advertisement -- RFC 1256 */
#define ICMP_ROUTERDISCOVERY    10      /* Router Discovery -- RFC 1256 */
#define ICMP_TIME_EXCEEDED	11	/* Time Exceeded		*/
/* Codes for TIME_EXCEEDED. */
#define   ICMP_EXC_TTL		0	/* TTL count exceeded		*/
#define   ICMP_EXC_FRAGTIME	1	/* Fragment Reass time exceeded	*/

#define ICMP_PARAMETERPROB	12	/* Parameter Problem		*/
#define ICMP_TIMESTAMP		13	/* Timestamp Request		*/
#define ICMP_TIMESTAMPREPLY	14	/* Timestamp Reply		*/
#define ICMP_INFO_REQUEST	15	/* Information Request		*/
#define ICMP_INFO_REPLY		16	/* Information Reply		*/
#define ICMP_ADDRESS		17	/* Address Mask Request		*/
#define ICMP_ADDRESSREPLY	18	/* Address Mask Reply		*/
#define NR_ICMP_TYPES		18



#define	MAXIPLEN	60
#define	MAXICMPLEN	76
#define	ICMP_MINLEN	8				/* abs minimum */
#define	ICMP_TSLEN	(8 + 3 * sizeof (u_long))	/* timestamp */
#define	ICMP_MASKLEN	12				/* address mask */
#define	ICMP_ADVLENMIN	(8 + sizeof (struct ip) + 8)	/* min */
#define	ICMP_ADVLEN(p)	(8 + ((p)->icmp_ip.ip_hl << 2) + 8)
	/* N.B.: must separately check that ip_hl >= 5 */

u_short icmp_cksum (u_char *addr, int len);
int icmp_generic_encode (u_char *buffer, size_t bufsize, int type, int ident,
			int seqno);
int icmp_generic_decode (u_char *buffer, size_t bufsize,
			struct ip **ipp, icmphdr_t **icmpp);

int icmp_echo_encode (u_char *buffer, size_t bufsize,
		     int ident, int seqno);
int icmp_echo_decode (u_char *buffer, size_t bufsize,
		     struct ip **ip, icmphdr_t **icmp);
int icmp_timestamp_encode (u_char *buffer, size_t bufsize,
			  int ident, int seqno);
int icmp_address_encode (u_char *buffer, size_t bufsize, int ident, int seqno);

