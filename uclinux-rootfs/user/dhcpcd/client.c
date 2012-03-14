/*
 * dhcpcd - DHCP client daemon -
 * Copyright (C) 1996 - 1997 Yoichi Hariguchi <yoichi@fore.com>
 * Copyright (C) January, 1998 Sergei Viznyuk <sv@phystech.com>
 * 
 * dhcpcd is an RFC2131 and RFC1541 compliant DHCP client daemon.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <net/if.h>
#include <net/if_arp.h>
#ifdef __GLIBC__
#include <net/if_packet.h>
#else
#include <linux/if_packet.h>
#endif
#include <net/route.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <setjmp.h>
#include <time.h>
#include "client.h"
#include "buildmsg.h"
#include "udpipgen.h"
#include "pathnames.h"
#include "kversion.h"

extern	char		*ProgramName;
extern	char		*IfName,*IfNameExt;
extern	int		IfName_len,IfNameExt_len;
extern	char		*HostName;
extern	unsigned char	*ClassID;
extern	int		ClassID_len;
extern  unsigned char	*ClientID;
extern  int		ClientID_len;
extern	int		DebugFlag;
extern	int		BeRFC1541;
extern	unsigned	LeaseTime;
extern	int		SetDomainName;
extern	int		SetHostName;
extern	int		SendSecondDiscover;
extern	unsigned short	ip_id;
extern  void		*(*currState)();
extern  time_t          TimeOut;
extern  unsigned        nleaseTime;
extern  struct in_addr  inform_ipaddr;
extern	int		DoCheckSum;
extern	int		TestCase;
extern	int		resolv_renamed,yp_renamed,ntp_renamed;
extern	int		InitialHostName_len,InitialDomainName_len;
extern	char		*InitialHostName,*InitialDomainName;

#if 0
extern	unsigned char	ClientMACaddr[ETH_ALEN];
extern	int		ClientMACaddr_ind;
#endif

#ifdef ARPCHECK
int arpCheck();
#endif
int	arpRelease();
int	dhcpConfig();
int	readDhcpCache();
int	deleteDhcpCache();
void	execute_on_change(char *prm);

int			dhcpSocket;
int			udpFooSocket;
int			prev_ip_addr;
time_t			ReqSentTime;
dhcpOptions		DhcpOptions;
dhcpInterface		DhcpIface;
udpipMessage		UdpIpMsgSend,UdpIpMsgRecv;
jmp_buf			env;
unsigned char		ClientHwAddr[ETH_ALEN];

const struct ip *ipSend=(struct ip *)((struct udpiphdr *)UdpIpMsgSend.udpipmsg)->ip;
const struct ip *ipRecv=(struct ip *)((struct udpiphdr *)UdpIpMsgRecv.udpipmsg)->ip;
const dhcpMessage *DhcpMsgSend = (dhcpMessage *)&UdpIpMsgSend.udpipmsg[sizeof(udpiphdr)];
      dhcpMessage *DhcpMsgRecv = (dhcpMessage *)&UdpIpMsgRecv.udpipmsg[sizeof(udpiphdr)];

static short int saved_if_flags = 0;
int	TokenRingIf	=	0;
/*****************************************************************************/
int parseDhcpMsgRecv() /* this routine parses dhcp message received */
{
#ifdef DEBUG
  int i,j;
#endif
  register u_char *p = DhcpMsgRecv->options+4;
  unsigned char *end = DhcpMsgRecv->options+sizeof(DhcpMsgRecv->options);

  /* Force T1 and T2 to 0: either new values will be in message, or they
     will need to be recalculated from lease time */
  if ( DhcpOptions.val[dhcpT1value] && DhcpOptions.len[dhcpT1value] > 0 )
    memset(DhcpOptions.val[dhcpT1value],0,DhcpOptions.len[dhcpT1value]);
  if ( DhcpOptions.val[dhcpT2value] && DhcpOptions.len[dhcpT2value] > 0 )
    memset(DhcpOptions.val[dhcpT2value],0,DhcpOptions.len[dhcpT2value]);
  while ( p < end )
    switch ( *p )
      {
        case endOption: goto swend;
       	case padOption: p++; break;
       	default:
	  if ( p[1] )
	    {
	      if ( DhcpOptions.len[*p] == p[1] )
	        memcpy(DhcpOptions.val[*p],p+2,p[1]);
	      else
	        {
		  DhcpOptions.len[*p] = p[1];
	          if ( DhcpOptions.val[*p] )
	            free(DhcpOptions.val[*p]);
	      	  else
		    DhcpOptions.num++;
	      	  DhcpOptions.val[*p] = malloc(p[1]+1);
		  memset(DhcpOptions.val[*p],0,p[1]+1);
	  	  memcpy(DhcpOptions.val[*p],p+2,p[1]);
	        }
	    }
	  p+=p[1]+2;
      }
swend:
#ifdef DEBUG
  fprintf(stderr,"parseDhcpMsgRecv: %d options received:\n",DhcpOptions.num);
  for (i=1;i<255;i++)
    if ( DhcpOptions.val[i] )
      switch ( i )
        {
	  case 1: /* subnet mask */
	  case 3: /* routers on subnet */
	  case 4: /* time servers */
	  case 5: /* name servers */
	  case 6: /* dns servers */
	  case 28:/* broadcast addr */
	  case 33:/* staticRoute */
	  case 41:/* NIS servers */
	  case 42:/* NTP servers */
	  case 50:/* dhcpRequestdIPaddr */
	  case 54:/* dhcpServerIdentifier */
	    for (j=0;j<DhcpOptions.len[i];j+=4)
	      fprintf(stderr,"i=%-2d  len=%-2d  option = %u.%u.%u.%u\n",
		i,DhcpOptions.len[i],
		((unsigned char *)DhcpOptions.val[i])[0+j],
		((unsigned char *)DhcpOptions.val[i])[1+j],
		((unsigned char *)DhcpOptions.val[i])[2+j],
		((unsigned char *)DhcpOptions.val[i])[3+j]);
	    break;
	  case 2: /* time offset */
	  case 51:/* dhcpAddrLeaseTime */
	  case 57:/* dhcpMaxMsgSize */
	  case 58:/* dhcpT1value */
	  case 59:/* dhcpT2value */
	    fprintf(stderr,"i=%-2d  len=%-2d  option = %d\n",
		i,DhcpOptions.len[i],
		    ntohl(*(int *)DhcpOptions.val[i]));
	    break;
	  case 23:/* defaultIPTTL */
	  case 29:/* performMaskdiscovery */
	  case 31:/* performRouterdiscovery */
	  case 53:/* dhcpMessageType */
	    fprintf(stderr,"i=%-2d  len=%-2d  option = %u\n",
		i,DhcpOptions.len[i],*(unsigned char *)DhcpOptions.val[i]);
	    break;
	  default:
	    fprintf(stderr,"i=%-2d  len=%-2d  option = \"%s\"\n",
		i,DhcpOptions.len[i],(char *)DhcpOptions.val[i]);
	}
fprintf(stderr,"\
DhcpMsgRecv->yiaddr  = %u.%u.%u.%u\n\
DhcpMsgRecv->siaddr  = %u.%u.%u.%u\n\
DhcpMsgRecv->giaddr  = %u.%u.%u.%u\n\
DhcpMsgRecv->sname   = \"%s\"\n\
ServerHardwareAddr   = %02X.%02X.%02X.%02X.%02X.%02X\n",
((unsigned char *)&DhcpMsgRecv->yiaddr)[0],
((unsigned char *)&DhcpMsgRecv->yiaddr)[1],
((unsigned char *)&DhcpMsgRecv->yiaddr)[2],
((unsigned char *)&DhcpMsgRecv->yiaddr)[3],
((unsigned char *)&DhcpMsgRecv->siaddr)[0],
((unsigned char *)&DhcpMsgRecv->siaddr)[1],
((unsigned char *)&DhcpMsgRecv->siaddr)[2],
((unsigned char *)&DhcpMsgRecv->siaddr)[3],
((unsigned char *)&DhcpMsgRecv->giaddr)[0],
((unsigned char *)&DhcpMsgRecv->giaddr)[1],
((unsigned char *)&DhcpMsgRecv->giaddr)[2],
((unsigned char *)&DhcpMsgRecv->giaddr)[3],
DhcpMsgRecv->sname,
UdpIpMsgRecv.ethhdr.ether_shost[0],
UdpIpMsgRecv.ethhdr.ether_shost[1],
UdpIpMsgRecv.ethhdr.ether_shost[2],
UdpIpMsgRecv.ethhdr.ether_shost[3],
UdpIpMsgRecv.ethhdr.ether_shost[4],
UdpIpMsgRecv.ethhdr.ether_shost[5]);
#endif
  if ( ! DhcpMsgRecv->yiaddr ) DhcpMsgRecv->yiaddr=DhcpMsgSend->ciaddr;
  if ( ! DhcpOptions.val[dhcpServerIdentifier] ) /* did not get dhcpServerIdentifier */
    {	/* make it the same as IP address of the sender */
      DhcpOptions.val[dhcpServerIdentifier] = malloc(4);
      memcpy(DhcpOptions.val[dhcpServerIdentifier],&ipRecv->ip_src.s_addr,4);
      DhcpOptions.len[dhcpServerIdentifier] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,
	"dhcpServerIdentifier option is missing in DHCP server response. Assuming %u.%u.%u.%u\n",
	((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[0],
	((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[1],
	((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[2],
	((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[3]);
    }
  if ( ! DhcpOptions.val[dns] ) /* did not get DNS */
    {	/* make it the same as dhcpServerIdentifier */
      DhcpOptions.val[dns] = malloc(4);
      memcpy(DhcpOptions.val[dns],DhcpOptions.val[dhcpServerIdentifier],4);
      DhcpOptions.len[dns] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,
	"dns option is missing in DHCP server response. Assuming %u.%u.%u.%u\n",
	((unsigned char *)DhcpOptions.val[dns])[0],
	((unsigned char *)DhcpOptions.val[dns])[1],
	((unsigned char *)DhcpOptions.val[dns])[2],
	((unsigned char *)DhcpOptions.val[dns])[3]);
    }
  if ( ! DhcpOptions.val[subnetMask] ) /* did not get subnetMask */
    {
      DhcpOptions.val[subnetMask] = malloc(4);
      ((unsigned char *)DhcpOptions.val[subnetMask])[0] = 255;
#if 0
      if ( ((unsigned char *)&DhcpMsgRecv->yiaddr)[0] < 128 )
#else
      if ( IN_CLASSA(ntohl(DhcpMsgRecv->yiaddr)) )
#endif
	{
          ((unsigned char *)DhcpOptions.val[subnetMask])[1] = 0; /* class A */
          ((unsigned char *)DhcpOptions.val[subnetMask])[2] = 0;
          ((unsigned char *)DhcpOptions.val[subnetMask])[3] = 0;
	}
      else
	{
          ((unsigned char *)DhcpOptions.val[subnetMask])[1] = 255;
#if 0
	  if ( ((unsigned char *)&DhcpMsgRecv->yiaddr)[0] < 192 )
#else
          if ( IN_CLASSB(ntohl(DhcpMsgRecv->yiaddr)) )
#endif
	    {
	      ((unsigned char *)DhcpOptions.val[subnetMask])[2] = 0;/* class B */
              ((unsigned char *)DhcpOptions.val[subnetMask])[3] = 0;
	    }
	  else
	    {
	      ((unsigned char *)DhcpOptions.val[subnetMask])[2] = 255;
	      if ( IN_CLASSC(ntohl(DhcpMsgRecv->yiaddr)) )
                ((unsigned char *)DhcpOptions.val[subnetMask])[3] = 0; /* class C */
	      else
                ((unsigned char *)DhcpOptions.val[subnetMask])[3] = 255;
	    }
	}
      DhcpOptions.len[subnetMask] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,
	"subnetMask option is missing in DHCP server response. Assuming %u.%u.%u.%u\n",
	((unsigned char *)DhcpOptions.val[subnetMask])[0],
	((unsigned char *)DhcpOptions.val[subnetMask])[1],
	((unsigned char *)DhcpOptions.val[subnetMask])[2],
	((unsigned char *)DhcpOptions.val[subnetMask])[3]);
    }
  if ( ! DhcpOptions.val[broadcastAddr] ) /* did not get broadcastAddr */
    {
      int br = DhcpMsgRecv->yiaddr | ~*((int *)DhcpOptions.val[subnetMask]);
      DhcpOptions.val[broadcastAddr] = malloc(4);
      memcpy(DhcpOptions.val[broadcastAddr],&br,4);
      DhcpOptions.len[broadcastAddr] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,
	"broadcastAddr option is missing in DHCP server response. Assuming %u.%u.%u.%u\n",
	((unsigned char *)DhcpOptions.val[broadcastAddr])[0],
	((unsigned char *)DhcpOptions.val[broadcastAddr])[1],
	((unsigned char *)DhcpOptions.val[broadcastAddr])[2],
	((unsigned char *)DhcpOptions.val[broadcastAddr])[3]);
    }
#if 0
  if ( ! DhcpOptions.val[routersOnSubnet] )
    {
      DhcpOptions.val[routersOnSubnet] = malloc(4);
      if ( DhcpMsgRecv->giaddr )
      	memcpy(DhcpOptions.val[routersOnSubnet],&DhcpMsgRecv->giaddr,4);
      else
	memcpy(DhcpOptions.val[routersOnSubnet],DhcpOptions.val[dhcpServerIdentifier],4);
      DhcpOptions.len[routersOnSubnet] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,
	"routersOnSubnet option is missing in DHCP server response. Assuming %u.%u.%u.%u\n",
	((unsigned char *)DhcpOptions.val[routersOnSubnet])[0],
	((unsigned char *)DhcpOptions.val[routersOnSubnet])[1],
	((unsigned char *)DhcpOptions.val[routersOnSubnet])[2],
	((unsigned char *)DhcpOptions.val[routersOnSubnet])[3]);
    }
#endif
  if ( DhcpOptions.val[dhcpIPaddrLeaseTime] && DhcpOptions.len[dhcpIPaddrLeaseTime] == 4 )
    {
      if ( *(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime] == 0 )
	{
          memcpy(DhcpOptions.val[dhcpIPaddrLeaseTime],&nleaseTime,4);
	  if ( DebugFlag )
	    syslog(LOG_DEBUG,"dhcpIPaddrLeaseTime=0 in DHCP server response. Assuming %u sec\n",LeaseTime);
	}
      else
	if ( DebugFlag )
	  syslog(LOG_DEBUG,"dhcpIPaddrLeaseTime=%u in DHCP server response.\n",
	  ntohl(*(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime]));
    }
  else /* did not get dhcpIPaddrLeaseTime */
    {
      DhcpOptions.val[dhcpIPaddrLeaseTime] = malloc(4);
      memcpy(DhcpOptions.val[dhcpIPaddrLeaseTime],&nleaseTime,4);
      DhcpOptions.len[dhcpIPaddrLeaseTime] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,"dhcpIPaddrLeaseTime option is missing in DHCP server response. Assuming %u sec\n",LeaseTime);
    }
  if ( DhcpOptions.val[dhcpT1value] && DhcpOptions.len[dhcpT1value] == 4 )
    {
      if ( *(unsigned int *)DhcpOptions.val[dhcpT1value] == 0 )
	{
	  unsigned t2 = 0.5*ntohl(*(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime]);
	  int t1 = htonl(t2);
	  memcpy(DhcpOptions.val[dhcpT1value],&t1,4);
	  DhcpOptions.len[dhcpT1value] = 4;
	  if ( DebugFlag )
	    syslog(LOG_DEBUG,"dhcpT1value is missing in DHCP server response. Assuming %u sec\n",t2);
	}
    }
  else		/* did not get T1 */
    {
      unsigned t2 = 0.5*ntohl(*(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime]);
      int t1 = htonl(t2);
      DhcpOptions.val[dhcpT1value] = malloc(4);
      memcpy(DhcpOptions.val[dhcpT1value],&t1,4);
      DhcpOptions.len[dhcpT1value] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,"dhcpT1value is missing in DHCP server response. Assuming %u sec\n",t2);
    }
  if ( DhcpOptions.val[dhcpT2value] && DhcpOptions.len[dhcpT2value] == 4 )
    {
      if ( *(unsigned int *)DhcpOptions.val[dhcpT2value] == 0 )
	{
	  unsigned t2 = 0.875*ntohl(*(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime]);
	  int t1 = htonl(t2);
	  memcpy(DhcpOptions.val[dhcpT2value],&t1,4);
	  DhcpOptions.len[dhcpT2value] = 4;
	  if ( DebugFlag )
	    syslog(LOG_DEBUG,"dhcpT2value is missing in DHCP server response. Assuming %u sec\n",t2);
	}
    }
  else		/* did not get T2 */
    {
      unsigned t2 = 0.875*ntohl(*(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime]);
      int t1 = htonl(t2);
      DhcpOptions.val[dhcpT2value] = malloc(4);
      memcpy(DhcpOptions.val[dhcpT2value],&t1,4);
      DhcpOptions.len[dhcpT2value] = 4;
      DhcpOptions.num++;
      if ( DebugFlag )
	syslog(LOG_DEBUG,"dhcpT2value is missing in DHCP server response. Assuming %u sec\n",t2);
    }
  if ( DhcpOptions.val[dhcpMessageType] )
    return *(unsigned char *)DhcpOptions.val[dhcpMessageType];
  return 0;
}
/*****************************************************************************/
void classIDsetup()
{
  struct utsname sname;
  if ( uname(&sname) ) syslog(LOG_ERR,"classIDsetup: uname: %m\n");
  DhcpIface.class_len=snprintf(DhcpIface.class_id,CLASS_ID_MAX_LEN,
  "%s %s %s",sname.sysname,sname.release,sname.machine);
}
/*****************************************************************************/
void clientIDsetup()
{
  unsigned char *c = DhcpIface.client_id;
  *c++ = dhcpClientIdentifier;
  if ( ClientID )
    {
      *c++ = ClientID_len + 1;	/* 1 for the field below */
      *c++ = 0;			/* type: string */
      memcpy(c,ClientID,ClientID_len);
      DhcpIface.client_len = ClientID_len + 3;
      return;
    }
  *c++ = ETH_ALEN + 1;	        /* length: 6 (MAC Addr) + 1 (# field) */
  *c++ = (TokenRingIf) ? ARPHRD_IEEE802_TR : ARPHRD_ETHER;	/* type: Ethernet address */
#if 0
  memcpy(c,DhcpIface.chaddr,ETH_ALEN);
#else
  memcpy(c,ClientHwAddr,ETH_ALEN);
#endif
  DhcpIface.client_len = ETH_ALEN + 3;
}
/*****************************************************************************/
void releaseDhcpOptions()
{
  register int i;
  for (i=1;i<256;i++)
    if ( DhcpOptions.val[i] ) free(DhcpOptions.val[i]);
  memset(&DhcpOptions,0,sizeof(dhcpOptions));
}
/*****************************************************************************/
#ifdef DEBUG
static void dumpframe(const char *title, struct packed_ether_header *frame)
{
  int i;
  unsigned char *dp;

  printf("%s:", title);
  dp = (unsigned char *)frame;
  for (i = 0; i < 32; i++)
    {
      if ((i % 16) == 0)
	printf("\n");
      printf("0x%02X ", *dp++);
    }
}
#endif /* DEBUG */
/*****************************************************************************/
/***** convert ethernet and token-ring frames *****/
int eth2tr(struct packed_ether_header *frame, int datalen)
{
  struct trh_hdr *phdr;
  struct trllc *pllc;
  char trheader[sizeof(struct trh_hdr) - sizeof(phdr->rseg) + sizeof(struct trllc)];
  int len;

#ifdef DEBUG
  dumpframe("eth2tr: Incoming eth frame", frame);
#endif
  memset(trheader, 0, sizeof(trheader));
  phdr = (struct trh_hdr *)trheader;
  phdr->ac = AC;
  phdr->fc = LLC_FRAME;
  memcpy(phdr->daddr, frame->ether_dhost, TR_ALEN);
  memcpy(phdr->saddr, frame->ether_shost, TR_ALEN);
  if (phdr->daddr[0] & 0x80)
    { /* Destination is a broadcast */
      phdr->rcf = sizeof(phdr->rcf) | htons(TR_RCF_BROADCAST | 0x70); /* Unlimited frame length */
      pllc = (struct trllc *)&phdr->rseg[0];
      phdr->saddr[0] |= TR_RII; /* Set source-route indicator */
      len = sizeof(trheader);
    }
  else
    {
      pllc = (struct trllc *)&phdr->rcf;
      len = sizeof(trheader) - sizeof(phdr->rcf);
    }
  pllc->dsap = EXTENDED_SAP;
  pllc->ssap = EXTENDED_SAP;
  pllc->llc = UI_CMD;
  pllc->protid[0] = pllc->protid[1] = pllc->protid[2] = 0;
  pllc->ethertype = frame->ether_type;
  memmove((char *)(frame + 1) + (len - sizeof(struct packed_ether_header)),
	  frame + 1, datalen); /* Make room for larger TR header */
  memcpy(frame, trheader, len); /* Install TR header */
#ifdef DEBUG
  dumpframe("eth2tr: Outgoing tr frame", frame);
#endif
  return len + datalen;
}
/*****************************************************************************/
int tr2eth(struct packed_ether_header *frame)
{
  struct trh_hdr hdr;
  struct trllc *pllc;
  int hlen = 0;

#ifdef DEBUG
  dumpframe("tr2eth: Incoming tr frame", frame);
#endif
  hdr = *((struct trh_hdr *)frame);
  if (hdr.saddr[0] & TR_RII)
    {
fake_rif :
      hlen = hdr.rcf & ntohs(TR_RCF_LEN_MASK);
#ifdef DEBUG
      printf("rcf = 0x%X SR len %d\n", hdr.rcf, hlen);
#endif
      if (hlen < sizeof(hdr.rcf) || (hlen & 1))
	return 1;
      hdr.saddr[0] &= ~TR_RII;
    }
  pllc = (struct trllc *)(((__u8 *)frame) + sizeof(struct trh_hdr) - TR_MAXRIFLEN + hlen);
  if (pllc->dsap != EXTENDED_SAP || pllc->llc != UI_CMD)
    {
      if (hlen == 0)
	  goto fake_rif;	/* Bug in 2.2.3 kernel */
#ifdef DEBUG
      printf("corrupted TR-IP packet of ui=0x%x and dsap 0x%x discarded\n",
	pllc->llc,pllc->dsap);
#endif
      return 1;
    }
  memcpy(frame->ether_dhost, hdr.daddr, ETH_ALEN);
  memcpy(frame->ether_shost, hdr.saddr, ETH_ALEN);
  frame->ether_type = pllc->ethertype;
  memmove(frame + 1, pllc + 1, IPPACKET_SIZE); /* Move data portion: Overlapping buffer */
#ifdef DEBUG
  dumpframe("tr2eth: Outgoing eth frame", frame);
#endif
  return 0;
}
/*****************************************************************************/
/* Subtract the `struct timeval' values X and Y,
   storing the result in RESULT.
   Return 1 if the difference is negative, otherwise 0.  */
static int timeval_subtract(result,x,y)
struct timeval *result,*x,*y;
{
  /* Perform the carry for the later subtraction by updating Y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     `tv_usec' is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}
/*****************************************************************************/
int dhcpSendAndRecv(xid,msg,buildUdpIpMsg)
unsigned xid,msg;
void (*buildUdpIpMsg)(unsigned);
{
  struct sockaddr addr;
  struct timeval begin, current, diff;
  int i,len,o,timeout=0;
  char foobuf[512];
  const struct udphdr *udpRecv;
  int j=DHCP_INITIAL_RTO/2;

  do
    {
      do
    	{
	  j+=j;
	  if (j > DHCP_MAX_RTO) j = DHCP_MAX_RTO;
      	  memset(&addr,0,sizeof(struct sockaddr));
      	  memcpy(addr.sa_data,IfName,IfName_len);
	  buildUdpIpMsg(xid);
	  if ( TokenRingIf )      /* Here we convert a Eth frame into an TR frame */
	    len = eth2tr(&UdpIpMsgSend.ethhdr,sizeof(udpiphdr)+sizeof(dhcpMessage));
	  else
	    len = sizeof(struct packed_ether_header)+sizeof(udpiphdr)+sizeof(dhcpMessage);
	  if ( sendto(dhcpSocket,&UdpIpMsgSend,len,0,
		&addr,sizeof(struct sockaddr)) == -1 )
	    {
	      syslog(LOG_ERR,"sendto: %m\n");
	      return -1;
	    }
	  gettimeofday(&begin, NULL);
      	  i=random();
    	}
      while ( peekfd(dhcpSocket,j+i%200000) );
      do
	{
	  struct ip ipRecv_local;
  	  char *tmp_ip;
	  memset(&UdpIpMsgRecv,0,sizeof(udpipMessage));
      	  o=sizeof(struct sockaddr);
      	  len=recvfrom(dhcpSocket,&UdpIpMsgRecv,sizeof(udpipMessage),0,
		     (struct sockaddr *)&addr,&o);
	  if ( len == -1 )
    	    {
      	      syslog(LOG_ERR,"recvfrom: %m\n");
      	      return -1;
    	    }
	  if ( TokenRingIf )
	    {    /* Here we convert a TR frame into an Eth frame */
	      if ( tr2eth(&UdpIpMsgRecv.ethhdr) ) continue;
	    }
	  gettimeofday(&current, NULL);
	  timeval_subtract(&diff, &current, &begin);
	  timeout = j - diff.tv_sec*1000000 - diff.tv_usec + random()%200000;
	  if ( UdpIpMsgRecv.ethhdr.ether_type != htons(ETHERTYPE_IP) )
	    continue;
	  tmp_ip = UdpIpMsgRecv.udpipmsg;
	  for (i=0;i<sizeof(struct ip)-2;i+=2)
	    if ( ( UdpIpMsgRecv.udpipmsg[i] == 0x45 ) &&
		 ( UdpIpMsgRecv.udpipmsg[i+1] == 0x00 ) )
	      {
		tmp_ip=&(UdpIpMsgRecv.udpipmsg[i]);
		break;
	      }
	  /* Use local copy because ipRecv is not aligned.  */
	  memcpy(&ipRecv_local,((struct udpiphdr *)tmp_ip)->ip,sizeof(struct ip));
	  udpRecv=(struct udphdr *)((char*)(((struct udpiphdr*)tmp_ip)->ip)+sizeof(struct ip));
	  if ( ipRecv_local.ip_p != IPPROTO_UDP ) continue;
	  if ( TokenRingIf )
	    {
	      if ( udpRecv->uh_dport != htons(DHCP_CLIENT_PORT) ) continue;
	    }
	  len-=sizeof(struct packed_ether_header);
	  i=(int )ntohs(ipRecv_local.ip_len);
	  if ( len < i )
	    {
	      if ( DebugFlag ) syslog(LOG_DEBUG,
		"corrupted IP packet of size=%d and ip_len=%d discarded\n",
		len,i);
	      continue;
	    }
	  len=i-(ipRecv_local.ip_hl<<2);
	  i=(int )ntohs(udpRecv->uh_ulen);
	  if ( len < i )
	    {
	      if ( DebugFlag ) syslog(LOG_DEBUG,
		"corrupted UDP msg of size=%d and uh_ulen=%d discarded\n",
		len,i);
	      continue;
	    }
	  if ( DoCheckSum )
	    {
	      len=udpipchk((udpiphdr *)tmp_ip);
	      if ( len )
		{
		  if ( DebugFlag )
		    switch ( len )
		      {
			case -1: syslog(LOG_DEBUG,
			  "corrupted IP packet with ip_len=%d discarded\n",
			  (int )ntohs(ipRecv_local.ip_len));
			  break;
			case -2: syslog(LOG_DEBUG,
			  "corrupted UDP msg with uh_ulen=%d discarded\n",
			  (int )ntohs(udpRecv->uh_ulen));
			break;
		      }
		  continue;
		}
	    }
	  DhcpMsgRecv=(dhcpMessage *)&tmp_ip[(ipRecv_local.ip_hl<<2)+sizeof(struct udphdr)];
	  if ( DhcpMsgRecv->xid != xid ) continue;
	  if (	DhcpMsgRecv->htype != ARPHRD_ETHER &&
		DhcpMsgRecv->htype != (char)ARPHRD_IEEE802_TR )
	    {
	        if ( DebugFlag )
		  syslog(LOG_DEBUG,"wrong msg htype 0x%X discarded\n",DhcpMsgRecv->htype);
		continue;
	    }
	  if ( DhcpMsgRecv->op != DHCP_BOOTREPLY ) continue;
	  while ( udpFooSocket > 0 &&
		recvfrom(udpFooSocket,(void *)foobuf,sizeof(foobuf),0,NULL,NULL) != -1 );
	  if ( parseDhcpMsgRecv() == msg ) return 0;
	  if ( DhcpOptions.val[dhcpMessageType] )
	  if ( *(unsigned char *)DhcpOptions.val[dhcpMessageType] == DHCP_NAK )
	    {
	      if ( DhcpOptions.val[dhcpMsg] )
		syslog(LOG_ERR,
		"DHCP_NAK server response received: %s\n",
		(char *)DhcpOptions.val[dhcpMsg]);
	      else
		syslog(LOG_ERR,
		"DHCP_NAK server response received\n");
	      return 1;
	    }
    	}
      while ( timeout > 0 && peekfd(dhcpSocket, timeout) == 0 );
    }
  while ( 1 );
  return 1;
}
/*****************************************************************************/
void *dhcpStart()
{
  int o = 1;
  unsigned i=0;
  struct ifreq	ifr;
  struct sockaddr_pkt sap;
  struct sockaddr_in clientAddr;
  memset(&ifr,0,sizeof(struct ifreq));
  memcpy(ifr.ifr_name,IfName,IfName_len);
#ifdef OLD_LINUX_VERSION
  dhcpSocket = socket(AF_INET,SOCK_PACKET,htons(ETH_P_ALL));
#else
  dhcpSocket = socket(AF_PACKET,SOCK_PACKET,htons(ETH_P_ALL));
#endif
  if ( dhcpSocket == -1 )
    {
      syslog(LOG_ERR,"dhcpStart: socket: %m\n");
      exit(1);
    }

  if ( ioctl(dhcpSocket,SIOCGIFHWADDR,&ifr) )
    {
      syslog(LOG_ERR,"dhcpStart: ioctl SIOCGIFHWADDR: %m\n");
      exit(1);
    }
  if ( ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER && ifr.ifr_hwaddr.sa_family != ARPHRD_IEEE802_TR )
    {
      syslog(LOG_ERR,"dhcpStart: interface %s is not Ethernet or 802.2 Token Ring\n",ifr.ifr_name);
      exit(1);
    }
  if ( ifr.ifr_hwaddr.sa_family == ARPHRD_IEEE802_TR )
    {
      TokenRingIf = 1;
      if ( DebugFlag )
	syslog(LOG_DEBUG,"found NIC of link layer type %d\n",ifr.ifr_hwaddr.sa_family);
    }
  if ( setsockopt(dhcpSocket,SOL_SOCKET,SO_BROADCAST,&o,sizeof(o)) == -1 )
    {
      syslog(LOG_ERR,"dhcpStart: setsockopt: %m\n");
      exit(1);
    }
  if ( ioctl(dhcpSocket,SIOCGIFFLAGS,&ifr) )  
    {  
      syslog(LOG_ERR,"dhcpStart: ioctl SIOCGIFFLAGS: %m\n");  
      exit(1);  
    }  
  saved_if_flags = ifr.ifr_flags;  
  ifr.ifr_flags = saved_if_flags | IFF_UP | IFF_BROADCAST | IFF_NOTRAILERS | IFF_RUNNING;
  if ( ioctl(dhcpSocket,SIOCSIFFLAGS,&ifr) )
    {
      syslog(LOG_ERR,"dhcpStart: ioctl SIOCSIFFLAGS: %m\n");
      exit(1);
    }
  memset(&sap,0,sizeof(sap));

  do
    {
      i++;
      if ( i>1 )
	syslog(LOG_WARNING,"dhcpStart: retrying MAC address request "
	"(returned %02x:%02x:%02x:%02x:%02x:%02x)",
	ClientHwAddr[0],ClientHwAddr[1],ClientHwAddr[2],
	ClientHwAddr[3],ClientHwAddr[4],ClientHwAddr[5]);
      if ( ioctl(dhcpSocket,SIOCGIFHWADDR,&ifr) )
        {
	  syslog(LOG_ERR,"dhcpStart: ioctl SIOCGIFHWADDR: %m\n");
	  exit(1);
	}
      if ( ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER && ifr.ifr_hwaddr.sa_family != ARPHRD_IEEE802_TR )
	{
	  syslog(LOG_ERR,"dhcpStart: interface %s is not Ethernet or 802.2 Token Ring\n",ifr.ifr_name);
	  exit(1);
	}
      if ( setsockopt(dhcpSocket,SOL_SOCKET,SO_BROADCAST,&o,sizeof(o)) == -1 )
	{
	  syslog(LOG_ERR,"dhcpStart: setsockopt: %m\n");
	  exit(1);
	}
      ifr.ifr_flags = saved_if_flags | IFF_UP | IFF_BROADCAST | IFF_NOTRAILERS | IFF_RUNNING;
      if ( ioctl(dhcpSocket,SIOCSIFFLAGS,&ifr) )
	{
	  syslog(LOG_ERR,"dhcpStart: ioctl SIOCSIFFLAGS: %m\n");
	  exit(1);
	}
      memset(&sap,0,sizeof(sap));
      sap.spkt_protocol = htons(ETH_P_ALL);
      memcpy(sap.spkt_device,IfName,IfName_len);
#ifdef OLD_LINUX_VERSION
      sap.spkt_family = AF_INET;
#else
      sap.spkt_family = AF_PACKET;
#endif
      if ( bind(dhcpSocket,(void*)&sap,sizeof(struct sockaddr)) == -1 )
        syslog(LOG_ERR,"dhcpStart: bind: %m\n");

      memcpy(ClientHwAddr,ifr.ifr_hwaddr.sa_data,ETH_ALEN);
      if ( DebugFlag )
	fprintf(stdout,"dhcpcd: MAC address = %02x:%02x:%02x:%02x:%02x:%02x\n",
	ClientHwAddr[0], ClientHwAddr[1], ClientHwAddr[2],
	ClientHwAddr[3], ClientHwAddr[4], ClientHwAddr[5]);
    }
  while ( !ClientHwAddr[0] &&
	  !ClientHwAddr[1] &&
	  !ClientHwAddr[2] &&
	  !ClientHwAddr[3] &&
	  !ClientHwAddr[4] &&
	  !ClientHwAddr[5] &&
	   i<HWADDR_TRIES );

  i=time(NULL)+ClientHwAddr[5]+4*ClientHwAddr[4]+8*ClientHwAddr[3]+
  16*ClientHwAddr[2]+32*ClientHwAddr[1]+64*ClientHwAddr[0];
  srandom(i);
  ip_id=i&0xffff;

  udpFooSocket = socket(AF_INET,SOCK_DGRAM,0);
  if ( udpFooSocket == -1 )
    {
      syslog(LOG_ERR,"dhcpStart: socket: %m\n");
      exit(1);
    }
  if ( setsockopt(udpFooSocket,SOL_SOCKET,SO_BROADCAST,&o,sizeof(o)) )
    syslog(LOG_ERR,"dhcpStart: setsockopt: %m\n");
  memset(&clientAddr.sin_addr,0,sizeof(&clientAddr.sin_addr));
  clientAddr.sin_family = AF_INET;
  clientAddr.sin_port = htons(DHCP_CLIENT_PORT);
  if ( bind(udpFooSocket,(struct sockaddr *)&clientAddr,sizeof(clientAddr)) )
    {
      if ( errno != EADDRINUSE )
	syslog(LOG_ERR,"dhcpStart: bind: %m\n");
      close(udpFooSocket);
      udpFooSocket = -1;
    }
  else
    if ( fcntl(udpFooSocket,F_SETFL,O_NONBLOCK) == -1 )
      {
	syslog(LOG_ERR,"dhcpStart: fcntl: %m\n");
	exit(1);
      }

  return &dhcpInit;
}
/*****************************************************************************/
void classclientsetup()
{
  if ( ClassID )
    {
      memcpy(DhcpIface.class_id,ClassID,ClassID_len);
      DhcpIface.class_len=ClassID_len;
    }
  else
    classIDsetup();
  clientIDsetup();
}
/*****************************************************************************/
void *dhcpReboot()
{
  if ( sigsetjmp(env,0xffff) )
    {
      if ( DebugFlag )
	syslog(LOG_DEBUG,"timed out waiting for DHCP_ACK response\n");
      alarm(TimeOut);
      return &dhcpInit;
    }
  dhcpStart();
  memset(&DhcpOptions,0,sizeof(DhcpOptions));
  memset(&DhcpIface,0,sizeof(dhcpInterface));
  if ( readDhcpCache() )
    {
      struct ifreq	ifr;
      struct sockaddr_in *p = (struct sockaddr_in *)&(ifr.ifr_addr);
      memset(&DhcpIface,0,sizeof(dhcpInterface));
      memset(&ifr,0,sizeof(struct ifreq));
#ifdef OLD_LINUX_VERSION
      memcpy(ifr.ifr_name,IfName,IfName_len);
#else
      memcpy(ifr.ifr_name,IfNameExt,IfNameExt_len);
#endif
      p->sin_family = AF_INET;
      if ( ioctl(dhcpSocket,SIOCGIFADDR,&ifr) == 0 )
        DhcpIface.ciaddr=p->sin_addr.s_addr;
#if 0
      if ( ClientMACaddr_ind )
        memcpy(DhcpIface.chaddr,ClientMACaddr,ETH_ALEN);
      else
        memcpy(DhcpIface.chaddr,ClientHwAddr,ETH_ALEN);
#endif
      classclientsetup();
      return &dhcpInit;
    }
  classclientsetup();
#if 0
  if ( ClientMACaddr_ind )
    memcpy(DhcpIface.chaddr,ClientMACaddr,ETH_ALEN);
  else
    memcpy(DhcpIface.chaddr,ClientHwAddr,ETH_ALEN);
#endif
  return dhcpRequest(random(),&buildDhcpReboot);
}
/*****************************************************************************/
void *dhcpInit()
{
  releaseDhcpOptions();

#ifdef DEBUG
fprintf(stderr,"ClassID  = \"%s\"\n\
ClientID = \"%u.%u.%u.%02X.%02X.%02X.%02X.%02X.%02X\"\n",
DhcpIface.class_id,
DhcpIface.client_id[0],DhcpIface.client_id[1],DhcpIface.client_id[2],
DhcpIface.client_id[3],DhcpIface.client_id[4],DhcpIface.client_id[5],
DhcpIface.client_id[6],DhcpIface.client_id[7],DhcpIface.client_id[8]);
#endif

  if ( DebugFlag ) syslog(LOG_DEBUG,"broadcasting DHCP_DISCOVER\n");
  if ( dhcpSendAndRecv(random(),DHCP_OFFER,&buildDhcpDiscover) )
    {
      dhcpStop();
      return 0;
    }
  if ( SendSecondDiscover )
    {
      if ( DebugFlag ) syslog(LOG_DEBUG,"broadcasting second DHCP_DISCOVER\n");
      dhcpSendAndRecv(DhcpMsgRecv->xid,DHCP_OFFER,&buildDhcpDiscover);
    }
  prev_ip_addr = DhcpIface.ciaddr;
  DhcpIface.ciaddr = DhcpMsgRecv->yiaddr;
  memcpy(&DhcpIface.siaddr,DhcpOptions.val[dhcpServerIdentifier],4);
  memcpy(DhcpIface.shaddr,UdpIpMsgRecv.ethhdr.ether_shost,ETH_ALEN);
  DhcpIface.xid = DhcpMsgRecv->xid;
/* DHCP_OFFER received */
  if ( DebugFlag )
    syslog(LOG_DEBUG,"DHCP_OFFER received from %s (%u.%u.%u.%u)\n",
    DhcpMsgRecv->sname,
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[0],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[1],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[2],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[3]);

  return dhcpRequest(DhcpIface.xid,&buildDhcpRequest);
}
/*****************************************************************************/
void *dhcpRequest(xid,buildDhcpMsg)
unsigned xid;
void (*buildDhcpMsg)(unsigned);
{
/* send the message and read and parse replies into DhcpOptions */
  if ( DebugFlag )
    syslog(LOG_DEBUG,"broadcasting DHCP_REQUEST for %u.%u.%u.%u\n",
	   ((unsigned char *)&DhcpIface.ciaddr)[0],
	   ((unsigned char *)&DhcpIface.ciaddr)[1],
	   ((unsigned char *)&DhcpIface.ciaddr)[2],
	   ((unsigned char *)&DhcpIface.ciaddr)[3]);
  if ( dhcpSendAndRecv(xid,DHCP_ACK,buildDhcpMsg) ) return &dhcpInit;
  ReqSentTime=time(NULL);
  if ( DebugFlag ) syslog(LOG_DEBUG,
    "DHCP_ACK received from %s (%u.%u.%u.%u)\n",DhcpMsgRecv->sname,
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[0],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[1],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[2],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[3]);
#ifdef ARPCHECK
/* check if the offered IP address already in use */
  if ( arpCheck() )
    {
      if ( DebugFlag ) syslog(LOG_DEBUG,
	"requested %u.%u.%u.%u address is in use\n",
	((unsigned char *)&DhcpIface.ciaddr)[0],
	((unsigned char *)&DhcpIface.ciaddr)[1],
	((unsigned char *)&DhcpIface.ciaddr)[2],
	((unsigned char *)&DhcpIface.ciaddr)[3]);
      dhcpDecline();
      DhcpIface.ciaddr = 0;
      return &dhcpInit;
    }
  if ( DebugFlag ) syslog(LOG_DEBUG,
    "verified %u.%u.%u.%u address is not in use\n",
    ((unsigned char *)&DhcpIface.ciaddr)[0],
    ((unsigned char *)&DhcpIface.ciaddr)[1],
    ((unsigned char *)&DhcpIface.ciaddr)[2],
    ((unsigned char *)&DhcpIface.ciaddr)[3]);
#endif
  if ( dhcpConfig() )
    {
      dhcpStop();
      return 0;
    }

  /* Successfull ACK: Use the fields obtained for future requests */
  memcpy(&DhcpIface.siaddr,DhcpOptions.val[dhcpServerIdentifier],4);
  memcpy(DhcpIface.shaddr,UdpIpMsgRecv.ethhdr.ether_shost,ETH_ALEN);

  return &dhcpBound;
}
/*****************************************************************************/
void *dhcpBound()
{
  int i;
  if ( sigsetjmp(env,0xffff) ) return &dhcpRenew;
  i=ReqSentTime+ntohl(*(unsigned int *)DhcpOptions.val[dhcpT1value])-time(NULL);
  if ( i > 0 )
    alarm(i);
  else
    return &dhcpRenew;

  sleep(ntohl(*(u_int *)DhcpOptions.val[dhcpT1value]));
  return &dhcpRenew;
}
/*****************************************************************************/
void *dhcpRenew()
{
  int i;
  if ( sigsetjmp(env,0xffff) ) return &dhcpRebind;
  i = ReqSentTime+ntohl(*(unsigned int *)DhcpOptions.val[dhcpT2value])-time(NULL);
  if ( i > 0 )
    alarm(i);
  else
    return &dhcpRebind;

  if ( DebugFlag )
    syslog(LOG_DEBUG,"sending DHCP_REQUEST for %u.%u.%u.%u to %u.%u.%u.%u\n",
	   ((unsigned char *)&DhcpIface.ciaddr)[0],
	   ((unsigned char *)&DhcpIface.ciaddr)[1],
	   ((unsigned char *)&DhcpIface.ciaddr)[2],
	   ((unsigned char *)&DhcpIface.ciaddr)[3],
	   ((unsigned char *)&DhcpIface.siaddr)[0],
	   ((unsigned char *)&DhcpIface.siaddr)[1],
	   ((unsigned char *)&DhcpIface.siaddr)[2],
	   ((unsigned char *)&DhcpIface.siaddr)[3]);
  if ( dhcpSendAndRecv(random(),DHCP_ACK,&buildDhcpRenew) ) return &dhcpRebind;
  ReqSentTime=time(NULL);
  if ( DebugFlag ) syslog(LOG_DEBUG,
    "DHCP_ACK received from %s (%u.%u.%u.%u)\n",DhcpMsgRecv->sname,
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[0],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[1],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[2],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[3]);
  return &dhcpBound;
}
/*****************************************************************************/
void *dhcpRebind()
{
  int i;
  if ( sigsetjmp(env,0xffff) ) return &dhcpStop;
  i = ReqSentTime+ntohl(*(unsigned int *)DhcpOptions.val[dhcpIPaddrLeaseTime])-time(NULL);
  if ( i > 0 )
    alarm(i);
  else
    return &dhcpStop;

  if ( DebugFlag )
    syslog(LOG_DEBUG,"broadcasting DHCP_REQUEST for %u.%u.%u.%u\n",
	   ((unsigned char *)&DhcpIface.ciaddr)[0],
	   ((unsigned char *)&DhcpIface.ciaddr)[1],
	   ((unsigned char *)&DhcpIface.ciaddr)[2],
	   ((unsigned char *)&DhcpIface.ciaddr)[3]);
  if ( dhcpSendAndRecv(random(),DHCP_ACK,&buildDhcpRebind) ) return &dhcpStop;
  ReqSentTime=time(NULL);
  if ( DebugFlag ) syslog(LOG_DEBUG,
    "DHCP_ACK received from %s (%u.%u.%u.%u)\n",DhcpMsgRecv->sname,
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[0],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[1],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[2],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[3]);

  /* Successfull ACK: Use the fields obtained for future requests */
  memcpy(&DhcpIface.siaddr,DhcpOptions.val[dhcpServerIdentifier],4);
  memcpy(DhcpIface.shaddr,UdpIpMsgRecv.ethhdr.ether_shost,ETH_ALEN);

  return &dhcpBound;
}
/*****************************************************************************/
void *dhcpRelease()
{
  struct sockaddr addr;
  deleteDhcpCache();
  if ( DhcpIface.ciaddr == 0 ) return &dhcpInit;

  buildDhcpRelease(random());

  memset(&addr,0,sizeof(struct sockaddr));
  memcpy(addr.sa_data,IfName,IfName_len);
  if ( DebugFlag )
    syslog(LOG_DEBUG,"sending DHCP_RELEASE for %u.%u.%u.%u to %u.%u.%u.%u\n",
	   ((unsigned char *)&DhcpIface.ciaddr)[0],
	   ((unsigned char *)&DhcpIface.ciaddr)[1],
	   ((unsigned char *)&DhcpIface.ciaddr)[2],
	   ((unsigned char *)&DhcpIface.ciaddr)[3],
	   ((unsigned char *)&DhcpIface.siaddr)[0],
	   ((unsigned char *)&DhcpIface.siaddr)[1],
	   ((unsigned char *)&DhcpIface.siaddr)[2],
	   ((unsigned char *)&DhcpIface.siaddr)[3]);
  if ( sendto(dhcpSocket,&UdpIpMsgSend,sizeof(struct packed_ether_header)+
	      sizeof(udpiphdr)+sizeof(dhcpMessage),0,
	      &addr,sizeof(struct sockaddr)) == -1 )
    syslog(LOG_ERR,"dhcpRelease: sendto: %m\n");
  arpRelease(); /* clear ARP cache entries for client IP addr */
  if ( SetHostName )
    {
      sethostname(InitialHostName,InitialHostName_len);
      if ( DebugFlag )
	fprintf(stdout,"dhcpcd: your hostname = %s\n",InitialHostName);
    }
  if ( SetDomainName )
    {
      setdomainname(InitialDomainName,InitialDomainName_len);
      if ( DebugFlag )
	fprintf(stdout,"dhcpcd: your domainname = %s\n",InitialDomainName);
    }
  DhcpIface.ciaddr=0;
  return &dhcpInit;
}
/*****************************************************************************/
void *dhcpStop()
{
  struct ifreq ifr;
  struct sockaddr_in	*p = (struct sockaddr_in *)&(ifr.ifr_addr);

  releaseDhcpOptions();
  if ( TestCase ) goto tsc;
  memset(&ifr,0,sizeof(struct ifreq));
#ifdef OLD_LINUX_VERSION
  memcpy(ifr.ifr_name,IfName,IfName_len);
#else
  memcpy(ifr.ifr_name,IfNameExt,IfNameExt_len);
#endif
  p->sin_family = AF_INET;
  p->sin_addr.s_addr = 0;
#ifndef OLD_LINUX_VERSION
  if ( ioctl(dhcpSocket,SIOCSIFADDR,&ifr) == -1 )
    syslog(LOG_ERR,"dhcpStop: ioctl SIOCSIFADDR: %m\n");
#endif
  ifr.ifr_flags = saved_if_flags & ~IFF_UP;
  if ( (IfName_len==IfNameExt_len) && ioctl(dhcpSocket,SIOCSIFFLAGS,&ifr) )
    syslog(LOG_ERR,"dhcpStop: ioctl SIOCSIFFLAGS: %m\n");
tsc:
  close(dhcpSocket);
  if ( resolv_renamed )
    rename(""RESOLV_CONF".sv",RESOLV_CONF);
  if ( yp_renamed )
    rename(""NIS_CONF".sv",NIS_CONF);
  if ( ntp_renamed )
    rename(""NTP_CONF".sv",NTP_CONF);
  execute_on_change("down");
  return &dhcpStart;
}
/*****************************************************************************/
#ifdef ARPCHECK
void *dhcpDecline()
{
  struct sockaddr addr;
  memset(&UdpIpMsgSend,0,sizeof(udpipMessage));
  memcpy(UdpIpMsgSend.ethhdr.ether_dhost,MAC_BCAST_ADDR,ETH_ALEN);
  memcpy(UdpIpMsgSend.ethhdr.ether_shost,ClientHwAddr,ETH_ALEN);
  UdpIpMsgSend.ethhdr.ether_type = htons(ETHERTYPE_IP);
  buildDhcpDecline(random());
  udpipgen((udpiphdr *)&UdpIpMsgSend.udpipmsg,0,INADDR_BROADCAST,
  htons(DHCP_CLIENT_PORT),htons(DHCP_SERVER_PORT),sizeof(dhcpMessage));
  memset(&addr,0,sizeof(struct sockaddr));
  memcpy(addr.sa_data,IfName,IfName_len);
  if ( DebugFlag ) syslog(LOG_DEBUG,"broadcasting DHCP_DECLINE\n");
  if ( sendto(dhcpSocket,&UdpIpMsgSend,sizeof(struct packed_ether_header)+
	      sizeof(udpiphdr)+sizeof(dhcpMessage),0,
	      &addr,sizeof(struct sockaddr)) == -1 )
    syslog(LOG_ERR,"dhcpDecline: sendto: %m\n");
  return &dhcpInit;
}
#endif
/*****************************************************************************/
void *dhcpInform()
{
  dhcpStart();
  memset(&DhcpOptions,0,sizeof(DhcpOptions));
  memset(&DhcpIface,0,sizeof(dhcpInterface));
  if ( ! inform_ipaddr.s_addr )
    {
      struct ifreq ifr;
      struct sockaddr_in *p = (struct sockaddr_in *)&(ifr.ifr_addr);
      memset(&ifr,0,sizeof(struct ifreq));
#ifdef OLD_LINUX_VERSION
      memcpy(ifr.ifr_name,IfName,IfName_len);
#else
      memcpy(ifr.ifr_name,IfNameExt,IfNameExt_len);
#endif
      p->sin_family = AF_INET;
      if ( ioctl(dhcpSocket,SIOCGIFADDR,&ifr) == 0 )
        inform_ipaddr.s_addr=p->sin_addr.s_addr;
      if ( ! inform_ipaddr.s_addr )
	{
	  if ( readDhcpCache() )
	    {
	      syslog(LOG_ERR,"dhcpInform: no IP address given\n");
	      return NULL;
	    }
          else
	    inform_ipaddr.s_addr=DhcpIface.ciaddr;
	}
    }
#if 0
  if ( ! DhcpIface.ciaddr )
    {
      if ( ClientMACaddr_ind )
	memcpy(DhcpIface.chaddr,ClientMACaddr,ETH_ALEN);
      else
	memcpy(DhcpIface.chaddr,ClientHwAddr,ETH_ALEN);
    }
#endif
  DhcpIface.ciaddr=inform_ipaddr.s_addr;
  if ( ! DhcpIface.class_len )
    { 
      if ( ClassID )
        {
    	  memcpy(DhcpIface.class_id,ClassID,ClassID_len);
	  DhcpIface.class_len=ClassID_len;
        }
      else
        classIDsetup();
    }
  if ( ! DhcpIface.client_len ) clientIDsetup();
  if ( sigsetjmp(env,0xffff) )
    {
      if ( DebugFlag )
	syslog(LOG_DEBUG,"timed out waiting for DHCP_ACK response\n");
      return 0;
    }
  if ( DebugFlag )
    syslog(LOG_DEBUG,"broadcasting DHCP_INFORM for %u.%u.%u.%u\n",
	   ((unsigned char *)&DhcpIface.ciaddr)[0],
	   ((unsigned char *)&DhcpIface.ciaddr)[1],
	   ((unsigned char *)&DhcpIface.ciaddr)[2],
	   ((unsigned char *)&DhcpIface.ciaddr)[3]);
  if ( dhcpSendAndRecv(random(),DHCP_ACK,buildDhcpInform) ) return 0;
  if ( DebugFlag ) syslog(LOG_DEBUG,
    "DHCP_ACK received from %s (%u.%u.%u.%u)\n",DhcpMsgRecv->sname,
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[0],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[1],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[2],
    ((unsigned char *)DhcpOptions.val[dhcpServerIdentifier])[3]);
#ifdef ARPCHECK
/* check if the offered IP address already in use */
  if ( arpCheck() )
    {
      if ( DebugFlag ) syslog(LOG_DEBUG,
	"requested %u.%u.%u.%u address is in use\n",
	((unsigned char *)&DhcpIface.ciaddr)[0],
	((unsigned char *)&DhcpIface.ciaddr)[1],
	((unsigned char *)&DhcpIface.ciaddr)[2],
	((unsigned char *)&DhcpIface.ciaddr)[3]);
      dhcpDecline();
      return 0;
    }
  if ( DebugFlag ) syslog(LOG_DEBUG,
    "verified %u.%u.%u.%u address is not in use\n",
    ((unsigned char *)&DhcpIface.ciaddr)[0],
    ((unsigned char *)&DhcpIface.ciaddr)[1],
    ((unsigned char *)&DhcpIface.ciaddr)[2],
    ((unsigned char *)&DhcpIface.ciaddr)[3]);
#endif
  if ( dhcpConfig() ) return 0;
  exit(0);
}
