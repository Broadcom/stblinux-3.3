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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <ctype.h>
#include "pathnames.h"
#include "client.h"
#include "signals.h"

struct in_addr  inform_ipaddr,default_router;
char		*ProgramName	=	NULL;
char            **ProgramEnviron=       NULL;
char		*IfName		=	DEFAULT_IFNAME;
char		*IfNameExt	=	DEFAULT_IFNAME;
int		IfName_len	=	DEFAULT_IFNAME_LEN;
int		IfNameExt_len	=	DEFAULT_IFNAME_LEN;
char		*HostName	=	NULL;
int		HostName_len	=	0;
char		*Cfilename	=	NULL;
unsigned char	*ClassID	=	NULL;
int		ClassID_len	=	0;
unsigned char	*ClientID	=	NULL;
int		ClientID_len	=	0;
void		*(*currState)()	=	&dhcpReboot;
int		DebugFlag	=	0;
int		BeRFC1541	=	0;
unsigned	LeaseTime	=	DEFAULT_LEASETIME;
int		ReplResolvConf	=	1;
int		ReplNISConf	=	1;
int		ReplNTPConf	=	1;
int		SetDomainName	=	0;
int		SetHostName	=	0;
int             BroadcastResp   =       0;
time_t          TimeOut         =	DEFAULT_TIMEOUT;
int 		magic_cookie    =       0;
unsigned short  dhcpMsgSize     =       0;
unsigned        nleaseTime      =       0;
int		DoCheckSum	=	0;
int		TestCase	=	0;
int		SendSecondDiscover	=	0;
int		Window		=	0;
char		*ConfigDir	=	CONFIG_DIR;
int		SetDHCPDefaultRoutes=	1;
#if 0
unsigned char	ClientMACaddr[ETH_ALEN];
int		ClientMACaddr_ind =	0;
#endif
/*****************************************************************************/
void print_version()
{
  fprintf(stderr,"\
DHCP Client Daemon v."VERSION"\n\
Copyright (C) 1996 - 1997 Yoichi Hariguchi <yoichi@fore.com>\n\
Copyright (C) January, 1998 Sergei Viznyuk <sv@phystech.com>\n\
Location: http://www.phystech.com/download/\n");
}
/*****************************************************************************/
void checkIfAlreadyRunning()
{
  int o;
  char pidfile[64];
  snprintf(pidfile,sizeof(pidfile),PID_FILE_PATH,ConfigDir,IfNameExt);
  o=open(pidfile,O_RDONLY);
  if ( o == -1 ) return;
  close(o);
  fprintf(stderr,"\
****  %s: already running\n\
****  %s: if not then delete %s file\n",ProgramName,ProgramName,pidfile);
  exit(1);
}
/*****************************************************************************/
int main(argn,argc,argv)
int argn;
char *argc[],*argv[];
{
  int killFlag		=	0;
  int versionFlag	=	0;
  int s			=	1;
  int k			=	1;
  int i			=	1;
  int j;

/*
 * Ensure that fds 0, 1, 2 are open, to /dev/null if nowhere else.
 * This way we can close 0, 1, 2 after forking the daemon without clobbering
 * a fd that we are using (such as our sockets). This is necessary if
 * this program is run from init scripts where 0, 1, and/or 2 may be closed.
 */
 j=open("/dev/null",O_RDWR);
 while ( j < 2 && j >= 0 ) j = dup(j);
 if ( j > 2 ) close(j);
 
  if ( geteuid() )
    {
      fprintf(stderr,"****  %s: not a superuser\n",argc[0]);
      exit(1);
    }

  while ( argc[i] )
    if ( argc[i][0]=='-' )
prgs: switch ( argc[i][s] )
	{
	  case 0:
	    i++;
	    s=1;
	    break;
	  case 'k':
	    s++;
	    killFlag=SIGHUP;
	    goto prgs;
	  case 'n':
	    s++;
	    killFlag=SIGALRM;
	    goto prgs;
	  case 'd':
	    s++;
	    DebugFlag=1;
	    goto prgs;
	  case 'r':
	    s++;
	    BeRFC1541=1;
	    goto prgs;
	  case 'D':
	    s++;
	    SetDomainName=1;
	    goto prgs;
	  case 'H':
	    s++;
	    SetHostName=1;
	    goto prgs;
	  case 'R':
	    s++;
	    ReplResolvConf=0;
	    goto prgs;
	  case 'Y':
	    s++;
	    ReplNISConf=0;
	    goto prgs;
	  case 'N':
	    s++;
	    ReplNTPConf=0;
	    goto prgs;
	  case 'V':
	    s++;
	    versionFlag=1;
	    goto prgs;
	  case 'c':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    Cfilename=argc[i++];
	    if ( Cfilename == NULL || Cfilename[0] == '-' ) goto usage;
	    s=1;
	    break;
	  case 'L':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    ConfigDir=argc[i++];
	    if ( ConfigDir == NULL || ConfigDir[0] != '/' ) goto usage;
	    s=1;
	    break;
#if 0
	  case 'M':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    if ( argc[i] == NULL ) goto usage;
	    if ( isxdigit(argc[i][0]) && isxdigit(argc[i][1]) && argc[i][2] == ':' )
	      ClientMACaddr[0] = 16*(argc[i][0]>64?(toupper(argc[i][0])-55):(argc[i][0]-48)) +
	      (argc[i][1]>64?(toupper(argc[i][1])-55):(argc[i][1]-48));
	    else
	      goto usage;
	    if ( isxdigit(argc[i][3]) && isxdigit(argc[i][4]) && argc[i][5] == ':' )
	      ClientMACaddr[1] = 16*(argc[i][3]>64?(toupper(argc[i][3])-55):(argc[i][3]-48)) +
	      (argc[i][4]>64?(toupper(argc[i][4])-55):(argc[i][4]-48));
	    else
	      goto usage;
	    if ( isxdigit(argc[i][6]) && isxdigit(argc[i][7]) && argc[i][8] == ':' )
	      ClientMACaddr[2] = 16*(argc[i][6]>64?(toupper(argc[i][6])-55):(argc[i][6]-48)) +
	      (argc[i][7]>64?(toupper(argc[i][7])-55):(argc[i][7]-48));
	    else
	      goto usage;
	    if ( isxdigit(argc[i][9]) && isxdigit(argc[i][10]) && argc[i][11] == ':' )
	      ClientMACaddr[3] = 16*(argc[i][9]>64?(toupper(argc[i][9])-55):(argc[i][9]-48)) +
	      (argc[i][10]>64?(toupper(argc[i][10])-55):(argc[i][10]-48));
	    else
	      goto usage;
	    if ( isxdigit(argc[i][12]) && isxdigit(argc[i][13]) && argc[i][14] == ':' )
	      ClientMACaddr[4] = 16*(argc[i][12]>64?(toupper(argc[i][12])-55):(argc[i][12]-48)) +
	      (argc[i][13]>64?(toupper(argc[i][13])-55):(argc[i][13]-48));
	    else
	      goto usage;
	    if ( isxdigit(argc[i][15]) && isxdigit(argc[i][16]) )
	      ClientMACaddr[5] = 16*(argc[i][15]>64?(toupper(argc[i][15])-55):(argc[i][15]-48)) +
	      (argc[i][16]>64?(toupper(argc[i][16])-55):(argc[i][16]-48));
	    else
	      goto usage;
	    s=1;
	    i++;
	    ClientMACaddr_ind=1;
	    break;
#endif
	  case 'i':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    ClassID=argc[i++];
	    if ( ClassID == NULL || ClassID[0] == '-' ) goto usage;
	    s=1;
	    if ( (ClassID_len=strlen(ClassID)) < CLASS_ID_MAX_LEN+1 ) break;
	    fprintf(stderr,"****  %s: too long ClassID string: strlen=%d\n",
	    argc[0],ClassID_len);
	    goto usage;
	  case 'I':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    ClientID=argc[i++];
	    if ( ClientID == NULL || ClientID[0] == '-' ) goto usage;
	    s=1;
	    if ( (ClientID_len=strlen(ClientID)) < CLIENT_ID_MAX_LEN+1 ) break;
	    fprintf(stderr,"****  %s: too long ClientID string: strlen=%d\n",
	    argc[0],ClientID_len);
	    goto usage;
	  case 'h':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    HostName=argc[i++];
	    if ( HostName == NULL || HostName[0] == '-' ) goto usage;
	    s=1;
	    if ( (HostName_len=strlen(HostName)+1) < HOSTNAME_MAX_LEN ) break;
	    fprintf(stderr,"****  %s: too long HostName string: strlen=%d\n",
	    argc[0],HostName_len);
	    break;
	  case 't':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    if ( argc[i] )
	      TimeOut=atol(argc[i++]);
	    else
	      goto usage;
	    s=1;
	    if ( TimeOut > 0 ) break;
	    goto usage;
	  case 'w':
	    if ( argc[i][s+1] ) goto usage;
            i++;
	    if ( argc[i] )
	      Window=atol(argc[i++]);
            else
	      goto usage;
	    s=1;
	    if ( Window >= 0 ) break;
	    goto usage;
	  case 's':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    if ( argc[i] && inet_aton(argc[i],&inform_ipaddr) )
	      i++;
	    else
	      memset(&inform_ipaddr,0,sizeof(inform_ipaddr));
	    currState = &dhcpInform;
	    s=1;
	    break;
	  case 'G':
	    if ( argc[i][s+1] ) goto usage;
	    SetDHCPDefaultRoutes=0;
	    i++;
	    if ( argc[i] && inet_aton(argc[i],&default_router) )
	      i++;
	    else
	      memset(&default_router,0,sizeof(default_router));
	    s=1;
	    break;
	  case 'B':
	    s++;
	    BroadcastResp=1;
	    goto prgs;
	  case 'C':
	    s++;
	    DoCheckSum=1;
	    goto prgs;
	  case 'T':
	    s++;
	    TestCase=1;
	    goto prgs;
	  case 'S':
	    s++;
	    SendSecondDiscover=1;
	    goto prgs;
	  case 'l':
	    if ( argc[i][s+1] ) goto usage;
	    i++;
	    if ( argc[i] )
	      LeaseTime=atol(argc[i++]);
	    else
	      goto usage;
	    s=1;
	    if ( LeaseTime > 0 ) break;
          default:
usage:	    print_version();
	    fprintf(stderr,
"Usage: dhcpcd [-dknrBCDHNRSTY] [-l leasetime] [-h hostname] [-t timeout]\n\
       [-i vendorClassID] [-I ClientID] [-c filename] [-s [ipaddr]]\n\
       [-w windowsize] [-L ConfigDir] [-G [gateway]] [interface]\n");
	    exit(1);
	}
    else
      argc[k++]=argc[i++];
  if ( k > 1 )
    {
      if ( (IfNameExt_len=strlen(argc[1])) > IFNAMSIZ ) goto usage;
      IfNameExt=argc[1];
      IfName=IfNameExt;
      IfName_len=IfNameExt_len;
      s=0;
      while ( IfNameExt[s] )
	if ( IfNameExt[s] == ':' )
	  {
	    IfName=(char *)malloc(s+1);
	    memcpy(IfName,IfNameExt,s);
	    IfName[s]=0;
	    IfName_len=s;
	    break;
	  }
	else
	  s++;
    }
  ProgramName=argc[0];
  ProgramEnviron=argv;
  umask(022);
  if ( killFlag ) killPid(killFlag);
  if ( ! TestCase ) checkIfAlreadyRunning();
  if ( versionFlag ) print_version();
  openlog(PROGRAM_NAME,LOG_PID|(DebugFlag?LOG_CONS:0),LOG_LOCAL0);
  signalSetup();
  if ( mkdir(ConfigDir,S_IRUSR|S_IWUSR|S_IXUSR|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH) && errno != EEXIST )
    {
      syslog(LOG_ERR,"mkdir(\"%s\",0): %m\n",ConfigDir);
      exit(1);
    }
  magic_cookie = htonl(MAGIC_COOKIE);
  dhcpMsgSize = htons(sizeof(dhcpMessage));
  nleaseTime = htonl(LeaseTime);
  alarm(TimeOut);
  do
    if ( (currState=(void *(*)())currState()) == NULL ) exit(1);
  while ( currState != &dhcpBound );
#if 0
  if ( TestCase ) exit(0);
#endif
  alarm(0);
#ifdef DEBUG
  writePidFile(getpid());
#else
#if 0 //def EMBED // PR35375: this is a misuse of vfork
  s=vfork();
#else
  s=fork();
#endif
  if ( s )
    {
      writePidFile(s);
      exit(0); /* got into bound state. */
    }
  setsid();
  if ( (i=open("/dev/null",O_RDWR,0)) >= 0 )
    {
      (void)dup2(i,STDIN_FILENO);
      (void)dup2(i,STDOUT_FILENO);
      (void)dup2(i,STDERR_FILENO);
      if ( i > 2 ) (void)close(i);
    }
#endif
  chdir("/");
  do currState=(void *(*)())currState(); while ( currState );
  deletePidFile();
  exit(1);
}
