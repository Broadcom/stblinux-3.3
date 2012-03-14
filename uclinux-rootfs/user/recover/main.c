/****************************************************************************/

/*
 *	main.c -- recover FLASH contents after network load.
 *
 *	(C) Copyright 2000, Lineo Inc (www.lineo.com)
 */

/****************************************************************************/

#include <stdio.h>
#include <setjmp.h>
#include <unistd.h>

/****************************************************************************/

char	serverbuf[128];
jmp_buf	doflash;

/*
 *	Define the ethernet interface to use.
 */
#ifndef ETHER_INTERFACE
#define	ETHER_INTERFACE		"eth0"
#endif

/****************************************************************************/

int main()
{
	char	*localargv[16];
	int	i;

	printf("RECOVER: launching DHCP client.\n");

	if (setjmp(doflash) == 0) {
		localargv[0] = "dhcpcd";
		localargv[1] = "-a";
		localargv[2] = "-p";
		localargv[3] = ETHER_INTERFACE;
		localargv[4] = NULL;
		dhcpcdmain(4, localargv, NULL);
	}

	printf("RECOVER: fetching new images from %s\n", serverbuf);

	optind = 0;
	i = 0;
	localargv[i++] = "netflash";
	localargv[i++] = "-k";
	localargv[i++] = "-i";
#ifdef HMACMD5_KEY
	localargv[i++] = "-m";
	localargv[i++] = HMACMD5_KEY;
#endif
	localargv[i++] = "-r";
	localargv[i++] = "/dev/flash/image";
#ifdef STATIC_SERVER_IP
	localargv[i++] = STATIC_SERVER_IP;
#else
	localargv[i++] = serverbuf;
#endif
	localargv[i++] = "/tftpboot/flash.bin";
	localargv[i] = NULL;
	netflashmain(i, localargv);

	exit(0);
}

/****************************************************************************/
