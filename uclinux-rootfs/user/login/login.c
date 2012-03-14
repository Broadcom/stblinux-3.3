/*****************************************************************************/

/*
 *	login.c -- simple login program.
 *
 *	(C) Copyright 1999-2001, Greg Ungerer (gerg@snapgear.com).
 * 	(C) Copyright 2001, SnapGear Inc. (www.snapgear.com) 
 * 	(C) Copyright 2000, Lineo Inc. (www.lineo.com) 
 *
 *	Made some changes and additions Nick Brok (nick@nbrok.iaehv.nl).
 */

/*****************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#include <sys/utsname.h>
#include <config/autoconf.h>
#ifndef __UC_LIBC__
#include <crypt.h>
#endif

/*****************************************************************************/

#if defined(CONFIG_USER_FLATFSD_FLATFSD)
#define PATH_PASSWD	"/etc/config/config"
#else
#define PATH_PASSWD	"/etc/passwd"
#endif

/* Delay bad password exit.
 * 
 * This doesn't really accomplish anything I guess..
 * as other connections can be made in the meantime.. and
 * someone attempting a brute force attack could kill their
 * connection if a delay is detected etc.
 *
 * -m2 (20000201)
 */
#define DELAY_EXIT	1

/*****************************************************************************/

char *version = "v1.0.2";

char usernamebuf[128];

/*****************************************************************************/

char *getrealpass(char *pfile)
{
	static char	tmpline[128];
	FILE		*fp;
	char		*spass;
	int		len;

	if ((fp = fopen(pfile, "r")) == NULL) {
		fprintf(stderr, "ERROR: failed to open(%s), errno=%d \n",
			pfile, errno);
		return((char *) NULL);
	}

	while (fgets(tmpline, sizeof(tmpline), fp)) {
		spass = strchr(tmpline, ' ');
		if (spass) {
			*spass++ = 0;
			if (strcmp(tmpline, "passwd") == 0) {
				len = strlen(spass);
				if (spass[len-1] == '\n')
					spass[len-1] = 0;
				return(spass);
			}
		}
	}

	fclose(fp);
	return((char *) NULL);
}

/*****************************************************************************/

int main(int argc, char *argv[])
{
	char	*user;
	char	*realpwd, *gotpwd;
	int	i;

	for (i = 1; (i < argc); i++) {
		if (*argv[i] != '-')
			break;
		if (strcmp(argv[i], "--") == 0) {
			i++;
			break;
		}
		/* Just ignore other options for now. */
	}

	chdir("/");

	if (i < argc) {
		user = argv[i];
	} else {
		printf("login: ");
		fflush(stdout);
		fgets(usernamebuf, sizeof(usernamebuf), stdin);
		user = &usernamebuf[0];
	}

	gotpwd = getpass("Password: ");
	realpwd = getrealpass(PATH_PASSWD);
	if (gotpwd && realpwd) {
		if (strcmp(crypt(gotpwd, realpwd), realpwd) == 0) {
#ifdef EMBED
			execlp("sh", "sh", NULL);
#else
			execlp("sh", "sh", "-t", NULL);
#endif
		} else {
			sleep(DELAY_EXIT);
		}
	}

	exit(0);
}

/*****************************************************************************/
