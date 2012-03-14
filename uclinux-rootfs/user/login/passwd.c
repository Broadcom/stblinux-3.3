/*****************************************************************************/

/*
 *	passwd.c -- simple change password program.
 *
 *	(C) Copyright 1999, Nick Brok (nick@nbrok.iaehv.nl).
 */

/*****************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pwd.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#ifndef __UC_LIBC__
#include <crypt.h>
#endif
#ifdef EMBED
#include <config/autoconf.h>
#endif

/*****************************************************************************/

char *version = "v1.0.2";

#if defined(CONFIG_USER_FLATFSD_FLATFSD)
#define WORK_DIR	"/etc/config/"
#define	PASSWDFILE	WORK_DIR "config"
#else
#define WORK_DIR	"/var/"
#define	PASSWDFILE	WORK_DIR "passwd"
#endif

#define MAX_CONFIG_LINE_SIZE	300

int writeConfig(char *filename, char *keyword, char *value);
int commitChanges();

/*****************************************************************************/

int main(int argc, char *argv[])
{
	char	*cryptmode, password2[128], password1[128];

	for(;;) {
		strcpy(password1, getpass("Enter new Unix password: "));
		cryptmode = "ab";
		strcpy(password2, getpass("Re-enter new Unix password: "));
		if (strcmp(password1,password2) == 0) {
			strcpy(password2, crypt(password1,cryptmode));
			if (-1 == writeConfig(PASSWDFILE, "passwd", password2))
				printf("Unable to write password file\n");
			else if (-1 == commitChanges())
				printf("Unable to commit new password file\n");
			else
				return 0;
			break;
		} else 
			printf("Password not matched, try again.\n");
	}
	return 1;
}

/*****************************************************************************/

/*
 * writeConfig
 *
 * Write to a config file (filename) a keyword and its value,
 * replacing any previous data for that keyword if it exists.
 * For example:
 * To update the /etc/config files wizard from 1 to 0 you would
 * call:
 *          writeConfig("/etc/config", "wizard", "0");
 *
 * args:    filename - the config file name and path (eg. /etc/config)
 *          keyword - the keywrod to write into the config file
 *                      (eg. wizard)
 *          value - the value for the keyword (eg. 0). If NULL then the
 *                  entry for the keyword is deleted.
 * retn:    0 on success, -1 on failure
 */
int writeConfig(char *filename, char *keyword, char *value) {
    FILE *in;
    FILE *out;
   
    char line_buffer[MAX_CONFIG_LINE_SIZE];
    char tmp[MAX_CONFIG_LINE_SIZE];

    in = fopen(filename, "r");
    out = fopen(WORK_DIR ".ptmp", "w");
   
    if (!out) {
        if(in)
            fclose(in);
        return -1;
    }
   
    while(in && (fgets(line_buffer, MAX_CONFIG_LINE_SIZE -1, in)) != NULL) {
        if(sscanf(line_buffer, "%s", tmp) > 0) {
            if(strcmp(tmp, keyword))
                fputs(line_buffer, out);
        }
    }
   
    if(in)
        fclose(in);

    if (value != NULL) {
        sprintf(tmp, "%s %s\n", keyword, value);
        fputs(tmp, out);
    }

    if (fclose(out) != 0)
        return -1;

    rename(WORK_DIR ".ptmp", filename);

    return 0;
}


int
commitChanges()
{
#ifdef CONFIG_USER_FLATFSD_FLATFSD
	#define FLATFSD_PID_FILE		"/var/run/flatfsd.pid"

	pid_t pid;
	FILE *in;
	char value[16];

	/* get the pid of flatfsd */
	in = fopen(FLATFSD_PID_FILE, "r");

	if(!in) {
		/* couldn't access flatfsd pid file */
		return -1;
	}

	if(fread(value, 1, sizeof(value), in) > 0) {
		/* we read something.. hopefully the pid */
	} else {
		/* no data read from file */
		fclose(in);
		return -1;
	}
	fclose(in);

	pid = atoi(value);

	/* send that pid signal 10 */
	if (pid == 0 || kill(pid, 10) == -1) {
		return -1;
	}
#endif
	return 0;
}

