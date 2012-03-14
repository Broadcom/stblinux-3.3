/*
 * Copyright (c) 1983, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef lint
static char copyright[] =
"@(#) Copyright (c) 1983, 1993\n\
	The Regents of the University of California.  All rights reserved.\n";
#endif /* not lint */

#ifndef lint
static char sccsid[] = "@(#)rexecd.c	8.1 (Berkeley) 6/4/93";
#endif /* not lint */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <sys/types.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#ifdef HAVE_SYS_FILIO_H
#include <sys/filio.h>
#endif
#include <sys/socket.h>
#ifdef TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h>
#else
# ifdef HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif
#endif

#include <netinet/in.h>

#include <errno.h>
#include <netdb.h>
#include <pwd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <crypt.h>
#ifdef HAVE_SYS_SELECT_H
#include <sys/select.h>
#endif
#ifdef HAVE_STDARG_H
#include <stdarg.h>
#else
#include <varargs.h>
#endif

void error __P ((const char *fmt, ...));
/*
 * remote execute server:
 *	username\0
 *	password\0
 *	command\0
 *	data
 */
/*ARGSUSED*/
int
main(int argc, char **argv)
{
	struct sockaddr_in from;
	int fromlen, sockfd = STDIN_FILENO;

	fromlen = sizeof (from);
	if (getpeername(sockfd, (struct sockaddr *)&from, &fromlen) < 0) {
		fprintf(stderr,
			"rexecd: getpeername: %s\n", strerror(errno));
		exit(1);
	}
	doit(sockfd, &from);
	exit (0);
}

char	username[20] = "USER=";
char	logname[23] = "LOGNAME=";
char	homedir[64] = "HOME=";
char	shell[64] = "SHELL=";
char	path[sizeof(PATH_DEFPATH) + sizeof("PATH=")] = "PATH=";
char	*envinit[] =
	    {homedir, shell, path, username, logname, 0};
extern char	**environ;

#ifdef HAVE_STRUCT_SOCKADDR_IN_SIN_LEN
struct	sockaddr_in asin = { sizeof(asin), AF_INET };
#else
struct	sockaddr_in asin = { AF_INET };
#endif

char *getstr __P ((const char *));

int
doit(int f, struct sockaddr_in *fromp)
{
	char *cmdbuf, *cp, *namep;
	char *user, *pass;
	struct passwd *pwd;
	int s;
	u_short port;
	int pv[2], pid, cc;
	fd_set readfrom, ready;
	char buf[BUFSIZ], sig;
	int one = 1;

	signal(SIGINT, SIG_DFL);
	signal(SIGQUIT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
#ifdef DEBUG
	{ int t = open(_PATH_TTY, O_RDWR);
	  if (t >= 0) {
		ioctl(t, TIOCNOTTY, (char *)0);
		close(t);
	  }
	}
#endif
	if (f != STDIN_FILENO) {
	    dup2(f, STDIN_FILENO);
	    dup2(f, STDOUT_FILENO);
	    dup2(f, STDERR_FILENO);
	}

	alarm(60);
	port = 0;
	for (;;) {
		char c;
		if (read(f, &c, 1) != 1)
			exit(1);
		if (c == 0)
			break;
		port = port * 10 + c - '0';
	}
	alarm(0);
	if (port != 0) {
		s = socket(AF_INET, SOCK_STREAM, 0);
		if (s < 0)
			exit(1);
		if (bind(s, (struct sockaddr *)&asin, sizeof (asin)) < 0)
			exit(1);
		alarm(60);
		fromp->sin_port = htons(port);
		if (connect(s, (struct sockaddr *)fromp, sizeof (*fromp)) < 0)
			exit(1);
		alarm(0);
	}

	user = getstr ("username");
	pass = getstr ("password");
	cmdbuf = getstr ("command");

	setpwent();
	pwd = getpwnam(user);
	if (pwd == NULL) {
		error("Login incorrect.\n");
		exit(1);
	}
	endpwent();
	if (*pwd->pw_passwd != '\0') {
		namep = CRYPT (pass, pwd->pw_passwd);
		if (strcmp(namep, pwd->pw_passwd)) {
			error("Password incorrect.\n");
			exit(1);
		}
	}
	write(STDERR_FILENO, "\0", 1);
	if (port) {
		pipe(pv);
		pid = fork();
		if (pid == -1)  {
			error("Try again.\n");
			exit(1);
		}
		if (pid) {
			close(STDIN_FILENO);
			close(STDOUT_FILENO);
			close(STDERR_FILENO);
			close(f); close(pv[1]);
			FD_ZERO(&readfrom);
			FD_SET(s, &readfrom);
			FD_SET(pv[0], &readfrom);
			ioctl(pv[1], FIONBIO, (char *)&one);
			/* should set s nbio! */
			do {
			        int maxfd = s;
				ready = readfrom;
				if (pv[0] > maxfd)
				    maxfd = pv[0];
				select(maxfd + 1, (fd_set *)&ready,
				    (fd_set *)NULL, (fd_set *)NULL,
				    (struct timeval *)NULL);
				if (FD_ISSET(s, &ready)) {
					if (read(s, &sig, 1) <= 0)
						FD_CLR(s, &readfrom);
					else
						killpg(pid, sig);
				}
				if (FD_ISSET(pv[0], &ready)) {
					cc = read(pv[0], buf, sizeof (buf));
					if (cc <= 0) {
						shutdown(s, 1+1);
						FD_CLR(pv[0], &readfrom);
					} else
						write(s, buf, cc);
				}
			} while (FD_ISSET(pv[0], &readfrom) ||
				FD_ISSET(s, &readfrom));
			exit(0);
		}
		setpgid (0, getpid());
		close(s);
		close(pv[0]);
		dup2(pv[1], STDERR_FILENO);
	}
	if (*pwd->pw_shell == '\0')
		pwd->pw_shell = PATH_BSHELL;
	if (f > 2)
		close(f);
	setegid((gid_t)pwd->pw_gid);
	setgid((gid_t)pwd->pw_gid);
#ifdef HAVE_INITGROUPS
	initgroups(pwd->pw_name, pwd->pw_gid);
#endif
	setuid((uid_t)pwd->pw_uid);
	if (chdir(pwd->pw_dir) < 0) {
		error("No remote directory.\n");
		exit(1);
	}
	strcat(path, PATH_DEFPATH);
	environ = envinit;
	strncat(homedir, pwd->pw_dir, sizeof(homedir)-6);
	strncat(shell, pwd->pw_shell, sizeof(shell)-7);
	strncat(username, pwd->pw_name, sizeof(username)-6);
	cp = strrchr(pwd->pw_shell, '/');
	if (cp)
		cp++;
	else
		cp = pwd->pw_shell;
	execl(pwd->pw_shell, cp, "-c", cmdbuf, 0);
	perror(pwd->pw_shell);
	exit(1);
}

/*VARARGS1*/
void
error(const char *fmt, ...)
{
  va_list ap;
  int len;
  char buf[BUFSIZ];
#if defined(HAVE_STDARG_H) && defined(__STDC__) && __STDC__
        va_start(ap, fmt);
#else
        va_start(ap);
#endif
  buf[0] = 1;
  snprintf (buf + 1, sizeof buf - 1, fmt, ap);
  write (STDERR_FILENO, buf, strlen(buf));
}

char *
getstr(const char *err)
{
	size_t buf_len = 100;
	char *buf = malloc (buf_len), *end = buf;

	if (! buf) {
		error ("Out of space reading %s\n", err);
		exit (1);
	}

	do {
		/* Oh this is efficient, oh yes.  [But what can be done?] */
		int rd = read(STDIN_FILENO, end, 1);
		if (rd <= 0) {
			if (rd == 0)
				error ("EOF reading %s\n", err);
			else
				perror (err);
			exit(1);
		}

		end += rd;
		if ((buf + buf_len - end) < (buf_len >> 3)) {
			/* Not very much room left in our buffer, grow it. */
			size_t end_offs = end - buf;
			buf_len += buf_len;
			buf = realloc (buf, buf_len);
			if (! buf) {
				error ("Out of space reading %s\n", err);
				exit (1);
			}
			end = buf + end_offs;
		}
	} while (*(end - 1));

	return buf;
}
