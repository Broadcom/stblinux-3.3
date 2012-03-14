/*
 */

#define __KLIBC_DIRENT_INTERNALS
#define _KLIBC_IN_OPEN_C

#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <sys/vfs.h>
#include <stdlib.h>
#include <signal.h>
#include <dirent.h>
#include "atexit.h"

/* Should be in bitsize.h */
#ifndef _BITSIZE
#define _BITSIZE 32
#endif
/* Should be in unistd.h */
extern char ** environ;

int execv(const char *path, char *const *argv)
{
	return execve(path, argv, environ);
}
/* Link chain for atexit/on_exit */
struct atexit *__atexit_list;

__noreturn exit(int rv)
{
	struct atexit *ap;

	for (ap = __atexit_list; ap; ap = ap->next) {
		/* This assumes extra args are harmless.  They should
		   be in all normal C ABIs, but if an architecture has
		   some particularly bizarre ABI this might be worth
		   watching out for. */
		ap->fctn(rv, ap->arg);
	}

	/* Handle any library destructors if we ever start using them... */

	_exit(rv);
}

#ifdef __NR_statfs64

extern int __statfs64(const char *, size_t, struct statfs *);

int statfs(const char *path, struct statfs *buf)
{
	return __statfs64(path, sizeof *buf, buf);
}
#endif

DIR *opendir(const char *name)
{
	DIR *dp = malloc(sizeof(DIR));

	if (!dp)
		return NULL;

	dp->__fd = open(name, O_DIRECTORY | O_RDONLY);

	if (dp->__fd < 0) {
		free(dp);
		return NULL;
	}

	dp->bytes_left = 0;

	return dp;
}

struct dirent *readdir(DIR *dir)
{
	struct dirent *dent;
	int rv;

	if (!dir->bytes_left) {
		rv = getdents(dir->__fd, dir->buffer, sizeof(dir->buffer));
		if (rv <= 0)
			return NULL;
		dir->bytes_left = rv;
		dir->next = dir->buffer;
	}

	dent = dir->next;
	dir->next = (struct dirent *)((char *)dir->next + dent->d_reclen);
	dir->bytes_left -= dent->d_reclen;

	return dent;
}

int closedir(DIR *dir)
{
	int rv;
	rv = close(dir->__fd);
	free(dir);
	return rv;
}

int raise(int signal)
{
	return kill(getpid(), signal);
}
#if _BITSIZE == 32 && !defined(__i386__)

extern int __open(const char *, int, mode_t);

int open(const char *pathname, int flags, mode_t mode)
{
	return __open(pathname, flags | O_LARGEFILE, mode);
}

#endif
