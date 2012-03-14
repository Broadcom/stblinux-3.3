/* Replacement for tzfile.h */
#define DAYSPERNYEAR 365
#define SECSPERDAY 86400

/* BSD-specific function */
void strmode(mode_t mode, char *bp);

/* Unclassified */
char *user_from_uid(uid_t uid, int ignored);
char *group_from_gid(gid_t gid, int ignored);

/* SYSV does not define dirfd() */
#ifndef HAVE_DIRFD
#define dirfd(dir) ((dirp)->dd_fd)
#endif

#ifndef _D_EXACT_NAMLEN
#define _D_EXACT_NAMLEN(d) (strlen(d->d_name) + 1)
#endif

