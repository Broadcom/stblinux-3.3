/*
 * Copyright (c) 2000-2003 Silicon Graphics, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.  Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Contact information: Silicon Graphics, Inc., 1600 Amphitheatre Pkwy,
 * Mountain View, CA  94043, or:
 *
 * http://www.sgi.com
 *
 * For further information regarding this notice, see:
 *
 * http://oss.sgi.com/projects/GenInfo/SGIGPLNoticeExplan/
 *
 * @configure_input@
 */
#ifndef __XFS_PLATFORM_DEFS_H__
#define __XFS_PLATFORM_DEFS_H__

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>


#if defined(__linux__)

#include <uuid/uuid.h>
#include <sys/vfs.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <linux/types.h>
#include <sys/stat.h>
#include <malloc.h>
#include <getopt.h>
#include <endian.h>
#include <xfs/swab.h>


static __inline__ int xfsctl(const char *path, int fd, int cmd, void *p)
{
	return ioctl(fd, cmd, p);
}

static __inline__ int platform_test_xfs_fd(int fd)
{
	struct statfs buf;
	if (fstatfs(fd, &buf) < 0)
		return 0;
	return (buf.f_type == 0x58465342);	/* XFSB */
}

static __inline__ int platform_test_xfs_path(const char *path)
{
	struct statfs buf;
	if (statfs(path, &buf) < 0)
		return 0;
	return (buf.f_type == 0x58465342);	/* XFSB */
}

static __inline__ int platform_fstatfs(int fd, struct statfs *buf)
{
	return fstatfs(fd, buf);
}

#ifndef O_DIRECT
# if defined (__powerpc__)
#  define O_DIRECT	0400000
# elif defined (__sparc__)
#  define O_DIRECT	0x100000
# endif
#endif

#if (__GLIBC__ < 2) || ((__GLIBC__ == 2) && (__GLIBC_MINOR__ <= 1))
# define constpp	const char * const *
#else
# define constpp	char * const *
#endif

#define ENOATTR		ENODATA	/* Attribute not found */
#define EFSCORRUPTED	990	/* Filesystem is corrupted */

typedef loff_t		xfs_off_t;
typedef __uint64_t	xfs_ino_t;
typedef __uint32_t	xfs_dev_t;
typedef __int64_t	xfs_daddr_t;
typedef char*		xfs_caddr_t;

#ifndef	_UCHAR_T_DEFINED
typedef unsigned char	uchar_t;
#define	_UCHAR_T_DEFINED	1
#endif

#ifndef _BOOLEAN_T_DEFINED
typedef enum {B_FALSE, B_TRUE}	boolean_t;
#define _BOOLEAN_T_DEFINED	1
#endif

#elif defined(__FreeBSD__)

#include <sys/stat.h>
#include <sys/param.h>
#include <sys/ioccom.h>
#include <sys/mount.h>
#include <ctype.h>
#include <libgen.h>
#include <paths.h>
#include <uuid.h>

#include <machine/endian.h>

/* FreeBSD file API is 64-bit aware */
#define	fstat64		fstat
#define	ftruncate64	ftruncate
#define lseek64		lseek
#define	stat64		stat
#define	pwrite64	pwrite
#define	pread64		pread
#define	fdatasync	fsync
#define memalign(a,size)	valloc(size)

typedef u_int8_t	__u8;
typedef int8_t		__s8;
typedef u_int16_t	__u16;
typedef int16_t		__s16;
typedef u_int32_t	__u32;
typedef int32_t		__s32;
typedef u_int64_t	__u64;
typedef int64_t		__s64;

#define constpp	char * const *

#define EFSCORRUPTED	990	/* Filesystem is corrupted */

typedef off_t		xfs_off_t;
typedef off_t		off64_t;
typedef __uint64_t	xfs_ino_t;
typedef __uint32_t	xfs_dev_t;
typedef __int64_t	xfs_daddr_t;
typedef char*		xfs_caddr_t;
typedef off_t		loff_t;

#ifndef	_UCHAR_T_DEFINED
typedef unsigned char	uchar_t;
#define	_UCHAR_T_DEFINED	1
#endif
typedef enum { B_FALSE,B_TRUE }	boolean_t;

#define	O_LARGEFILE	0

#define HAVE_FID	1
#define HAVE_SWABMACROS	1
#define INT_SWAP16(type,var) ((typeof(type))(__bswap16((__u16)(var))))
#define INT_SWAP32(type,var) ((typeof(type))(__bswap32((__u32)(var))))
#define INT_SWAP64(type,var) ((typeof(type))(__bswap64((__u64)(var))))

static __inline__ int xfsctl(const char *path, int fd, int cmd, void *p)
{
	return ioctl(fd, cmd, p);
}

static __inline__ int platform_test_xfs_fd(int fd)
{
	struct statfs buf;
	if (fstatfs(fd, &buf) < 0)
		return 0;
	return strcpy(buf.f_fstypename, "xfs") == 0;
}

static __inline__ int platform_test_xfs_path(const char *path)
{
	struct statfs buf;
	if (statfs(path, &buf) < 0)
		return 0;
	return strcpy(buf.f_fstypename, "xfs") == 0;
}

static __inline__ int platform_fstatfs(int fd, struct statfs *buf)
{
	return fstatfs(fd, buf);
}

/*
 * Implement Linux libuuid functions in terms of DEC DCE's uuid 
 * functions from FreeBSD libc.
 */

static __inline__ int gnu_uuid_compare(uuid_t a, uuid_t b)
{
	return uuid_compare(&a, &b, NULL);
}
#define	uuid_compare	gnu_uuid_compare

static __inline__ int uuid_is_null(uuid_t uid)
{
	return uuid_is_nil(&uid, NULL);
}

static __inline__ void uuid_unparse(uuid_t uid, char *buf)
{
	uint32_t status;
	char *str;
	uuid_to_string(&uid, &str, &status);
	if (status == uuid_s_ok)
		strcpy(buf, str);
	else *buf = '\0';
	free(str);
}

static __inline__ int gnu_uuid_parse(const char *buf, uuid_t *uid)
{
	uint32_t status;
	uuid_from_string(buf, uid, &status);
	return (status == uuid_s_ok);
}
#define	uuid_parse(s,u)	gnu_uuid_parse((s), &(u))

#define uuid_generate(uid)  uuid_create(&(uid), NULL)
#define uuid_clear(uid)  uuid_create_nil(&(uid), NULL)
#define uuid_copy(dst, src)  memcpy(&(dst), &(src), sizeof(uuid_t))

#elif defined(__APPLE__)	/* Darwin */
#include <uuid/uuid.h>
#include <libgen.h>
#include <sys/vm.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mount.h>

#include <sys/syscall.h>
# ifndef SYS_fsctl
#  define SYS_fsctl	242
# endif
static __inline__ int xfsctl(const char *path, int fd, int cmd, void *p)
{
	return syscall(SYS_fsctl, path, cmd, p, 0);
}

static __inline__ int platform_test_xfs_fd(int fd)
{
	struct statfs buf;
	if (fstatfs(fd, &buf) < 0)
		return 0;
	return (strcmp(buf.f_fstypename, "xfs") == 0);
}

static __inline__ int platform_test_xfs_path(const char *path)
{
	struct statfs buf;
	if (statfs(path, &buf) < 0)
		return 0;
	return (strcmp(buf.f_fstypename, "xfs") == 0);
}

static __inline__ int platform_fstatfs(int fd, struct statfs *buf)
{
	return fstatfs(fd, buf);
}

#define __int8_t	int8_t
#define __int16_t	int16_t
#define __int32_t	int32_t
#define __int32_t	int32_t
#define __int64_t	int64_t
#define __uint8_t	u_int8_t
#define __uint16_t	u_int16_t
#define __uint32_t	u_int32_t
#define __uint64_t	u_int64_t
#define __s8		int8_t
#define __s16		int16_t
#define __s32		int32_t
#define __s64		int64_t
#define __u8		u_int8_t
#define __u16		u_int16_t
#define __u32		u_int32_t
#define __u64		u_int64_t
#define loff_t		off_t
#define off64_t		off_t

typedef off_t		xfs_off_t;
typedef u_int64_t	xfs_ino_t;
typedef u_int32_t	xfs_dev_t;
typedef int64_t		xfs_daddr_t;
typedef char*		xfs_caddr_t;

typedef unsigned char	uchar_t;
#define stat64		stat
#define fstat64		fstat
#define lseek64		lseek
#define pread64		pread
#define pwrite64	pwrite
#define ftruncate64	ftruncate
#define fdatasync	fsync
#define memalign(a,sz)	valloc(sz)

#include <machine/endian.h>
#define __BYTE_ORDER	BYTE_ORDER
#define __BIG_ENDIAN	BIG_ENDIAN
#define __LITTLE_ENDIAN	LITTLE_ENDIAN
#include <xfs/swab.h>

#define O_LARGEFILE     0
#ifndef O_DIRECT
#define O_DIRECT        0
#endif
#ifndef O_SYNC
#define O_SYNC          0
#endif

#define B_FALSE		0
#define B_TRUE		1

#define ENOATTR		989     /* Attribute not found */
#define EFSCORRUPTED	990	/* Filesystem is corrupted */
#define constpp		char * const *

#define HAVE_FID	1

#elif defined(__sgi__)	/* SGI IRIX */

#include <libgen.h>
#include <values.h>
#include <strings.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/statvfs.h>
#include <sys/sysmacros.h>
#include <uuid/uuid.h>

#define __s8		char
#define __s16		short
#define __s32		__int32_t
#define __s64		__int64_t
#define __u8		unsigned char
#define __u16		unsigned short
#define __u32		__int32_t
#define __u64		__int64_t
#define __int8_t	char
#define __int16_t	short
#define __uint8_t	unsigned char
#define __uint16_t	unsigned short
#define loff_t		off64_t
typedef off64_t		xfs_off_t;
typedef __int64_t	xfs_ino_t;
typedef __int32_t	xfs_dev_t;
typedef __int64_t	xfs_daddr_t;
typedef char*		xfs_caddr_t;

#include <sys/endian.h>
#define __BYTE_ORDER	BYTE_ORDER
#define __BIG_ENDIAN	BIG_ENDIAN
#define __LITTLE_ENDIAN	LITTLE_ENDIAN
#include <xfs/swab.h>

#define INT_MAX		INT32_MAX
#define UINT_MAX	UINT32_MAX
#define PATH_MAX	MAXPATHLEN
#define constpp		char * const *
#define isset(a,i)	((a)[(i)/NBBY] & (1<<((i)%NBBY)))

static __inline__ int xfsctl(const char *path, int fd, int cmd, void *p)
{
	fprintf(stderr, "%s: line %d syssgi/fcntl mapping not implemented\n",
			__FILE__, __LINE__);
	errno = ENOSYS;
	return -1;
}

static __inline__ int platform_test_xfs_fd(int fd)
{
	struct statvfs sbuf;
	if (fstatvfs(fd, &sbuf) < 0)
		return 0;
	return (strcmp(sbuf.f_basetype, "xfs") == 0);
}

static __inline__ int platform_test_xfs_path(const char *path)
{
	struct statvfs sbuf;
	if (statvfs(path, &sbuf) < 0)
		return 0;
	return (strcmp(sbuf.f_basetype, "xfs") == 0);
}

static __inline__ int platform_fstatfs(int fd, struct statfs *buf)
{
	return fstatfs(fd, buf, sizeof(struct statfs), 0);
}

static __inline__ char * strsep(char **s, const char *ct)
{
	char *sbegin = *s, *end;

	if (!sbegin)
		return NULL;
	end = strpbrk(sbegin, ct);
	if (end)
		*end++ = '\0';
	*s = end;
	return sbegin;
}

#define HAVE_DIOATTR	1
#define HAVE_FSXATTR	1
#define HAVE_GETBMAP	1
#define HAVE_GETBMAPX	1
#define HAVE_FSDMIDATA	1
#define HAVE_FID	1
#define HAVE_BBMACROS	1


#else
# error unknown platform... have fun porting!
#endif


/* long and pointer must be either 32 bit or 64 bit */
//#undef HAVE_32BIT_LONG
#undef HAVE_64BIT_LONG
//#undef HAVE_32BIT_PTR
#undef HAVE_64BIT_PTR

#if defined(HAVE_32BIT_LONG)
# define BITS_PER_LONG	32
#elif defined(HAVE_64BIT_LONG)
# define BITS_PER_LONG	64
#else
# error Unknown long size
#endif

/* Check if __psint_t is set to something meaningful */
#undef HAVE___PSINT_T
#ifndef HAVE___PSINT_T
# ifdef HAVE_32BIT_PTR
typedef int __psint_t;
# elif defined HAVE_64BIT_PTR
#  ifdef HAVE_64BIT_LONG
typedef long __psint_t;
#  else
/* This is a very strange architecture, which has 64 bit pointers but */
/* not 64 bit longs.  So, just punt here and assume long long is OK.  */
typedef long long __psint_t;
#  endif
# else
#  error Unknown pointer size
# endif
#endif

/* Check if __psunsigned_t is set to something meaningful */
#undef HAVE___PSUNSIGNED_T
#ifndef HAVE___PSUNSIGNED_T
# ifdef HAVE_32BIT_PTR
typedef unsigned int __psunsigned_t;
# elif defined HAVE_64BIT_PTR
#  ifdef HAVE_64BIT_LONG
typedef long __psunsigned_t;
#  else
/* This is a very strange architecture, which has 64 bit pointers but */
/* not 64 bit longs.  So, just punt here and assume long long is OK.  */
typedef unsigned long long __psunsigned_t;
#  endif
# else
#  error Unknown pointer size
# endif
#endif

/* Define if you want gettext (I18N) support */
#undef ENABLE_GETTEXT
#ifdef ENABLE_GETTEXT
# include <libintl.h>
# define _(x)                   gettext(x)
#else
# define _(x)                   (x)
# define textdomain(d)          do { } while (0)
# define bindtextdomain(d,dir)  do { } while (0)
#endif
#include <locale.h>

#ifdef DEBUG
# define ASSERT		assert
#else
# define ASSERT(EX)	((void) 0)
#endif

#define IRIX_DEV_BITSMAJOR      14
#define IRIX_DEV_BITSMINOR      18
#define IRIX_DEV_MAXMAJ         0x1ff
#define IRIX_DEV_MAXMIN         0x3ffff
#define IRIX_DEV_MAJOR(dev)	((int)(((unsigned)(dev) >> IRIX_DEV_BITSMINOR) \
					& IRIX_DEV_MAXMAJ))
#define IRIX_DEV_MINOR(dev)	((int)((dev) & IRIX_DEV_MAXMIN))
#define IRIX_MKDEV(major,minor)	((dev_t)(((major) << IRIX_DEV_BITSMINOR) \
					| (minor&IRIX_DEV_MAXMIN)))
#define IRIX_DEV_TO_KDEVT(dev)	makedev(IRIX_DEV_MAJOR(dev),IRIX_DEV_MINOR(dev))

#endif	/* __XFS_PLATFORM_DEFS_H__ */
