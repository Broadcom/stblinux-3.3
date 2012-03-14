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
 * or the like.	 Any license provided herein, whether implied or
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
 */

#include <xfs/libxfs.h>
#include <sys/stat.h>
#include "init.h"

#define findrawpath(x)	x
#define findblockpath(x) x

char *progname = "libxfs";	/* default, changed by each tool */

/*
 * dev_map - map open devices to fd.
 */
#define MAX_DEVS 10	/* arbitary maximum */
int nextfakedev = -1;	/* device number to give to next fake device */
static struct dev_to_fd {
	dev_t	dev;
	int	fd;
} dev_map[MAX_DEVS]={{0}};

/*
 * Checks whether a given device has a mounted, writable
 * filesystem, returns 1 if it does & fatal (just warns
 * if not fatal, but allows us to proceed).
 *
 * Useful to tools which will produce uncertain results
 * if the filesystem is active - repair, check, logprint.
 */
static int
check_isactive(char *name, char *block, int fatal)
{
	struct stat64	st;

	if (stat64(block, &st) < 0)
		return 0;
	if ((st.st_mode & S_IFMT) != S_IFBLK)
		return 0;
	if (platform_check_ismounted(name, block, &st, 0) == 0)
		return 0;
	return platform_check_iswritable(name, block, &st, fatal);
}

/* libxfs_device_to_fd:
 *     lookup a device number in the device map
 *     return the associated fd
 */
int
libxfs_device_to_fd(dev_t device)
{
	int	d;

	for (d = 0; d < MAX_DEVS; d++)
		if (dev_map[d].dev == device)
			return dev_map[d].fd;

	fprintf(stderr, _("%s: %s: device %lld is not open\n"),
		progname, __FUNCTION__, (long long)device);
	exit(1);
}

/* libxfs_device_open:
 *     open a device and return its device number
 */
dev_t
libxfs_device_open(char *path, int creat, int readonly, int setblksize)
{
	int		fd;
	dev_t		dev;
	int		d;
	struct stat	statb;

	if ((fd = open(path,
			(readonly ? O_RDONLY : O_RDWR) |
			(creat ? O_CREAT|O_TRUNC : 0),
			0666)) < 0) {
		fprintf(stderr, _("%s: cannot open %s: %s\n"),
			progname, path, strerror(errno));
		exit(1);
	}

	if (stat(path, &statb)<0) {
		fprintf(stderr, _("%s: cannot stat %s: %s\n"),
			progname, path, strerror(errno));
		exit(1);
	}

	if (!readonly && setblksize && (statb.st_mode & S_IFMT) == S_IFBLK) {
		platform_set_blocksize(fd, path, 512);
	}

	/*
	 * Get the device number from the stat buf - unless
	 * we're not opening a real device, in which case
	 * choose a new fake device number.
	 */
	dev = (statb.st_rdev) ? (statb.st_rdev) : (nextfakedev--);

	for (d = 0; d < MAX_DEVS; d++)
		if (dev_map[d].dev == dev) {
			fprintf(stderr, _("%s: device %lld is already open\n"),
			    progname, (long long)dev);
			exit(1);
		}

	for (d = 0; d < MAX_DEVS; d++)
		if (!dev_map[d].dev) {
			dev_map[d].dev = dev;
			dev_map[d].fd = fd;

			return dev;
		}

	fprintf(stderr, _("%s: %s: too many open devices\n"),
		progname, __FUNCTION__);
	exit(1);
}

void
libxfs_device_close(dev_t dev)
{
	int	d;

	for (d = 0; d < MAX_DEVS; d++)
		if (dev_map[d].dev == dev) {
			int	fd;

			fd = dev_map[d].fd;
			dev_map[d].dev = dev_map[d].fd = 0;

			fsync(fd);
			platform_flush_device(fd);
			close(fd);

			return;
		}

	fprintf(stderr, _("%s: %s: device %lld is not open\n"),
			progname, __FUNCTION__, (long long)dev);
	exit(1);
}

static int
check_open(char *path, int flags, char **rawfile, char **blockfile)
{
	int readonly = (flags & LIBXFS_ISREADONLY);
	int inactive = (flags & LIBXFS_ISINACTIVE);
	int dangerously = (flags & LIBXFS_DANGEROUSLY);
	struct stat64	stbuf;

	if (stat64(path, &stbuf) < 0) {
		perror(path);
		return 0;
	}
	if (!(*rawfile = findrawpath(path))) {
		fprintf(stderr, _("%s: "
				  "can't find a character device matching %s\n"),
			progname, path);
		return 0;
	}
	if (!(*blockfile = findblockpath(path))) {
		fprintf(stderr, _("%s: "
				  "can't find a block device matching %s\n"),
			progname, path);
		return 0;
	}
	if (!readonly && !inactive && platform_check_ismounted(path, *blockfile, NULL, 1))
		return 0;
	
	if (inactive && check_isactive(path, *blockfile, ((readonly|dangerously)?1:0)))
		return 0;

	return 1;
}

/*
 * libxfs initialization.
 * Caller gets a 0 on failure (and we print a message), 1 on success.
 */
int
libxfs_init(libxfs_init_t *a)
{
	char		*blockfile;
	char		curdir[MAXPATHLEN];
	char		*dname;
	char		dpath[25];
	int		fd;
	char		*logname;
	char		logpath[25];
	int		needcd;
	char		*rawfile;
	char		*rtname;
	char		rtpath[25];
	int		rval = 0;
	int		readonly;
	int		inactive;
	int		flags;

	dpath[0] = logpath[0] = rtpath[0] = '\0';
	dname = a->dname;
	logname = a->logname;
	rtname = a->rtname;
	a->dfd = a->logfd = a->rtfd = -1;
	a->ddev = a->logdev = a->rtdev = 0;
	a->dbsize = a->lbsize = a->rtbsize = 0;
	a->dsize = a->logBBsize = a->logBBstart = a->rtsize = 0;

	(void)getcwd(curdir,MAXPATHLEN);
	needcd = 0;
	fd = -1;
	readonly = (a->isreadonly & LIBXFS_ISREADONLY);
	inactive = (a->isreadonly & LIBXFS_ISINACTIVE);
	flags = a->isreadonly;

	if (a->volname) {
		if(!check_open(a->volname,flags,&rawfile,&blockfile))
			goto done;
		needcd = 1;
		fd = open(rawfile, O_RDONLY);
#ifdef HAVE_VOLUME_MANAGER
		xlv_getdev_t getdev;
		if (ioctl(fd, DIOCGETVOLDEV, &getdev) < 0)
#else
		if (1)
#endif
		{
			if (a->notvolok) {
				dname = a->dname = a->volname;
				a->volname = NULL;
				goto voldone;
			}
			fprintf(stderr, _("%s: "
				"%s is not a volume device name\n"),
				progname, a->volname);
			if (a->notvolmsg)
				fprintf(stderr, a->notvolmsg, a->volname);
			goto done;
		}
#ifdef HAVE_VOLUME_MANAGER
		if (getdev.data_subvol_dev && dname) {
			fprintf(stderr, _("%s: "
				"%s has a data subvolume, cannot specify %s\n"),
				progname, a->volname, dname);
			goto done;
		}
		if (getdev.log_subvol_dev && logname) {
			fprintf(stderr, _("%s: "
				"%s has a log subvolume, cannot specify %s\n"),
				progname, a->volname, logname);
			goto done;
		}
		if (getdev.rt_subvol_dev && rtname) {
			fprintf(stderr, _("%s: %s has a realtime subvolume, "
				"cannot specify %s\n"),
				progname, a->volname, rtname);
			goto done;
		}
		if (!dname && getdev.data_subvol_dev) {
			strcpy(dpath, "/tmp/libxfsdXXXXXX");
			(void)mktemp(dpath);
			if (mknod(dpath, S_IFCHR | 0600,
				  getdev.data_subvol_dev) < 0) {
				fprintf(stderr, _("%s: mknod failed: %s\n"),
					progname, strerror(errno));
				goto done;
			}
			dname = dpath;
		}
		if (!logname && getdev.log_subvol_dev) {
			strcpy(logpath, "/tmp/libxfslXXXXXX");
			(void)mktemp(logpath);
			if (mknod(logpath, S_IFCHR | 0600,
				  getdev.log_subvol_dev) < 0) {
				fprintf(stderr, _("%s: mknod failed: %s\n"),
					progname, strerror(errno));
				goto done;
			}
			logname = logpath;
		}
		if (!rtname && getdev.rt_subvol_dev) {
			strcpy(rtpath, "/tmp/libxfsrXXXXXX");
			(void)mktemp(rtpath);
			if (mknod(rtpath, S_IFCHR | 0600,
				  getdev.rt_subvol_dev) < 0) {
				fprintf(stderr, _("%s: mknod failed: %s\n"),
					progname, strerror(errno));
				goto done;
			}
			rtname = rtpath;
		}
#endif
	}
voldone:
	if (dname) {
		if (dname[0] != '/' && needcd)
			chdir(curdir);
		if (a->disfile) {
			a->ddev= libxfs_device_open(dname, a->dcreat, readonly,
						    a->setblksize);
			a->dfd = libxfs_device_to_fd(a->ddev);
		} else {
			if (!check_open(dname, flags, &rawfile, &blockfile))
				goto done;
			a->ddev = libxfs_device_open(rawfile,
					a->dcreat, readonly, a->setblksize);
			a->dfd = libxfs_device_to_fd(a->ddev);
			platform_findsizes(rawfile, a->dfd,
						&a->dsize, &a->dbsize);
		}
		needcd = 1;
	} else
		a->dsize = 0;
	if (logname) {
		if (logname[0] != '/' && needcd)
			chdir(curdir);
		if (a->lisfile) {
			a->logdev = libxfs_device_open(logname,
					a->lcreat, readonly, a->setblksize);
			a->logfd = libxfs_device_to_fd(a->logdev);
		} else {
			if (!check_open(logname, flags, &rawfile, &blockfile))
				goto done;
			a->logdev = libxfs_device_open(rawfile,
					a->lcreat, readonly, a->setblksize);
			a->logfd = libxfs_device_to_fd(a->logdev);
			platform_findsizes(rawfile, a->logfd,
						&a->logBBsize, &a->lbsize);
		}
		needcd = 1;
	} else
		a->logBBsize = 0;
	if (rtname) {
		if (rtname[0] != '/' && needcd)
			chdir(curdir);
		if (a->risfile) {
			a->rtdev = libxfs_device_open(rtname,
					a->rcreat, readonly, a->setblksize);
			a->rtfd = libxfs_device_to_fd(a->rtdev);
		} else {
			if (!check_open(rtname, flags, &rawfile, &blockfile))
				goto done;
			a->rtdev = libxfs_device_open(rawfile,
					a->rcreat, readonly, a->setblksize);
			a->rtfd = libxfs_device_to_fd(a->rtdev);
			platform_findsizes(rawfile, a->rtfd,
						&a->rtsize, &a->rtbsize);
		}
		needcd = 1;
	} else
		a->rtsize = 0;
	if (a->dsize < 0) {
		fprintf(stderr, _("%s: can't get size for data subvolume\n"),
			progname);
		goto done;
	}
	if (a->logBBsize < 0) {
		fprintf(stderr, _("%s: can't get size for log subvolume\n"),
			progname);
		goto done;
	}
	if (a->rtsize < 0) {
		fprintf(stderr, _("%s: can't get size for realtime subvolume\n"),
			progname);
		goto done;
	}
	if (needcd)
		chdir(curdir);
	rval = 1;
done:
	if (dpath[0])
		unlink(dpath);
	if (logpath[0])
		unlink(logpath);
	if (rtpath[0])
		unlink(rtpath);
	if (fd >= 0)
		close(fd);
	if (!rval && a->ddev)
		libxfs_device_close(a->ddev);
	if (!rval && a->logdev)
		libxfs_device_close(a->logdev);
	if (!rval && a->rtdev)
		libxfs_device_close(a->rtdev);
	return rval;
}


/*
 * Initialize/destroy all of the zone allocators we use.
 */
static void
manage_zones(int release)
{
	extern xfs_zone_t	*xfs_ili_zone;
	extern xfs_zone_t	*xfs_inode_zone;
	extern xfs_zone_t	*xfs_ifork_zone;
	extern xfs_zone_t	*xfs_dabuf_zone;
	extern xfs_zone_t	*xfs_buf_item_zone;
	extern xfs_zone_t	*xfs_da_state_zone;
	extern xfs_zone_t	*xfs_btree_cur_zone;
	extern xfs_zone_t	*xfs_bmap_free_item_zone;
	extern void		xfs_dir_startup();

	if (release) {	/* free zone allocation */
		libxfs_free(xfs_inode_zone);
		libxfs_free(xfs_ifork_zone);
		libxfs_free(xfs_dabuf_zone);
		libxfs_free(xfs_buf_item_zone);
		libxfs_free(xfs_da_state_zone);
		libxfs_free(xfs_btree_cur_zone);
		libxfs_free(xfs_bmap_free_item_zone);
		return;
	}
	/* otherwise initialise zone allocation */
	xfs_inode_zone = libxfs_zone_init(sizeof(xfs_inode_t), "xfs_inode");
	xfs_ifork_zone = libxfs_zone_init(sizeof(xfs_ifork_t), "xfs_ifork");
	xfs_dabuf_zone = libxfs_zone_init(sizeof(xfs_dabuf_t), "xfs_dabuf");
	xfs_ili_zone = libxfs_zone_init(
			sizeof(xfs_inode_log_item_t), "xfs_inode_log_item");
	xfs_buf_item_zone = libxfs_zone_init(
			sizeof(xfs_buf_log_item_t), "xfs_buf_log_item");
	xfs_da_state_zone = libxfs_zone_init(
			sizeof(xfs_da_state_t), "xfs_da_state");
	xfs_btree_cur_zone = libxfs_zone_init(
			sizeof(xfs_btree_cur_t), "xfs_btree_cur");
	xfs_bmap_free_item_zone = libxfs_zone_init(
			sizeof(xfs_bmap_free_item_t), "xfs_bmap_free_item");
	xfs_dir_startup();
}

/*
 * Get the bitmap and summary inodes into the mount structure
 * at mount time.
 */
static int
rtmount_inodes(xfs_mount_t *mp)
{
	int		error;
	xfs_sb_t	*sbp;

	sbp = &mp->m_sb;
	if (sbp->sb_rbmino == NULLFSINO)
		return 0;
	error = libxfs_iread(mp, NULL, sbp->sb_rbmino, &mp->m_rbmip, 0);
	if (error) {
		fprintf(stderr,
			_("%s: cannot read realtime bitmap inode (%d)\n"),
			progname, error);
		return error;
	}
	ASSERT(mp->m_rbmip != NULL);
	ASSERT(sbp->sb_rsumino != NULLFSINO);
	error = libxfs_iread(mp, NULL, sbp->sb_rsumino, &mp->m_rsumip, 0);
	if (error) {
		fprintf(stderr,
			_("%s: cannot read realtime summary inode (%d)\n"),
			progname, error);
		return error;
	}
	ASSERT(mp->m_rsumip != NULL);
	return 0;
}

/*
 * Initialize realtime fields in the mount structure.
 */
static int
rtmount_init(
	xfs_mount_t	*mp)	/* file system mount structure */
{
	xfs_buf_t	*bp;	/* buffer for last block of subvolume */
	xfs_daddr_t	d;	/* address of last block of subvolume */
	xfs_sb_t	*sbp;	/* filesystem superblock copy in mount */

	sbp = &mp->m_sb;
	if (sbp->sb_rblocks == 0)
		return 0;
	if (mp->m_rtdev == 0) {
		fprintf(stderr, _("%s: filesystem has a realtime subvolume\n"),
			progname);
		return -1;
	}
	mp->m_rsumlevels = sbp->sb_rextslog + 1;
	mp->m_rsumsize =
		(uint)sizeof(xfs_suminfo_t) * mp->m_rsumlevels *
		sbp->sb_rbmblocks;
	mp->m_rsumsize = roundup(mp->m_rsumsize, sbp->sb_blocksize);
	mp->m_rbmip = mp->m_rsumip = NULL;
	/*
	 * Check that the realtime section is an ok size.
	 */
	d = (xfs_daddr_t)XFS_FSB_TO_BB(mp, mp->m_sb.sb_rblocks);
	if (XFS_BB_TO_FSB(mp, d) != mp->m_sb.sb_rblocks) {
		fprintf(stderr, _("%s: realtime init - %llu != %llu\n"),
			progname, (unsigned long long) XFS_BB_TO_FSB(mp, d),
			(unsigned long long) mp->m_sb.sb_rblocks);
		return -1;
	}
	bp = libxfs_readbuf(mp->m_rtdev,
			d - XFS_FSB_TO_BB(mp, 1), XFS_FSB_TO_BB(mp, 1), 0);
	if (bp == NULL) {
		fprintf(stderr, _("%s: realtime size check failed\n"),
			progname);
		return -1;
	}
	libxfs_putbuf(bp);
	return 0;
}

#define XFS_MOUNT_32BITINODES	0x1
/*
 * Mount structure initialization, provides a filled-in xfs_mount_t
 * such that the numerous XFS_* macros can be used.  If dev is zero,
 * no IO will be performed (no size checks, read root inodes).
 */
xfs_mount_t *
libxfs_mount(
	xfs_mount_t	*mp,
	xfs_sb_t	*sb,
	dev_t		dev,
	dev_t		logdev,
	dev_t		rtdev,
	int		flags)
{
	xfs_daddr_t	d;
	xfs_buf_t	*bp;
	xfs_sb_t	*sbp;
	size_t		size;
	int		error;

	mp->m_dev = dev;
	mp->m_rtdev = rtdev;
	mp->m_logdev = logdev;
	mp->m_flags = XFS_MOUNT_32BITINODES;
	mp->m_sb = *sb;
	sbp = &(mp->m_sb);
	manage_zones(0);

	libxfs_mount_common(mp, sb);

	libxfs_alloc_compute_maxlevels(mp);
	libxfs_bmap_compute_maxlevels(mp, XFS_DATA_FORK);
	libxfs_bmap_compute_maxlevels(mp, XFS_ATTR_FORK);
	libxfs_ialloc_compute_maxlevels(mp);

	if (sbp->sb_imax_pct) {
		/* Make sure the maximum inode count is a multiple of the
		 * units we allocate inodes in.
		 */
		mp->m_maxicount = (sbp->sb_dblocks * sbp->sb_imax_pct) / 100;
		mp->m_maxicount = ((mp->m_maxicount / mp->m_ialloc_blks) *
				  mp->m_ialloc_blks)  << sbp->sb_inopblog;
	} else
		mp->m_maxicount = 0;

	mp->m_inode_cluster_size = XFS_INODE_BIG_CLUSTER_SIZE;

	/*
	 * Set whether we're using stripe alignment.
	 */
	if (XFS_SB_VERSION_HASDALIGN(&mp->m_sb)) {
		mp->m_dalign = sbp->sb_unit;
		mp->m_swidth = sbp->sb_width;
	}

	/*
	 * Set whether we're using inode alignment.
	 */
	if (XFS_SB_VERSION_HASALIGN(&mp->m_sb) &&
	    mp->m_sb.sb_inoalignmt >=
	    XFS_B_TO_FSBT(mp, mp->m_inode_cluster_size))
		mp->m_inoalign_mask = mp->m_sb.sb_inoalignmt - 1;
	else
		mp->m_inoalign_mask = 0;
	/*
	 * If we are using stripe alignment, check whether
	 * the stripe unit is a multiple of the inode alignment
	 */
	if (   mp->m_dalign
	    && mp->m_inoalign_mask && !(mp->m_dalign & mp->m_inoalign_mask))
		mp->m_sinoalign = mp->m_dalign;
	else
		mp->m_sinoalign = 0;

	/*
	 * Check that the data (and log if separate) are an ok size.
	 */
	d = (xfs_daddr_t) XFS_FSB_TO_BB(mp, mp->m_sb.sb_dblocks);
	if (XFS_BB_TO_FSB(mp, d) != mp->m_sb.sb_dblocks) {
		fprintf(stderr, _("%s: size check failed\n"), progname);
		if (!(flags & LIBXFS_MOUNT_DEBUGGER))
			return NULL;
	}

	/* Initialize the appropriate directory manager */
	if (XFS_SB_VERSION_HASDIRV2(sbp))
		libxfs_dir2_mount(mp);
	else
		libxfs_dir_mount(mp);

	/* Initialize the precomputed transaction reservations values */
	libxfs_trans_init(mp);

	if (dev == 0)	/* maxtrres, we have no device so leave now */
		return mp;

	bp = libxfs_readbuf(mp->m_dev,
			d - XFS_FSS_TO_BB(mp, 1), XFS_FSS_TO_BB(mp, 1),
			!(flags & LIBXFS_MOUNT_DEBUGGER));
	if (!bp) {
		fprintf(stderr, _("%s: data size check failed\n"), progname);
		if (!(flags & LIBXFS_MOUNT_DEBUGGER))
			return NULL;
	}
	libxfs_putbuf(bp);

	if (mp->m_logdev && mp->m_logdev != mp->m_dev) {
		d = (xfs_daddr_t) XFS_FSB_TO_BB(mp, mp->m_sb.sb_logblocks);
		if ( (XFS_BB_TO_FSB(mp, d) != mp->m_sb.sb_logblocks) ||
		     (!(bp = libxfs_readbuf(mp->m_logdev,
					d - XFS_FSB_TO_BB(mp, 1),
					XFS_FSB_TO_BB(mp, 1),
					!(flags & LIBXFS_MOUNT_DEBUGGER)))) ) {
			fprintf(stderr, _("%s: log size checks failed\n"),
					progname);
			if (!(flags & LIBXFS_MOUNT_DEBUGGER))
				return NULL;
		}
		libxfs_putbuf(bp);
	}

	/* Initialize realtime fields in the mount structure */
	if (!(flags & LIBXFS_MOUNT_DEBUGGER) && rtmount_init(mp)) {
		fprintf(stderr, _("%s: realtime device init failed\n"),
			progname);
			return NULL;
	}

	/* Allocate and initialize the per-ag data */
	size = sbp->sb_agcount * sizeof(xfs_perag_t);
	if (size && (mp->m_perag = calloc(size, 1)) == NULL) {
		fprintf(stderr, _("%s: failed to alloc %ld bytes: %s\n"),
			progname, (long)size, strerror(errno));
		exit(1);
	}

	libxfs_initialize_perag(mp, sbp->sb_agcount);

	/*
	 * mkfs calls mount before the root inode is allocated.
	 */
	if ((flags & LIBXFS_MOUNT_ROOTINOS) && sbp->sb_rootino != NULLFSINO) {
		error = libxfs_iread(mp, NULL, sbp->sb_rootino,
				&mp->m_rootip, 0);
		if (error) {
			fprintf(stderr, _("%s: cannot read root inode (%d)\n"),
				progname, error);
			if (!(flags & LIBXFS_MOUNT_DEBUGGER))
				return NULL;
		}
		ASSERT(mp->m_rootip != NULL);
	}
	if ((flags & LIBXFS_MOUNT_ROOTINOS) &&
	    !(flags & LIBXFS_MOUNT_DEBUGGER) &&
	    rtmount_inodes(mp))
		return NULL;
	return mp;
}

/*
 * Release any resource obtained during a mount.
 */
void
libxfs_umount(xfs_mount_t *mp)
{
	manage_zones(1);
	free(mp->m_perag);
}
