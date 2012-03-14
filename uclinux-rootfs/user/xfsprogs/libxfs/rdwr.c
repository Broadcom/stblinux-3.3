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
#include <xfs/xfs_log.h>
#include <xfs/xfs_log_priv.h>

#define BBTOOFF64(bbs)	(((xfs_off_t)(bbs)) << BBSHIFT)
#define BDSTRAT_SIZE	(256 * 1024)
#define min(x, y)	((x) < (y) ? (x) : (y))

void
libxfs_device_zero(dev_t dev, xfs_daddr_t start, uint len)
{
	xfs_off_t	start_offset, end_offset, offset;
	ssize_t		zsize, bytes;
	char		*z;
	int		fd;

	zsize = min(BDSTRAT_SIZE, BBTOB(len));
	if ((z = memalign(getpagesize(), zsize)) == NULL) {
		fprintf(stderr,
			_("%s: %s can't memalign %d bytes: %s\n"),
			progname, __FUNCTION__, (int)zsize, strerror(errno));
		exit(1);
	}
	memset(z, 0, zsize);

	fd = libxfs_device_to_fd(dev);
	start_offset = BBTOOFF64(start);

	if ((lseek64(fd, start_offset, SEEK_SET)) < 0) {
		fprintf(stderr, _("%s: %s seek to offset %llu failed: %s\n"),
			progname, __FUNCTION__,
			(unsigned long long)start_offset, strerror(errno));
		exit(1);
	}

	end_offset = BBTOOFF64(start + len) - start_offset;
	for (offset = 0; offset < end_offset; ) {
		bytes = min((ssize_t)(end_offset - offset), zsize);
		if ((bytes = write(fd, z, bytes)) < 0) {
			fprintf(stderr, _("%s: %s write failed: %s\n"),
				progname, __FUNCTION__, strerror(errno));
			exit(1);
		} else if (bytes == 0) {
			fprintf(stderr, _("%s: %s not progressing?\n"),
				progname, __FUNCTION__);
			exit(1);
		}
		offset += bytes;
	}
	free(z);
}

static void unmount_record(void *p)
{
	xlog_op_header_t	*op = (xlog_op_header_t *)p;
	/* the data section must be 32 bit size aligned */
	struct {
	    __uint16_t magic;
	    __uint16_t pad1;
	    __uint32_t pad2; /* may as well make it 64 bits */
	} magic = { XLOG_UNMOUNT_TYPE, 0, 0 };

	memset(p, 0, BBSIZE);
	INT_SET(op->oh_tid,		ARCH_CONVERT, 1);
	INT_SET(op->oh_len,		ARCH_CONVERT, sizeof(magic));
	INT_SET(op->oh_clientid,	ARCH_CONVERT, XFS_LOG);
	INT_SET(op->oh_flags,		ARCH_CONVERT, XLOG_UNMOUNT_TRANS);
	INT_SET(op->oh_res2,		ARCH_CONVERT, 0);

	/* and the data for this op */
	memcpy(p + sizeof(xlog_op_header_t), &magic, sizeof(magic));
}

static xfs_caddr_t next(xfs_caddr_t ptr, int offset, void *private)
{
	xfs_buf_t	*buf = (xfs_buf_t *)private;

	if (XFS_BUF_COUNT(buf) < (int)(ptr - XFS_BUF_PTR(buf)) + offset)
		abort();
	return ptr + offset;
}

int
libxfs_log_clear(
	dev_t			device,
	xfs_daddr_t		start,
	uint			length,
	uuid_t			*fs_uuid,
	int			version,
	int			sunit,
	int			fmt)
{
	xfs_buf_t		*buf;
	int			len;

	if (!device || !fs_uuid)
		return -EINVAL;

	/* first zero the log */
	libxfs_device_zero(device, start, length);

	/* then write a log record header */
	len = ((version == 2) && sunit) ? BTOBB(sunit) : 2;
	len = MAX(len, 2);
	buf = libxfs_getbuf(device, start, len);
	if (!buf)
		return -1;
	libxfs_log_header(XFS_BUF_PTR(buf),
			  fs_uuid, version, sunit, fmt, next, buf);
	if (libxfs_writebuf(buf, 0))
		return -1;

	return 0;
}

int
libxfs_log_header(
	xfs_caddr_t		caddr,
	uuid_t			*fs_uuid,
	int			version,
	int			sunit,
	int			fmt,
	libxfs_get_block_t	*nextfunc,
	void			*private)
{
	xlog_rec_header_t	*head = (xlog_rec_header_t *)caddr;
	xfs_caddr_t		p = caddr;
	uint			cycle_lsn;
	int			i, len;

	len = ((version == 2) && sunit) ? BTOBB(sunit) : 1;

	/* note that oh_tid actually contains the cycle number
	 * and the tid is stored in h_cycle_data[0] - that's the
	 * way things end up on disk.
	 */
	memset(p, 0, BBSIZE);
	INT_SET(head->h_magicno,	ARCH_CONVERT, XLOG_HEADER_MAGIC_NUM);
	INT_SET(head->h_cycle,		ARCH_CONVERT, 1);
	INT_SET(head->h_version,	ARCH_CONVERT, version);
	if (len != 1)
		INT_SET(head->h_len,		ARCH_CONVERT, sunit - BBSIZE);
	else
		INT_SET(head->h_len,		ARCH_CONVERT, 20);
	INT_SET(head->h_chksum,		ARCH_CONVERT, 0);
	INT_SET(head->h_prev_block,	ARCH_CONVERT, -1);
	INT_SET(head->h_num_logops,	ARCH_CONVERT, 1);
	INT_SET(head->h_cycle_data[0],	ARCH_CONVERT, 0xb0c0d0d0);
	INT_SET(head->h_fmt,		ARCH_CONVERT, fmt);
	INT_SET(head->h_size,		ARCH_CONVERT, XLOG_HEADER_CYCLE_SIZE);

	ASSIGN_ANY_LSN(head->h_lsn,	    1, 0, ARCH_CONVERT);
	ASSIGN_ANY_LSN(head->h_tail_lsn,    1, 0, ARCH_CONVERT);

	memcpy(&head->h_fs_uuid, fs_uuid, sizeof(uuid_t));

	len = MAX(len, 2);
	p = nextfunc(p, BBSIZE, private);
	unmount_record(p);

	cycle_lsn = CYCLE_LSN_NOCONV(head->h_lsn, ARCH_CONVERT);
	for (i = 2; i < len; i++) {
		p = nextfunc(p, BBSIZE, private);
		memset(p, 0, BBSIZE);
		*(uint *)p = cycle_lsn;
	}

	return BBTOB(len);
}


/*
 * Simple I/O interface
 */

xfs_buf_t *
libxfs_getbuf(dev_t device, xfs_daddr_t blkno, int len)
{
	xfs_buf_t	*buf;
	size_t		total;

	total = sizeof(xfs_buf_t) + BBTOB(len);
	if ((buf = calloc(total, 1)) == NULL) {
		fprintf(stderr, _("%s: buf calloc failed (%ld bytes): %s\n"),
			progname, (long)total, strerror(errno));
		exit(1);
	}
	/* by default, we allocate buffer directly after the header */
	buf->b_blkno = blkno;
	buf->b_bcount = BBTOB(len);
	buf->b_dev = device;
	buf->b_addr = (char *)(&buf->b_addr + 1);	/* must be last field */
#ifdef IO_DEBUG
	fprintf(stderr, "getbuf allocated %ubytes, blkno=%llu(%llu), %p\n",
		BBTOB(len), BBTOOFF64(blkno), blkno, buf);
#endif

	return(buf);
}

int
libxfs_readbufr(dev_t dev, xfs_daddr_t blkno, xfs_buf_t *buf, int len, int flags)
{
	int	fd = libxfs_device_to_fd(dev);

	buf->b_dev = dev;
	buf->b_blkno = blkno;
	ASSERT(BBTOB(len) <= buf->b_bcount);

	if (pread64(fd, buf->b_addr, BBTOB(len), BBTOOFF64(blkno)) < 0) {
		fprintf(stderr, _("%s: read failed: %s\n"),
			progname, strerror(errno));
		if (flags & LIBXFS_EXIT_ON_FAILURE)
			exit(1);
		return errno;
	}
#ifdef IO_DEBUG
	fprintf(stderr, "readbufr read %ubytes, blkno=%llu(%llu), %p\n",
		BBTOB(len), BBTOOFF64(blkno), blkno, buf);
#endif
	return 0;
}

xfs_buf_t *
libxfs_readbuf(dev_t dev, xfs_daddr_t blkno, int len, int flags)
{
	xfs_buf_t	*buf;
	int		error;

	buf = libxfs_getbuf(dev, blkno, len);
	error = libxfs_readbufr(dev, blkno, buf, len, flags);
	if (error) {
		libxfs_putbuf(buf);
		return NULL;
	}
	return buf;
}

xfs_buf_t *
libxfs_getsb(xfs_mount_t *mp, int flags)
{
	return libxfs_readbuf(mp->m_dev, XFS_SB_DADDR,
				XFS_FSB_TO_BB(mp, 1), flags);
}

int
libxfs_writebuf_int(xfs_buf_t *buf, int flags)
{
	int	sts;
	int	fd = libxfs_device_to_fd(buf->b_dev);

#ifdef IO_DEBUG
	fprintf(stderr, "writing %ubytes at blkno=%llu(%llu), %p\n",
		buf->b_bcount, BBTOOFF64(buf->b_blkno), buf->b_blkno, buf);
#endif
	sts = pwrite64(fd, buf->b_addr, buf->b_bcount, BBTOOFF64(buf->b_blkno));
	if (sts < 0) {
		fprintf(stderr, _("%s: pwrite64 failed: %s\n"),
			progname, strerror(errno));
		if (flags & LIBXFS_EXIT_ON_FAILURE)
			exit(1);
		return errno;
	}
	else if (sts != buf->b_bcount) {
		fprintf(stderr, _("%s: error - wrote only %d of %d bytes\n"),
			progname, sts, buf->b_bcount);
		if (flags & LIBXFS_EXIT_ON_FAILURE)
			exit(1);
		return EIO;
	}
	return 0;
}

int
libxfs_writebuf(xfs_buf_t *buf, int flags)
{
	int error = libxfs_writebuf_int(buf, flags);
	libxfs_putbuf(buf);
	return error;
}

void
libxfs_putbuf(xfs_buf_t *buf)
{
	if (buf != NULL) {
		xfs_buf_log_item_t	*bip;
		extern xfs_zone_t	*xfs_buf_item_zone;

		bip = XFS_BUF_FSPRIVATE(buf, xfs_buf_log_item_t *);

		if (bip)
		    libxfs_zone_free(xfs_buf_item_zone, bip);
#ifdef IO_DEBUG
		fprintf(stderr, "putbuf released %ubytes, %p\n",
			buf->b_bcount, buf);
#endif
		free(buf);
		buf = NULL;
	}
}


/*
 * Simple memory interface
 */

xfs_zone_t *
libxfs_zone_init(int size, char *name)
{
	xfs_zone_t	*ptr;

	if ((ptr = malloc(sizeof(xfs_zone_t))) == NULL) {
		fprintf(stderr, _("%s: zone init failed (%s, %d bytes): %s\n"),
			progname, name, (int)sizeof(xfs_zone_t), strerror(errno));
		exit(1);
	}
	ptr->zone_unitsize = size;
	ptr->zone_name = name;
#ifdef MEM_DEBUG
	ptr->allocated = 0;
	fprintf(stderr, "new zone %p for \"%s\", size=%d\n", ptr, name, size);
#endif
	return ptr;
}

void *
libxfs_zone_zalloc(xfs_zone_t *z)
{
	void	*ptr;

	if ((ptr = calloc(z->zone_unitsize, 1)) == NULL) {
		fprintf(stderr, _("%s: zone calloc failed (%s, %d bytes): %s\n"),
			progname, z->zone_name, z->zone_unitsize,
			strerror(errno));
		exit(1);
	}
#ifdef MEM_DEBUG
	z->allocated++;
	fprintf(stderr, "## zone alloc'd item %p from %s (%d bytes) (%d active)\n",
		ptr, z->zone_name,  z->zone_unitsize,
		z->allocated);
#endif
	return ptr;
}

void
libxfs_zone_free(xfs_zone_t *z, void *ptr)
{
#ifdef MEM_DEBUG
	z->allocated--;
	fprintf(stderr, "## zone freed item %p from %s (%d bytes) (%d active)\n",
		ptr, z->zone_name, z->zone_unitsize,
		z->allocated);
#endif
	if (ptr != NULL) {
		free(ptr);
		ptr = NULL;
	}
}

void *
libxfs_malloc(size_t size)
{
	void	*ptr;

	if ((ptr = calloc(1, size)) == NULL) {
		fprintf(stderr, _("%s: calloc failed (%d bytes): %s\n"),
			progname, (int)size, strerror(errno));
		exit(1);
	}
#ifdef MEM_DEBUG
	fprintf(stderr, "## calloc'd item %p size %d bytes\n",
		ptr, size);
#endif
	return ptr;
}

void
libxfs_free(void *ptr)
{
#ifdef MEM_DEBUG
	fprintf(stderr, "## freed item %p\n",
		ptr);
#endif
	if (ptr != NULL) {
		free(ptr);
		ptr = NULL;
	}
}

void *
libxfs_realloc(void *ptr, size_t size)
{
#ifdef MEM_DEBUG
	void *optr=ptr;
#endif
	if ((ptr = realloc(ptr, size)) == NULL) {
		fprintf(stderr, _("%s: realloc failed (%d bytes): %s\n"),
			progname, (int)size, strerror(errno));
		exit(1);
	}
#ifdef MEM_DEBUG
	fprintf(stderr, "## realloc'd item %p now %p size %d bytes\n",
		optr, ptr, size);
#endif
	return ptr;
}


int
libxfs_iget(xfs_mount_t *mp, xfs_trans_t *tp, xfs_ino_t ino, uint lock_flags,
		xfs_inode_t **ipp, xfs_daddr_t bno)
{
	xfs_inode_t	*ip;
	int		error;

	error = libxfs_iread(mp, tp, ino, &ip, bno);
	if (error)
		return error;
	*ipp = ip;
	return 0;
}

void
libxfs_iput(xfs_inode_t *ip, uint lock_flags)
{
	extern xfs_zone_t	*xfs_ili_zone;
	extern xfs_zone_t	*xfs_inode_zone;

	if (ip != NULL) {

		/* free attached inode log item */
		if (ip->i_itemp)
			libxfs_zone_free(xfs_ili_zone, ip->i_itemp);
		ip->i_itemp = NULL;

		libxfs_zone_free(xfs_inode_zone, ip);
		ip = NULL;
	}
}

/*
 * libxfs_mod_sb can be used to copy arbitrary changes to the
 * in-core superblock into the superblock buffer to be logged.
 *
 * In user-space, we simply convert to big-endian, and write the
 * the whole superblock - the in-core changes have all been made
 * already.
 */
void
libxfs_mod_sb(xfs_trans_t *tp, __int64_t fields)
{
	xfs_buf_t	*bp;
	xfs_mount_t	*mp;

	mp = tp->t_mountp;
	bp = libxfs_getbuf(mp->m_dev, XFS_SB_DADDR, 1);
	libxfs_xlate_sb(XFS_BUF_PTR(bp), &mp->m_sb, -1, ARCH_CONVERT,
			XFS_SB_ALL_BITS);
	libxfs_writebuf(bp, LIBXFS_EXIT_ON_FAILURE);
}
