#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <alloca.h>
#include <inttypes.h>

#define BUF_SZ		65536
extern ssize_t readfile(const char *filename, char *buf, ssize_t size);

/* Find dev_t for e.g. "hda,NULL" or "hdb,2" */
static dev_t try_name(const char *name, int part)
{
	char path[BUF_SZ];
	char buf[BUF_SZ];
	int range;
	unsigned int major, minor;
	dev_t res;
	char *s;
	int len;
	int fd;

	/* read device number from /sys/block/.../dev */
	snprintf(path, sizeof(path), "/sys/block/%s/dev", name);
	len = readfile(path, buf, BUF_SZ);

	if (len <= 0 || len == BUF_SZ || buf[len - 1] != '\n')
		goto fail;
	buf[len - 1] = '\0';
	major = strtoul(buf, &s, 10);
	if (*s != ':')
		goto fail;
	minor = strtoul(s + 1, &s, 10);
	if (*s)
		goto fail;
	res = makedev(major, minor);
	/* if it's there and we are not looking for a partition - that's it */
	if (!part)
		return res;

	/* otherwise read range from .../range */
	snprintf(path, sizeof(path), "/sys/block/%s/range", name);
	fd = open(path, 0, 0);
	if (fd < 0)
		goto fail;
	len = read(fd, buf, 32);
	close(fd);
	if (len <= 0 || len == 32 || buf[len - 1] != '\n')
		goto fail;
	buf[len - 1] = '\0';
	range = strtoul(buf, &s, 10);
	if (*s)
		goto fail;
	/* if partition is within range - we got it */
	if (part < range) {
		return res + part;
	}

fail:
	return (dev_t) 0;
}

/*
 *	Convert a name into device number.  We accept the following variants:
 *
 *	1) device number in hexadecimal	represents itself
 *	2) device number in major:minor decimal represents itself
 *	3) /dev/<disk_name> represents the device number of disk
 *	4) /dev/<disk_name><decimal> represents the device number of partition - device number of disk plus the partition number
 *
 *	If name doesn't have fall into the categories above, we return 0.
 *	Driverfs is used to check if something is a disk name - it has
 *	all known disks under bus/block/devices.  If the disk name
 *	contains slashes, name of driverfs node has them replaced with
 *	dots.  try_name() does the actual checks, assuming that driverfs
 *	is mounted on rootfs /sys.
 */

static inline dev_t name_to_dev_t_real(const char *name)
{
	char *p, *s, *cptr, *e1, *e2;
	dev_t res = 0;
	int part, major, minor;
	struct stat st;
	const char *devname;
	char dname[32];

	if (name[0] == '/') {
		devname = name;
	} else {
		sprintf(dname, "/dev/%s", name);
		devname = dname;
	}

	if (!stat(devname, &st) && S_ISBLK(st.st_mode))
		return st.st_rdev;

	if (strncmp(name, "/dev/", 5)) {
		if ((cptr = strchr(devname+5, ':')) &&
		    cptr[1] != '\0') {
			/* Colon-separated decimal device number */
			*cptr = '\0';
			major = strtoul(devname+5, &e1, 10);
			minor = strtoul(cptr+1, &e2, 10);
			if (!*e1 && !*e2)
				return makedev(major, minor);
			*cptr = ':';
		} else {
			/* Hexadecimal device number */
			res = (dev_t) strtoul(name, &p, 16);
			if (!*p)
				return res;
		}
	} else {
		name += 5;
	}
	
	if (!strncmp(name, "ubi", 3))
		return __makedev(0, 254);
	
	/* Try block device, something like /dev/sda1, etc */
	s = name;
	for (p = name; *p; p++)
		if (*p == '/')
			*p = '!';
	res = try_name(name, 0);
	if (res)
		return res;

	while (p > s && isdigit(p[-1]))
		p--;
	if (p == s || !*p || *p == '0')
		goto fail;
	
	part = strtoul(p, NULL, 10);
	*p = '\0';
	res = try_name(s, part);
	if (res)
		return res;

	if (p < s + 2 || !isdigit(p[-2]) || p[-1] != 'p')
		goto fail;
	p[-1] = '\0';
	res = try_name(s, part);
	return res;
fail:
	return (dev_t) 0;
}

dev_t name_to_dev_t(const char *name)
{
	dev_t dev = name_to_dev_t_real(name);
	return dev;
}
