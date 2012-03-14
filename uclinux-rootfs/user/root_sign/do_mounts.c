#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <alloca.h>
#include <inttypes.h>

#define ROOT_UBI	__makedev(0, 254)
extern char * get_arg(int argc, const char * argv[], const char * name);
extern ssize_t readfile(const char * filename, char *buf, ssize_t);
extern dev_t name_to_dev_t(const char *name);

/* Create the device node "name" */
int create_dev(const char *name, dev_t dev)
{
	unlink(name);
	return mknod(name, S_IFBLK | 0600, dev);
}

/* mount a filesystem, possibly trying a set of different types */
const char *mount_block(const char *source, const char *target,
			const char *type, unsigned long flags,
			const void *data)
{
	int rv = 0;
	fprintf(stdout, "%s: trying to mount %s on %s with type %s\n",
	       __FUNCTION__, source, target, type);
	rv = mount(source, target, type, flags, data);
	/* Mount readonly if necessary */
	if (rv == -1 && errno == EACCES && !(flags & MS_RDONLY))
		rv = mount(source, target, type, flags | MS_RDONLY,
			   data);
	return rv ? NULL : type;

}

static int mount_block_root(int argc, const char *argv[], dev_t root_dev, const char *type, unsigned long flags)
{
	const char *data, *rp;

	data = get_arg(argc, argv, "rootflags=");
	create_dev("/dev/root", root_dev);

	errno = 0;

	if (type) {
		if ((rp = mount_block("/dev/root", "/root", type, flags, data)))
			goto ok;
		if (errno != EINVAL)
			goto bad;
	}

	if (!errno
	    && (rp = mount_block("/dev/root", "/root", NULL, flags, data)))
		goto ok;

bad:
	if (errno != EINVAL) {
		/*
		 * Allow the user to distinguish between failed open
		 * and bad superblock on root device.
		 */
		printf("%s: Cannot open root device %d\n",
			__FUNCTION__, root_dev);
		return -errno;
	} else {
		printf("%s: Unable to mount root fs on device %d\n",
			__FUNCTION__, root_dev);
		return -ESRCH;
	}

ok:
	printf("%s: Mounted root (%s filesystem)%s.\n",
	       __FUNCTION__, rp, (flags & MS_RDONLY) ? " readonly" : "");
	return 0;
}
static int mount_ubi_root(const char * root_dev_name, const char * type, unsigned long flags)
{
	char * data = NULL;
	if(!type)
		type = "ubifs";
	//flags = 0;
	if(mount(root_dev_name, "/root", type, flags, data)) {
		fprintf(stderr, "%s: Unable to mount MTD %s (%s filesystem) as root\n", 
			__FUNCTION__, root_dev_name, type);
		return -EFAULT;
	}else {
		fprintf(stderr, "%s: Mounted root (%s filesystem)%s.\n", 
			__FUNCTION__, type, (flags & MS_RDONLY)? "readonly" : "");
		return 0;
	}
	
}
int mount_root(int argc, const char * argv[], dev_t root_dev, const char *root_dev_name)
{
	int ret, i;
	unsigned long flags = MS_RDONLY | MS_VERBOSE;
	const char *type = get_arg(argc, argv, "rootfstype=");

	if(type == NULL) {
		fprintf(stderr, "Append proper root file system type rootfstype=\n");
		return -EINVAL;
	}
	for(i = 0; i < argc; i++) {
		if(argv[i] && strncmp(argv[i], "rw", 2) == 0)
			flags &= ~MS_RDONLY;
	}
	if( root_dev == ROOT_UBI) {
		fprintf(stdout, "Mounting ubifs on %s\n", root_dev_name);
		ret = mount_ubi_root(root_dev_name, "ubifs", flags);
	}else {
		fprintf(stdout, "Mounting %s on %s\n", type, root_dev_name);
		ret = mount_block_root(argc, argv, root_dev, type, flags);
	}

	if (!ret)
		chdir("/root");

	return ret;
}
int do_mounts(int argc, const char * argv[])
{
	const char *root_dev_name = get_arg(argc, argv, "root=");
	dev_t root_dev = 0;
	
	if (root_dev_name) {
		root_dev = name_to_dev_t(root_dev_name);
	} else {
		char rootdev[16];
		fprintf(stderr, "%s: no root specified in command line\n", __FUNCTION__);
		if(readfile("/proc/sys/kernel/real-root-dev", &rootdev[0], 16) <= 0) {
			fprintf(stderr, "can not read real-root-dev\n");
			return -EIO;
		}
		root_dev = (dev_t) strtoul(rootdev, NULL, 10);
	}

	fprintf(stdout, "init: root_dev = %s\n", root_dev_name);
	return mount_root(argc, argv, root_dev, root_dev_name);
}
