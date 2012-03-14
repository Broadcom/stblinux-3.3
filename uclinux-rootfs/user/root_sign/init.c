#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <sys/vfs.h>
#include <fcntl.h>
#include <dirent.h>
#include "sha1.h"

extern int do_mounts(int argc, char * argv[]);
static int nuke_dir(const char *what);

static const char * init_paths[] = {"/sbin/init", "/bin/init", "/etc/init", "/bin/sh", NULL};

extern char * get_arg(int argc, char *argv[], const char * name)
{
	int len = strlen(name);
	char * ret = NULL;
	int i;
	for (i = 0; i < argc; i++) {
		if(argv[i] && !strncmp(argv[i], name, len) && argv[i][len] != '\0') {
				ret = argv[i] + len;
				break;
		}
	}
	return ret;
}
				
ssize_t readfile(const char *filename, char *buf, ssize_t size)
{
	ssize_t val;
	FILE *f = fopen(filename, "r");

	if(!f) {
		return -EIO;
	}
	
	val = _fread(buf, size, f);
	fclose(f);

	return val;
}
static int mount_sys_fs(const char * path, const char * name, const char * type)
{
	struct stat st;
	if(stat(path, &st) == 0) {
		/* already mounted */
		return 0;
	}
	mkdir(name, 0555);
	if(mount("none", name, type, 0, NULL) == -1) {
		printf("%s: failed to mount %s as %s\n", __FUNCTION__, 
			name, type);
		return -1;
	}
	return 0;
	
}
static int process_cmdline(char * cmdline, char *argv[], int * argc)
{
	int space = 1;
	char * p = cmdline;
	int count=0;
	*argc = 0;

	while(p) {
		if(*p == 0x0a || *p == 0x0d ) {
			*p = '\0';
			break;
		}
		if(*p != 0x20 ) {
			if(space) {
				argv[*argc] = p;
				*argc += 1;
				space = 0;
			}
		}else {
			if(space == 0)
				*p = '\0';
			space = 1;
		}
		p++;
		count++;
	}
	return 0;
}
static const char * find_init(const char * root, const char * user)
{
	const char ** p;
	const char *init = 0;
	
	if(chdir(root)) {
		fprintf(stderr, "%s: can't chdir to %s\n", __FUNCTION__, root);
		return init;
	}
	for(p = init_paths; *p; p++) {
		fprintf(stdout, "Checking for %s\n", *p);
		if(!access(*p+1, X_OK)) {
			init = *p;
			break;
		}
	}
	chdir("/");
	return init;
	
}
static int nuke(const char *what)
{
	int rv;
	int err = 0;

	rv = unlink(what);
	if (rv < 0) {
		if (errno == EISDIR) {
			/* It's a directory. */
			err = nuke_dir(what);
			if (!err)
				err = rmdir(what) ? errno : err;
		} else {
			err = errno;
		}
	}

	if (err) {
		errno = err;
		return err;
	} else {
		return 0;
	}
}
static int nuke_dirent(int len, const char *dir, const char *name, dev_t me)
{
	int bytes = len + strlen(name) + 2;
	char path[bytes];
	int xlen;
	struct stat st;

	xlen = snprintf(path, bytes, "%s/%s", dir, name);
	//assert(xlen < bytes);
	if(xlen > bytes ) return -1;

	if (lstat(path, &st))
		return ENOENT;	/* Return 0 since already gone? */

	if (st.st_dev != me)
		return 0;	/* DO NOT recurse down mount points!!!!! */

	return nuke(path);
}

/* Wipe the contents of a directory, but not the directory itself */
static int nuke_dir(const char *what)
{
	int len = strlen(what);
	DIR *dir;
	struct dirent *d;
	int err = 0;
	struct stat st;
	if (lstat(what, &st))
		return errno;

	if (!S_ISDIR(st.st_mode))
		return ENOTDIR;

	if (!(dir = opendir(what))) {
		/* EACCES means we can't read it.  Might be empty and removable;
		   if not, the rmdir() in nuke() will trigger an error. */
		return (errno == EACCES) ? 0 : errno;
	}

	while ((d = readdir(dir))) {
		/* Skip . and .. */
		if (d->d_name[0] == '.' &&
		    (d->d_name[1] == '\0' ||
		     (d->d_name[1] == '.' && d->d_name[2] == '\0')))
			continue;

		err = nuke_dirent(len, what, d->d_name, st.st_dev);
		if (err) {
			closedir(dir);
			return err;
		}
	}

	closedir(dir);

	return 0;
}

const char *run_init(const char *realroot, const char *console,
		     const char *init, char **initargs)
{
	struct stat rst, cst, ist;
	struct statfs sfs;
	int confd;

	/* First, change to the new root directory */
	if (chdir(realroot))
		return "chdir to new root";

	/* Make sure the current directory is not on the same filesystem
	   as the root directory */
	if (stat("/", &rst) || stat(".", &cst))
		return "stat";

	if (rst.st_dev == cst.st_dev)
		return "current directory on the same filesystem as the root";

	/* The initramfs should have /init */
	if (stat("/init", &ist) || !S_ISREG(ist.st_mode))
		return "can't find /init on initramfs";

	/* Make sure we're on a ramfs */
	if (statfs("/", &sfs))
		return "statfs /";
	if (sfs.f_type != RAMFS_MAGIC && sfs.f_type != TMPFS_MAGIC)
		return "rootfs not a ramfs or tmpfs";

	/* Okay, I think we should be safe... */

	/* Delete rootfs contents */
	if (nuke_dir("/"))
		return "nuking initramfs contents";

	/* Overmount the root */
	if (mount(".", "/", NULL, MS_MOVE, NULL))
		return "overmounting root";

	/* chroot, chdir */
	if (chroot(".") || chdir("/"))
		return "chroot";

	/* Open /dev/console */
	if ((confd = open(console, O_RDWR)) < 0)
		return "opening console";
	dup2(confd, 0);
	dup2(confd, 1);
	dup2(confd, 2);
	close(confd);
	/* Spawn init */
	fprintf(stdout, "executing init from rootfs: %s \n", init);
	
	execv(init, initargs);

	fprintf(stderr, "failed to execute init\n");
	return init;		/* Failed to spawn init */
}
static int verify_signature(const char * path, const char * root)
{
	SHA1Context sha;
	struct stat st;
	char *buf, *line, *fname, *sigstr, signature_str[64];
	unsigned char *msgbuf, msgdigest[20];
	ssize_t bufsize;
	int i, ret = 0, bytes_read = 0;
	FILE *f = 0;
	
	if(stat(path, &st) < 0) {
		fprintf(stderr, "%s: failed to get file stat for %s\n", __FUNCTION__, path);
		return -1;
	}
	bufsize = (ssize_t)st.st_size;

	if((buf = (char *)malloc(bufsize)) == NULL) {
		fprintf(stderr, "%s: failed to alloc %d size memory\n", __FUNCTION__, bufsize);
		return -ENOMEM;
	}else if ((ret = readfile(path, buf, bufsize)) < 0) {
		fprintf(stderr, "%s: failed to read file %s\n", __FUNCTION__, path);
		goto error2;
	}

	if((msgbuf = (unsigned char *)malloc(64*BUFSIZ)) == NULL) {
		fprintf(stderr, "%s: failed to alloc %d bytes memory\n", __FUNCTION__, 64*BUFSIZ);
		ret = -1;
		goto error2;
	}
	line = buf;

	/* loop through each line in the file */
	while(line) {
		sigstr = strchr(line, ':');
		if(sigstr == NULL) {
			fprintf(stderr, "%s: invalid signature file\n", __FUNCTION__);
			ret = -EINVAL;
			break;
		}
		*sigstr = '\0';
		sigstr++;
		fprintf(stdout, "Processing %s..\n", line);
		/* The real root is mounted in "/root", and the do_mounts() did a chdir("/root"),
		 * at this point, so we strip out leading "/" in the signature list file.
		 */
		if(*line == '/')
			fname = line + 1;
		else
			fname = line;
		if((f = fopen(fname, "r")) == NULL) {
			fprintf(stderr, "failed to open file %s\n", line);
			ret = -EIO;
			break;
		}else if((ret = SHA1Reset(&sha))) {
			fprintf(stderr, "SHA1Reset error\n");
			break;
		}
		/* Compute SHA1 digest */
		do {
			bytes_read = fread(msgbuf, 1, 64*BUFSIZ, f);
			if((ret = SHA1Input(&sha, msgbuf, bytes_read))) {
				fclose(f);
				fprintf(stderr, "SHA1Input error\n");
				goto error1;
			}
		}while(bytes_read == 64*BUFSIZ);
		
		if((ret = SHA1Result(&sha, msgdigest))) {
			fprintf(stderr, "SHA1Result error\n");
			break;
		}
		for(i = 0; i < 20; i++)
			sprintf(&signature_str[i*2], "%02x",msgdigest[i]);
		/* compare with stored SHA1 digest */
		if(strncmp(sigstr, signature_str, 40)) {
			fprintf(stderr, "signature doesn't match for file: %s\n", line);
			ret = -EINVAL;
			break;
		}
		fclose(f);
		f = 0;
		
		line = strchr(sigstr, '\n');
		if(line == NULL) {
			fprintf(stderr, "File format wrong!\n");
			ret = -EINVAL;
			break;
		}
		if(buf+bufsize != line+1) 
			line++;
		else 
			line = NULL;
	}
	
	if(f) fclose(f);
error1:
	free(msgbuf);
error2:
	free(buf);
	return ret;

}
#define MAX_CMD_ARGS	16
const char * init_path = NULL;
int main(int argc, char *argv[])
{
	int fd = 0, procfs = 0, sysfs = 0, ret = 0;
	char cmdlines[BUFSIZ];
	char * cmds[MAX_CMD_ARGS];
	char ** init_argv;
	int cmd_argc;

	init_argv = (char **)malloc( (argc+1) * sizeof(char *));
	if(init_argv) {
		memcpy(init_argv, argv, (argc+1) * sizeof(char*));
	}else {
		fprintf(stderr, "%s: failed to alloc memory for init args\n", __FUNCTION__);
		return -ENOMEM;
	}
	if((fd = open("/dev/console", O_RDWR)) != -1) {
		dup2(fd, STDIN_FILENO);
		dup2(fd, STDOUT_FILENO);
		dup2(fd, STDERR_FILENO);

		if(fd > STDERR_FILENO) {
			close(fd);
		}
	}
	procfs = mount_sys_fs("/proc/cmdline", "/proc", "proc");
	if(procfs) {
		fprintf(stderr, "can not mount /proc\n");
		ret = -EFAULT;
		goto error;
	}
	/* sysfs used to figure out root device */
	sysfs = mount_sys_fs("/sys/bus", "/sys", "sysfs");
	if(sysfs) {
		fprintf(stderr, "can not mount /sys\n");
		ret = -EFAULT;
		goto error;
	}

	ret = readfile("/proc/cmdline", &cmdlines[0], BUFSIZ);
	if(ret < 0) {
		fprintf(stderr, "%s can not read /proc/cmdline\n", __FUNCTION__);
		ret = -EIO;
		goto error;
	}
	ret = process_cmdline(cmdlines, cmds, &cmd_argc);
	
	if(do_mounts(cmd_argc, cmds)) 
		goto error;
	if(procfs) {
		umount2("/proc", 0);
		procfs = 0;
	}
	if(sysfs) {
		umount2("/sys", 0);
		sysfs = 0;
	}
	/* chdir("/"); */
	fprintf(stdout, "Verifying signature for rootfs ...\n");
	
	if(verify_signature("/etc/signature.txt", "/root")) {
		fprintf(stderr, "Signature verification failure, system can't boot\n");
		ret = -EINVAL;
		goto error;
	}
	
	init_path = find_init("/root", 0);
	if(init_path == 0) {
		fprintf(stderr, "no init found\n");
		ret = -1;
		goto error;
	}
	/* get the real init first arg */
	init_argv[0] = strrchr(init_path, '/') + 1; 
	printf("running init\n");

	init_path = run_init("/root", "/dev/console", init_path, init_argv);
	
	return 0;

error:
	free(init_argv);
	if(procfs) umount2("/proc", 0);
	if(sysfs) umount2("/sys", 0);

	return ret;
}
