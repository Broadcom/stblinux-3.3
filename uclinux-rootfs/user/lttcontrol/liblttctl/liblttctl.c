/* libltt
 *
 * Linux Trace Toolkit Netlink Control Library
 *
 * Controls the ltt-control kernel module through debugfs.
 *
 * Copyright 2005 -
 * 	Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <liblttctl/lttctl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <limits.h>
#include <fcntl.h>
#include <stdlib.h>

#define MAX_CHANNEL	(256)

static char debugfsmntdir[PATH_MAX];

static int initdebugfsmntdir(void)
{
	return getdebugfsmntdir(debugfsmntdir);
}

/*
 * This function must called posterior to initdebugfsmntdir(),
 * because it need to use debugfsmntdir[] which is inited in initdebugfsmntdir()
 */
static int initmodule(void)
{
	char controldirname[PATH_MAX];
	DIR *dir;
	int tryload_done = 0;

	sprintf(controldirname, "%s/ltt/control/", debugfsmntdir);

check_again:
	/*
	 * Check ltt control's debugfs dir
	 *
	 * We don't check is ltt-trace-control module exist, because it maybe
	 * compiled into kernel.
	 */
	dir = opendir(controldirname);
	if (dir) {
		closedir(dir);
		return 0;
	}

	if (!tryload_done) {
		system("modprobe ltt-trace-control");
		tryload_done = 1;
		goto check_again;
	}

	return -ENOENT;
}

int lttctl_init(void)
{
	int ret;


	ret = initdebugfsmntdir();
	if (ret) {
		fprintf(stderr, "Get debugfs mount point failed\n");
		return ret;
	}

	ret = initmodule();
	if (ret) {
		fprintf(stderr, "Control module seems not work\n");
		return ret;
	}

	return 0;
}

int lttctl_destroy(void)
{
	return 0;
}

static int lttctl_sendop(const char *fname, const char *op)
{
	int fd;

	if (!fname) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		return 1;
	}

	fd = open(fname, O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "%s: open %s failed: %s\n", __func__, fname,
			strerror(errno));
		return errno;
	}

	if (write(fd, op, strlen(op)) == -1) {
		int ret = errno;
		fprintf(stderr, "%s: write %s to %s failed: %s\n", __func__, op,
			fname, strerror(errno));
		close(fd);
		return ret;
	}

	close(fd);

	return 0;
}

/*
 * check is trace exist(check debugfsmntdir too)
 * expect:
 *   0: expect that trace not exist
 *   !0: expect that trace exist
 *
 * ret:
 *   0: check pass
 *   -(EEXIST | ENOENT): check failed
 *   -ERRNO: error happened (no check)
 */
static int lttctl_check_trace(const char *name, int expect)
{
	char tracedirname[PATH_MAX];
	DIR *dir;
	int exist;

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		return -EINVAL;
	}

	if (!debugfsmntdir[0]) {
		fprintf(stderr, "%s: debugfsmntdir not valid\n", __func__);
		return -EINVAL;
	}

	sprintf(tracedirname, "%s/ltt/control/%s", debugfsmntdir, name);

	dir = opendir(tracedirname);
	if (dir) {
		exist = 1;
		closedir(dir);
	} else {
		if (errno != ENOENT) {
			fprintf(stderr, "%s: %s\n", __func__, strerror(errno));
			return -EINVAL;
		}
		exist = 0;
	}

	if (!expect != !exist) {
		if (exist)
		{
			fprintf(stderr, "Trace %s already exist\n", name);
			return -EEXIST;
		}
		else
		{
			fprintf(stderr, "Trace %s not exist\n", name);
			return -ENOENT;
		}
		
	}

	return 0;
}

/*
 * get channel list of a trace
 * don't include metadata channel when metadata is 0
 *
 * return number of channel on success
 * return negative number on fail
 * Caller must free channellist.
 */
static int lttctl_get_channellist(const char *tracename,
		char ***channellist, int metadata)
{
	char tracedirname[PATH_MAX];
	struct dirent *dirent;
	DIR *dir;
	char **list = NULL, **old_list;
	int nr_chan = 0;
	
	sprintf(tracedirname, "%s/ltt/control/%s/channel", debugfsmntdir,
		tracename);

	dir = opendir(tracedirname);
	if (!dir) {
		nr_chan = -ENOENT;
		goto error;
	}

	for (;;) {
		dirent = readdir(dir);
		if (!dirent)
			break;
		if (!strcmp(dirent->d_name, ".")
				|| !strcmp(dirent->d_name, ".."))
			continue;
		if (!metadata && !strcmp(dirent->d_name, "metadata"))
			continue;
		old_list = list;
		list = malloc(sizeof(char *) * ++nr_chan);
		memcpy(list, old_list, sizeof(*list) * (nr_chan - 1));
		free(old_list);
		list[nr_chan - 1] = strdup(dirent->d_name);
	}	

	closedir(dir);

	*channellist = list;
	return nr_chan;
error:
	free(list);
	*channellist = NULL;
	return nr_chan;
}

static void lttctl_free_channellist(char **channellist, int n_channel)
{
	int i = 0;
	for(; i < n_channel; ++i)
		free(channellist[i]);
	free(channellist);
}

int lttctl_setup_trace(const char *name)
{
	int ret;
	char ctlfname[PATH_MAX];

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 0);
	if (ret)
		goto arg_error;

	sprintf(ctlfname, "%s/ltt/setup_trace", debugfsmntdir);

	ret = lttctl_sendop(ctlfname, name);
	if (ret) {
		fprintf(stderr, "Setup trace failed\n");
		goto op_err;
	}

	return 0;

op_err:
arg_error:
	return ret;
}

int lttctl_destroy_trace(const char *name)
{
	int ret;
	char ctlfname[PATH_MAX];

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	sprintf(ctlfname, "%s/ltt/destroy_trace", debugfsmntdir);

	ret = lttctl_sendop(ctlfname, name);
	if (ret) {
		fprintf(stderr, "Destroy trace failed\n");
		goto op_err;
	}

	return 0;

op_err:
arg_error:
	return ret;
}

int lttctl_alloc_trace(const char *name)
{
	int ret;
	char ctlfname[PATH_MAX];

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	sprintf(ctlfname, "%s/ltt/control/%s/alloc", debugfsmntdir, name);

	ret = lttctl_sendop(ctlfname, "1");
	if (ret) {
		fprintf(stderr, "Allocate trace failed\n");
		goto op_err;
	}

	return 0;

op_err:
arg_error:
	return ret;
}

int lttctl_start(const char *name)
{
	int ret;
	char ctlfname[PATH_MAX];

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	sprintf(ctlfname, "%s/ltt/control/%s/enabled", debugfsmntdir, name);

	ret = lttctl_sendop(ctlfname, "1");
	if (ret) {
		fprintf(stderr, "Start trace failed\n");
		goto op_err;
	}

	return 0;

op_err:
arg_error:
	return ret;
}

int lttctl_pause(const char *name)
{
	int ret;
	char ctlfname[PATH_MAX];

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	sprintf(ctlfname, "%s/ltt/control/%s/enabled", debugfsmntdir, name);

	ret = lttctl_sendop(ctlfname, "0");
	if (ret) {
		fprintf(stderr, "Pause trace failed\n");
		goto op_err;
	}

	return 0;

op_err:
arg_error:
	return ret;
}

int lttctl_set_trans(const char *name, const char *trans)
{
	int ret;
	char ctlfname[PATH_MAX];

	if (!name) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	sprintf(ctlfname, "%s/ltt/control/%s/trans", debugfsmntdir, name);

	ret = lttctl_sendop(ctlfname, trans);
	if (ret) {
		fprintf(stderr, "Set transport failed\n");
		goto op_err;
	}

	return 0;

op_err:
arg_error:
	return ret;
}

static int __lttctl_set_channel_enable(const char *name, const char *channel,
		int enable)
{
	int ret;
	char ctlfname[PATH_MAX];

	sprintf(ctlfname, "%s/ltt/control/%s/channel/%s/enable", debugfsmntdir,
		name, channel);

	ret = lttctl_sendop(ctlfname, enable ? "1" : "0");
	if (ret)
		fprintf(stderr, "Set channel's enable mode failed\n");

	return ret;
}

int lttctl_set_channel_enable(const char *name, const char *channel,
		int enable)
{
	int ret;
	char **channellist;
	int n_channel;

	if (!name || !channel) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	if (strcmp(channel, "all")) {
		ret = __lttctl_set_channel_enable(name, channel, enable);
		if (ret)
			goto op_err;
	} else {
		/* Don't allow set enable state for metadata channel */
		n_channel = lttctl_get_channellist(name, &channellist, 0);
		if (n_channel < 0) {
			fprintf(stderr, "%s: lttctl_get_channellist failed\n",
				__func__);
			ret = -ENOENT;
			goto op_err;
		}

		for (; n_channel > 0; n_channel--) {
			ret = __lttctl_set_channel_enable(name,
				channellist[n_channel - 1], enable);
			if (ret)
				goto op_err_clean;
		}
		lttctl_free_channellist(channellist, n_channel);
	}

	return 0;

op_err_clean:
	lttctl_free_channellist(channellist, n_channel);
op_err:
arg_error:
	return ret;
}

static int __lttctl_set_channel_overwrite(const char *name, const char *channel,
		int overwrite)
{
	int ret;
	char ctlfname[PATH_MAX];

	sprintf(ctlfname, "%s/ltt/control/%s/channel/%s/overwrite",
		debugfsmntdir, name, channel);

	ret = lttctl_sendop(ctlfname, overwrite ? "1" : "0");
	if (ret)
		fprintf(stderr, "Set channel's overwrite mode failed\n");

	return ret;
}
int lttctl_set_channel_overwrite(const char *name, const char *channel,
		int overwrite)
{
	int ret;
	char **channellist;
	int n_channel;

	if (!name || !channel) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	if (strcmp(channel, "all")) {
		ret = __lttctl_set_channel_overwrite(name, channel, overwrite);
		if (ret)
			goto op_err;
	} else {
		/* Don't allow set overwrite for metadata channel */
		n_channel = lttctl_get_channellist(name, &channellist, 0);
		if (n_channel < 0) {
			fprintf(stderr, "%s: lttctl_get_channellist failed\n",
				__func__);
			ret = -ENOENT;
			goto op_err;
		}

		for (; n_channel > 0; n_channel--) {
			ret = __lttctl_set_channel_overwrite(name,
				channellist[n_channel - 1], overwrite);
			if (ret)
				goto op_err_clean;
		}
		lttctl_free_channellist(channellist, n_channel);
	}

	return 0;

op_err_clean:
	lttctl_free_channellist(channellist, n_channel);
op_err:
arg_error:
	return ret;
}

static int __lttctl_set_channel_subbuf_num(const char *name,
		const char *channel, unsigned subbuf_num)
{
	int ret;
	char ctlfname[PATH_MAX];
	char opstr[32];

	sprintf(ctlfname, "%s/ltt/control/%s/channel/%s/subbuf_num",
		debugfsmntdir, name, channel);

	sprintf(opstr, "%u", subbuf_num);

	ret = lttctl_sendop(ctlfname, opstr);
	if (ret)
		fprintf(stderr, "Set channel's subbuf number failed\n");

	return ret;
}
int lttctl_set_channel_subbuf_num(const char *name, const char *channel,
		unsigned subbuf_num)
{
	int ret;
	char **channellist;
	int n_channel;

	if (!name || !channel) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	if (strcmp(channel, "all")) {
		ret = __lttctl_set_channel_subbuf_num(name, channel,
			subbuf_num);
		if (ret)
			goto op_err;
	} else {
		/* allow set subbuf_num for metadata channel */
		n_channel = lttctl_get_channellist(name, &channellist, 1);
		if (n_channel < 0) {
			fprintf(stderr, "%s: lttctl_get_channellist failed\n",
				__func__);
			ret = -ENOENT;
			goto op_err;
		}

		for (; n_channel > 0; n_channel--) {
			ret = __lttctl_set_channel_subbuf_num(name,
				channellist[n_channel - 1], subbuf_num);
			if (ret)
				goto op_err_clean;
		}
		lttctl_free_channellist(channellist, n_channel);
	}

	return 0;

op_err_clean:
	lttctl_free_channellist(channellist, n_channel);
op_err:
arg_error:
	return ret;
}

static int __lttctl_set_channel_subbuf_size(const char *name,
		const char *channel, unsigned subbuf_size)
{
	int ret;
	char ctlfname[PATH_MAX];
	char opstr[32];

	sprintf(ctlfname, "%s/ltt/control/%s/channel/%s/subbuf_size",
		debugfsmntdir, name, channel);

	sprintf(opstr, "%u", subbuf_size);

	ret = lttctl_sendop(ctlfname, opstr);
	if (ret)
		fprintf(stderr, "Set channel's subbuf size failed\n");
	return ret;
}

int lttctl_set_channel_subbuf_size(const char *name, const char *channel,
		unsigned subbuf_size)
{
	int ret;
	char **channellist;
	int n_channel;

	if (!name || !channel) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	if (strcmp(channel, "all")) {
		ret = __lttctl_set_channel_subbuf_size(name, channel,
			subbuf_size);
		if (ret)
			goto op_err;
	} else {
		/* allow set subbuf_size for metadata channel */
		n_channel = lttctl_get_channellist(name, &channellist, 1);
		if (n_channel < 0) {
			fprintf(stderr, "%s: lttctl_get_channellist failed\n",
				__func__);
			ret = -ENOENT;
			goto op_err;
		}

		for (; n_channel > 0; n_channel--) {
			ret = __lttctl_set_channel_subbuf_size(name,
				channellist[n_channel - 1], subbuf_size);
			if (ret)
				goto op_err_clean;
		}
		lttctl_free_channellist(channellist, n_channel);
	}

	return 0;

op_err_clean:
	lttctl_free_channellist(channellist, n_channel);
op_err:
arg_error:
	return ret;
}

static int __lttctl_set_channel_switch_timer(const char *name,
		const char *channel, unsigned switch_timer)
{
	int ret;
	char ctlfname[PATH_MAX];
	char opstr[32];

	sprintf(ctlfname, "%s/ltt/control/%s/channel/%s/switch_timer",
		debugfsmntdir, name, channel);

	sprintf(opstr, "%u", switch_timer);

	ret = lttctl_sendop(ctlfname, opstr);
	if (ret)
		fprintf(stderr, "Set channel's switch timer failed\n");
	return ret;
}

int lttctl_set_channel_switch_timer(const char *name, const char *channel,
		unsigned switch_timer)
{
	int ret;
	char **channellist;
	int n_channel;

	if (!name || !channel) {
		fprintf(stderr, "%s: args invalid\n", __func__);
		ret = -EINVAL;
		goto arg_error;
	}

	ret = lttctl_check_trace(name, 1);
	if (ret)
		goto arg_error;

	if (strcmp(channel, "all")) {
		ret = __lttctl_set_channel_subbuf_size(name, channel,
			switch_timer);
		if (ret)
			goto op_err;
	} else {
		/* allow set subbuf_size for metadata channel */
		n_channel = lttctl_get_channellist(name, &channellist, 1);
		if (n_channel < 0) {
			fprintf(stderr, "%s: lttctl_get_channellist failed\n",
				__func__);
			ret = -ENOENT;
			goto op_err;
		}

		for (; n_channel > 0; n_channel--) {
			ret = __lttctl_set_channel_switch_timer(name,
				channellist[n_channel - 1], switch_timer);
			if (ret)
				goto op_err_clean;
		}
		lttctl_free_channellist(channellist, n_channel);
	}

	return 0;

op_err_clean:
	lttctl_free_channellist(channellist, n_channel);
op_err:
arg_error:
	return ret;
}

int getdebugfsmntdir(char *mntdir)
{
	char mnt_dir[PATH_MAX];
	char mnt_type[PATH_MAX];
	int trymount_done = 0;

	FILE *fp = fopen("/proc/mounts", "r");
	if (!fp)
		return -EINVAL;

find_again:
	while (1) {
		if (fscanf(fp, "%*s %s %s %*s %*s %*s", mnt_dir, mnt_type) <= 0)
			break;

		if (!strcmp(mnt_type, "debugfs")) {
			strcpy(mntdir, mnt_dir);
			return 0;
		}
	}

	if (!trymount_done) {
		mount("debugfs", "/sys/kernel/debug/", "debugfs", 0, NULL);
		trymount_done = 1;
		goto find_again;
	}

	return -ENOENT;
}
