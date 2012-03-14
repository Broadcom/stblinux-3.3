/*---------------------------------------------------------------------------

    Copyright (c) 2001-2007 Broadcom Corporation                 /\
                                                          _     /  \     _
    _____________________________________________________/ \   /    \   / \_
                                                            \_/      \_/  

 Copyright (c) 2007 Broadcom Corporation
 All rights reserved.
 
 Redistribution and use of this software in source and binary forms, with or
 without modification, are permitted provided that the following conditions
 are met:
 
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 
 * Neither the name of Broadcom Corporation nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission of Broadcom Corporation.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 File: pmlib.c

 Description:
 Power management API for Broadcom STB/DTV peripherals

    when        who         what
    -----       ---         ----
    20071030    cernekee    initial version
 ------------------------------------------------------------------------- */

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <glob.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/sockios.h>
#include <linux/ethtool.h>
#include <linux/if.h>

#include <pmlib.h>

struct brcm_pm_priv
{
	struct brcm_pm_state	last_state;
	struct brcm_pm_cfg	cfg;
	int			has_eth1;
};

#define BUF_SIZE	64
#define MAX_ARGS	16

#define SYS_MEMC1_STAT	"/sys/devices/platform/brcmstb/memc1_power"
#define SYS_SATA_STAT	"/sys/devices/platform/brcmstb/sata_power"
#define SYS_DDR_STAT	"/sys/devices/platform/brcmstb/ddr_timeout"
#define SYS_STANDBY_FLAGS "/sys/devices/platform/brcmstb/standby_flags"
#define SYS_TP1_STAT	"/sys/devices/system/cpu/cpu1/online"
#define SYS_CPU_KHZ	"/sys/devices/platform/brcmstb/cpu_khz"
#define SYS_CPU_PLL	"/sys/devices/platform/brcmstb/cpu_pll"
#define SYS_CPU_DIV	"/sys/devices/platform/brcmstb/cpu_div"
#define SYS_STANDBY	"/sys/power/state"
#define HALT_PATH	"/sbin/halt"
#define SATA_RESCAN_GLOB "/sys/class/scsi_host/host*/scan"
#define SATA_DEVICE_GLOB "/sys/class/scsi_device/*/device/block:*"
#define SATA_DELETE_GLOB "/sys/class/scsi_device/*/device/delete"

static int sysfs_get(char *path, unsigned int *out)
{
	FILE *f;
	unsigned int tmp;
	char buf[BUF_SIZE];

	f = fopen(path, "r");
	if(! f)
		return(-1);
	if(fgets(buf, BUF_SIZE, f) != buf)
	{
		fclose(f);
		return(-1);
	}
	fclose(f);
	if(sscanf(buf, "0x%x", &tmp) != 1 && sscanf(buf, "%u", &tmp) != 1)
		return(-1);
	*out = tmp;
	return(0);
}

static int sysfs_set(char *path, int in)
{
	FILE *f;
	char buf[BUF_SIZE];

	f = fopen(path, "w");
	if(! f)
		return(-1);
	sprintf(buf, "%u", in);
	if((fputs(buf, f) < 0) || (fflush(f) < 0))
	{
		fclose(f);
		return(-1);
	}
	fclose(f);
	return(0);
}

static int sysfs_set_string(char *path, const char *in)
{
	FILE *f;

	f = fopen(path, "w");
	if(! f)
		return(-1);
	if((fputs(in, f) < 0) || (fflush(f) < 0))
	{
		fclose(f);
		return(-1);
	}
	fclose(f);
	return(0);
}

static int sysfs_get_string(char *path, char *in, int size)
{
	FILE *f;

	f = fopen(path, "r");
	if (!f)
		return(-1);
	if (fgets(in, size, f) < 0)
	{
		fclose(f);
		return(-1);
	}
	fclose(f);
	return(0);
}

static int run(char *prog, ...)
{
	va_list ap;
	int status, i = 1;
	pid_t pid;
	char *args[MAX_ARGS], *a;

	va_start(ap, prog);

	pid = fork();
	if(pid < 0)
		return(-1);
	
	if(pid != 0)
	{
		wait(&status);
		va_end(ap);
		return(WEXITSTATUS(status) ? -1 : 0);
	}

	/* child */
	args[0] = prog;
	do
	{
		a = va_arg(ap, char *);
		args[i++] = a;
	} while(a);

	execv(prog, args);
	_exit(1);

	va_end(ap);	/* never reached */
	return(0);
}

static int brcm_pm_eth1_check(void)
{
	struct ifreq ifr;
	struct ethtool_drvinfo drvinfo;
	int ret = 0;
	int fd;

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(fd < 0)
		return(0);

	memset(&ifr, 0, sizeof(ifr));
	memset(&drvinfo, 0, sizeof(drvinfo));

	drvinfo.cmd = ETHTOOL_GDRVINFO;
	ifr.ifr_data = (caddr_t)&drvinfo;
	strcpy(ifr.ifr_name, "eth1");

	if(ioctl(fd, SIOCETHTOOL, &ifr) == 0) {
		if(strcmp(drvinfo.driver, "BCMINTMAC") == 0)
			ret = 1;
		if(strcmp(drvinfo.driver, "BCMUNIMAC") == 0)
			ret = 1;
	}

	close(fd);

	return(ret);
}

void *brcm_pm_init(void)
{
	struct brcm_pm_priv *ctx;

	ctx = (void *)malloc(sizeof(*ctx));
	if(! ctx)
		goto bad;

	/* this is the current PLL frequency going into the CPU */
	if(sysfs_get(SYS_CPU_KHZ,
		(unsigned int *)&ctx->last_state.cpu_base) != 0)
	{
		/* cpufreq not supported on this platform */
		ctx->last_state.cpu_base = BRCM_PM_UNDEF;
	}
	
	if(brcm_pm_get_status(ctx, &ctx->last_state) != 0)
		goto bad_free;
	
	ctx->has_eth1 = brcm_pm_eth1_check();
	
	return(ctx);

bad_free:
	free(ctx);
bad:
	return(NULL);
}

void brcm_pm_close(void *vctx)
{
	free(vctx);
}

int brcm_pm_get_cfg(void *vctx, struct brcm_pm_cfg *cfg)
{
	struct brcm_pm_priv *ctx = vctx;

	*cfg = ctx->cfg;
	return(0);
}

int brcm_pm_set_cfg(void *vctx, struct brcm_pm_cfg *cfg)
{
	struct brcm_pm_priv *ctx = vctx;

	ctx->cfg = *cfg;
	return(0);
}

/* USB autosuspend
 * We need to scan the entire USB sysfs tree and enable/disable
 * autosuspend for all devices that support it
 */

#define POWER_RT_STATUS		"/power/runtime_status"
#define POWER_AUTOSUSPEND	"/power/autosuspend"
#define POWER_CONTROL		"/power/control"
#define POWER_LEVEL		"/power/level"
#define STATUS_SUSPENDED	"suspended"
#define STATUS_UNSUPPORTED	"unsupported"
#define CONTROL_AUTO		"auto"
#define CONTROL_ON		"on"

#define USB_RT_STATUS_GLOB	"/sys/devices/platform/*/usb*"
#define USB_RT_MAX_PATH		256

static char* chomp(char* string)
{
	char *pc;
	if (!string) return NULL;
	if ((pc = strpbrk(string, "\r\n")) != NULL)
		*pc ='\0';
	return string;
}

static int check_if_directory(char* path)
{
	struct stat stat;
	lstat(path, &stat);
	/* uclibc glob() did not accept GLOB_ONLYDIR, so filter out 
	 * non-directories and symlinks.
	 * This will guarantee we will eventually break recursion
	 */
	if (!S_ISDIR(stat.st_mode) || S_ISLNK(stat.st_mode))
		return 0;

	return 1;
}
static int brcm_pm_scan_tree(char* path, int (*func)(char*, int), int status)
{
	char newpath[USB_RT_MAX_PATH];
	glob_t g;
	int i, retval = 0;
	/* Going one level lower */
	strcpy(newpath, path);
	strcat(newpath, "/*");
	if (glob(newpath, GLOB_NOSORT, NULL, &g) != 0)
		return 0;
	for (i = 0; i < (int)g.gl_pathc; i++) {
		if (!check_if_directory(g.gl_pathv[i]))
			continue;
		func(g.gl_pathv[i], status);
		retval |= brcm_pm_scan_tree(g.gl_pathv[i], func, status);
	}
	globfree(&g);

	return retval;
	
}

static int brcm_pm_get_one_status(char* path, int status)
{
	char level_file_path[USB_RT_MAX_PATH];
	char level_string[32];

	if (!check_if_directory(path))
		return 0;

	/* This is strictly 2.6.31 case */
	strcpy(level_file_path, path);
	strcat(level_file_path, POWER_LEVEL);
	if (sysfs_get_string(level_file_path, level_string, sizeof level_string))
		return 0;

	chomp(level_string);
	if (!strncmp(level_string, CONTROL_AUTO, strlen(level_string)))
		return 0;

	return 1;
}

static int brcm_pm_usb_get_status(void)
{
	glob_t g;
	int i, status = 0;


	if (glob(USB_RT_STATUS_GLOB POWER_RT_STATUS, GLOB_NOSORT, NULL, &g) != 0) {
		/* No 'runtime_status' files found - must be 2.6.31
		 * In this case (1) traverse the whole tree,
		 * (2) check if any level file is 'on'. If none are found assume 
		 * USB is suspended. This may be not true, but it is the best we can do.
		 */
		if (glob(USB_RT_STATUS_GLOB, GLOB_NOSORT, NULL, &g) != 0)
			return BRCM_PM_UNDEF;

		for (i = 0; i < (int)g.gl_pathc && !status; i++)
			status |= brcm_pm_get_one_status(g.gl_pathv[i], status);

		/* Now traverse all the subtrees */
		if (!status)
			status |= brcm_pm_scan_tree(USB_RT_STATUS_GLOB, brcm_pm_get_one_status, status);

		globfree(&g);
	}
	else {
		/* Do not need to traverse the tree.
		 * Read status of all controllers - if any one is
		 * active, USB is active
		 */
		for (i = 0; i < (int)g.gl_pathc; i++) {
			char status_string[32];
			if (sysfs_get_string(g.gl_pathv[i], status_string, sizeof status_string))
				return BRCM_PM_UNDEF;
			chomp(status_string);
			if (strncmp(status_string, STATUS_SUSPENDED, strlen(status_string))) {
				status = 1;
				break;
			}
		}
	}
	globfree(&g);

	return status;
}

static int brcm_pm_set_one_status(char* path, int status)
{
	char status_file_path[USB_RT_MAX_PATH];
	char status_string[32];

	/* Verify runtime_status */
	strcpy(status_file_path, path);
	strcat(status_file_path, POWER_RT_STATUS);
	if (!sysfs_get_string(status_file_path, status_string, sizeof status_string)) {
		chomp(status_string);
		if (!strncmp(status_string, STATUS_UNSUPPORTED, strlen(status_string)))
			return -1;
	}

	/* Okay, autosuspend is supported by the device */
	if (status) {
		/* turn device on, autosuspend off */
		strcpy(status_file_path, path);
		strcat(status_file_path, POWER_CONTROL);
		if (sysfs_set_string(status_file_path, CONTROL_ON)) {
			/* for 2.6.31 which does not have "control" entry */
			strcpy(status_file_path, path);
			strcat(status_file_path, POWER_LEVEL);
			sysfs_set_string(status_file_path, CONTROL_ON);
		}
	}
	else {
		/* autosuspend on, default timeout 2 seconds */
		strcpy(status_file_path, path);
		strcat(status_file_path, POWER_AUTOSUSPEND);
		sysfs_set(status_file_path, 2);
		strcpy(status_file_path, path);
		strcat(status_file_path, POWER_CONTROL);
		if (sysfs_set_string(status_file_path, CONTROL_AUTO)) {
			/* for 2.6.31 which does not have "control" entry */
			strcpy(status_file_path, path);
			strcat(status_file_path, POWER_LEVEL);
			sysfs_set_string(status_file_path, CONTROL_AUTO);
		}
	}
	return 0;
}

static int brcm_pm_usb_set_status(int status)
{
	glob_t g;
	int i;

	/* Set status recursively for all the controllers
	 * and all the devices attached.
	 * NOTE: this does not guarantee that USB is suspended,
	 * because interface drivers must be closed as well.
	 * How it is done depends on class of the interface - for 
	 * usbnet it is ifdown, for usb-storage - unmount, etc.
	 */
	if (glob(USB_RT_STATUS_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return -1;

	for (i = 0; i < (int)g.gl_pathc; i++)
		brcm_pm_set_one_status(g.gl_pathv[i], status);

	/* Now traverse all the subtrees */
	brcm_pm_scan_tree(USB_RT_STATUS_GLOB, brcm_pm_set_one_status, status);

	globfree(&g);
	return 0;
}

int brcm_pm_get_status(void *vctx, struct brcm_pm_state *st)
{
	struct brcm_pm_priv *ctx = vctx;

	st->usb_status = brcm_pm_usb_get_status();

	/* read status from /proc */
	if(sysfs_get(SYS_SATA_STAT, (unsigned int *)&st->sata_status) != 0) {
		st->sata_status = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_DDR_STAT, (unsigned int *)&st->ddr_timeout) != 0) {
		st->ddr_timeout = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_STANDBY_FLAGS,
			(unsigned int *)&st->standby_flags) != 0) {
		st->standby_flags = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_TP1_STAT, (unsigned int *)&st->tp1_status) != 0) {
		st->tp1_status = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_MEMC1_STAT, (unsigned int *)&st->memc1_status) != 0) {
		st->memc1_status = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_CPU_KHZ, (unsigned int *)&st->cpu_base) != 0) {
		st->cpu_base = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_CPU_DIV, (unsigned int *)&st->cpu_divisor) != 0) {
		st->cpu_divisor = BRCM_PM_UNDEF;
	}
	if(sysfs_get(SYS_CPU_PLL, (unsigned int *)&st->cpu_pll) != 0) {
		st->cpu_pll = BRCM_PM_UNDEF;
	}

	if(st != &ctx->last_state)
		memcpy(&ctx->last_state, st, sizeof(*st));

	return(0);
}

static int sata_rescan_hosts(void)
{
	glob_t g;
	int i, ret = 0;

	if(glob(SATA_RESCAN_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return(-1);

	for(i = 0; i < (int)g.gl_pathc; i++)
		ret |= sysfs_set_string(g.gl_pathv[i], "0 - 0");
	globfree(&g);

	return(ret);
}

static int sata_delete_devices(void)
{
	glob_t g;
	int i, ret = 0;

	if(glob(SATA_DELETE_GLOB, GLOB_NOSORT, NULL, &g) != 0)
		return(0);

	for(i = 0; i < (int)g.gl_pathc; i++)
		ret |= sysfs_set(g.gl_pathv[i], 1);

	globfree(&g);

	return(ret);
}

static int sata_power_updown(int updown)
{
	int i;

	/* give the OS some time to remove the device */
	for(i = 0; i < 200; i++)
	{
		if(sysfs_set(SYS_SATA_STAT, updown) == 0)
			return 0;
		usleep(50000);
	}
	return -1;
}

int brcm_pm_set_status(void *vctx, struct brcm_pm_state *st)
{
	struct brcm_pm_priv *ctx = vctx;
	int ret = 0;

#define CHANGED(element) \
	((st->element != BRCM_PM_UNDEF) && \
	 (st->element != ctx->last_state.element))

	
	if(CHANGED(usb_status))
	{
		brcm_pm_usb_set_status(!!st->usb_status);
		ctx->last_state.usb_status = st->usb_status;
	}

	if(CHANGED(sata_status))
	{
		if(st->sata_status)
		{
			ret |= sata_power_updown(1);
			ret |= sata_rescan_hosts();
		} else {
			ret |= sata_delete_devices();
			ret |= sata_power_updown(0);
		}
		ctx->last_state.sata_status = st->sata_status;
	}

	if(CHANGED(tp1_status))
	{
		ret |= sysfs_set(SYS_TP1_STAT, st->tp1_status);
	}

	if(CHANGED(cpu_divisor))
	{
		ret |= sysfs_set(SYS_CPU_DIV, st->cpu_divisor);
	}

	if(CHANGED(cpu_pll))
	{
		ret |= sysfs_set(SYS_CPU_PLL, st->cpu_pll);
	}

	if(CHANGED(ddr_timeout))
	{
		ret |= sysfs_set(SYS_DDR_STAT, st->ddr_timeout);
	}

	if(CHANGED(memc1_status))
	{
		ret |= sysfs_set(SYS_MEMC1_STAT, st->memc1_status);
	}

	if(CHANGED(standby_flags))
	{
		ret |= sysfs_set(SYS_STANDBY_FLAGS, st->standby_flags);
	}

#undef CHANGED

	return(ret);
}

int brcm_pm_suspend(void *vctx, int suspend_mode)
{
	if(suspend_mode == BRCM_PM_STANDBY)
		return sysfs_set_string(SYS_STANDBY, "standby");
	if(suspend_mode == BRCM_PM_SUSPEND)
		return sysfs_set_string(SYS_STANDBY, "mem");
	if(suspend_mode == BRCM_PM_HIBERNATE)
		return sysfs_set_string(SYS_STANDBY, "disk");
	if(suspend_mode == BRCM_PM_IRW_HALT)
		return run(HALT_PATH, NULL);
	return -1;
}
