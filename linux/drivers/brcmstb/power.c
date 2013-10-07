/*
 * Copyright (C) 2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <stdarg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/smp.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/mii.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/compiler.h>
#include <linux/brcmstb/brcmstb.h>

/* Wakeup on timeout (20 sec) even if standby_flags does not request this */
#define WATCHDOG_TIMER_WAKEUP_ALWAYS		(0)

#define PRINT_PM_CALLBACK		printk(KERN_DEBUG "%s %02x\n", \
	__func__, (u8)flags)
#if 0
#define DBG			printk
#else
#define DBG(...)		do { } while (0)
#endif

/***********************************************************************
 * USB / ENET / GENET / MoCA / SATA PM common internal functions
 ***********************************************************************/
struct clk {
	char		name[16];
	int             refcnt;
	struct clk     *parent;
	int		(*cb)(int, void *);
	void		*cb_arg;
	void		(*disable)(u32 flags);
	void		(*enable)(u32 flags);
	int		(*set_rate)(unsigned long rate);
	u32		flags;
	struct list_head list;
};
/*
 * Flags:
 * BRCM_PM_FLAG_S3		- S3 standby inititated. Since all blocks lose
 *	power, there is no need to preserve state.
 *	On-chip SRAM can be unconditionally powered down.
 * BRCM_PM_FLAG_ENET_WOL	- Ethernet WOL is a wake-up event
 * BRCM_PM_FLAG_MOCA_WOL	- MoCA WOL is a wake-up event
 * BRCM_PM_FLAG_USB_WAKEUP	- USB insertion/removal is a wake-up event
 */
#define BRCM_PM_FLAG_S3		0x01 /* set when suspend begins */
#define BRCM_PM_FLAG_ENET_WOL	0x02 /* ENET WOL is enabled */
#define BRCM_PM_FLAG_MOCA_WOL	0x04 /* MoCA WOL is enabled */
#define BRCM_PM_FLAG_USB_WAKEUP	0x08 /* USB insertion/removal wakeup */

#define ANY_WOL(flags) (flags & (BRCM_PM_FLAG_ENET_WOL|BRCM_PM_FLAG_MOCA_WOL))
#define ENET_WOL(flags) (flags & BRCM_PM_FLAG_ENET_WOL)
#define MOCA_WOL(flags) (flags & BRCM_PM_FLAG_MOCA_WOL)

static DEFINE_SPINLOCK(brcm_pm_clk_lock);

static void brcm_pm_sata_disable(u32 flags);
static void brcm_pm_sata_enable(u32 flags);
static void brcm_pm_genet_disable(u32 flags);
static void brcm_pm_genet_enable(u32 flags);
static void brcm_pm_genet_disable_wol(u32 flags);
static void brcm_pm_genet_enable_wol(u32 flags);
static void brcm_pm_moca_disable(u32 flags);
static void brcm_pm_moca_enable(u32 flags);
static void brcm_pm_moca_disable_wol(u32 flags);
static void brcm_pm_moca_enable_wol(u32 flags);
static void brcm_pm_genet1_disable(u32 flags);
static void brcm_pm_genet1_enable(u32 flags);
static void brcm_pm_network_disable(u32 flags);
static void brcm_pm_network_enable(u32 flags);
static void brcm_pm_usb_disable(u32 flags);
static void brcm_pm_usb_enable(u32 flags);
static void brcm_pm_set_ddr_timeout(int);
static void brcm_pm_initialize(void);
static int  brcm_pm_moca_cpu_set_rate(unsigned long rate);
static int  brcm_pm_moca_phy_set_rate(unsigned long rate);

static int brcm_pm_ddr_timeout;
static unsigned long brcm_pm_standby_flags;
static unsigned long brcm_pm_standby_timeout;

enum {
	BRCM_CLK_SATA,
	BRCM_CLK_GENET,
	BRCM_CLK_MOCA,
	BRCM_CLK_USB,
	BRCM_CLK_GENET1,
	BRCM_CLK_NETWORK,	/* PLLs/clocks common to all GENETs */
	BRCM_CLK_GENET_WOL,
	BRCM_CLK_MOCA_WOL,
	BRCM_CLK_MOCA_PHY,
	BRCM_CLK_MOCA_CPU,
};

static struct clk brcm_clk_table[] = {
	[BRCM_CLK_SATA] = {
		.name		= "sata",
		.disable	= &brcm_pm_sata_disable,
		.enable		= &brcm_pm_sata_enable,
	},
	[BRCM_CLK_GENET] = {
		.name		= "enet",
		.disable	= &brcm_pm_genet_disable,
		.enable		= &brcm_pm_genet_enable,
		.parent		= &brcm_clk_table[BRCM_CLK_NETWORK],
	},
	[BRCM_CLK_MOCA] = {
		.name		= "moca",
		.disable	= &brcm_pm_moca_disable,
		.enable		= &brcm_pm_moca_enable,
		.parent		= &brcm_clk_table[BRCM_CLK_GENET1],
	},
	[BRCM_CLK_USB] = {
		.name		= "usb",
		.disable	= &brcm_pm_usb_disable,
		.enable		= &brcm_pm_usb_enable,
	},
	[BRCM_CLK_GENET1] = {
		.name		= "moca_genet",
		.disable	= &brcm_pm_genet1_disable,
		.enable		= &brcm_pm_genet1_enable,
		.parent		= &brcm_clk_table[BRCM_CLK_NETWORK],
	},
	[BRCM_CLK_NETWORK] = {
		.name		= "network",
		.disable	= &brcm_pm_network_disable,
		.enable		= &brcm_pm_network_enable,
	},
	[BRCM_CLK_GENET_WOL] = {
		.name		= "enet-wol",
		.disable	= &brcm_pm_genet_disable_wol,
		.enable		= &brcm_pm_genet_enable_wol,
	},
	[BRCM_CLK_MOCA_WOL] = {
		.name		= "moca-wol",
		.disable	= &brcm_pm_moca_disable_wol,
		.enable		= &brcm_pm_moca_enable_wol,
	},
	[BRCM_CLK_MOCA_CPU] = {
		.name		= "moca-cpu",
		.set_rate	= &brcm_pm_moca_cpu_set_rate,
	},
	[BRCM_CLK_MOCA_PHY] = {
		.name		= "moca-phy",
		.set_rate	= &brcm_pm_moca_phy_set_rate,
	},
};

/* These clocks are chip specific - each .initialize()
  method will add them if needed */
static LIST_HEAD(brcm_dyn_clk_list);

static __maybe_unused void __clk_dyn_add(struct clk *clk)
{
	struct clk *clk_p;
	list_for_each_entry(clk_p, &brcm_dyn_clk_list, list) {
		if (clk_p == clk)
			return;
	}
	list_add(&clk->list, &brcm_dyn_clk_list);
}

static __maybe_unused void __clk_dyn_del(struct clk *clk)
{
	list_del(&clk->list);
}

static struct clk *brcm_pm_clk_find(const char *name)
{
	int i;
	struct clk *clk = brcm_clk_table;

	BUG_ON(!name);
	/* first check static clocks */
	for (i = 0; i < ARRAY_SIZE(brcm_clk_table); i++, clk++)
		if (!strncmp(name, clk->name, strlen(name)))
			return clk;

	/* check dynamic clocks */
	list_for_each_entry(clk, &brcm_dyn_clk_list, list) {
		if (!strncmp(name, clk->name, strlen(name)))
			return clk;
	}
	return NULL;
}

/* sysfs attributes */

ssize_t brcm_pm_show_sata_power(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct clk *clk = brcm_pm_clk_find("sata");
	return snprintf(buf, PAGE_SIZE, "%d\n", !!clk->refcnt);
}

ssize_t brcm_pm_store_sata_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct clk *clk = brcm_pm_clk_find("sata");
	int val;

	if (!clk || !clk->cb || sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	return clk->cb(val ? PM_EVENT_RESUME : PM_EVENT_SUSPEND,
		clk->cb_arg) ? : count;
}

ssize_t brcm_pm_show_ddr_timeout(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", brcm_pm_ddr_timeout);
}

ssize_t brcm_pm_store_ddr_timeout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	brcm_pm_ddr_timeout = val;
	if (brcm_pm_enabled)
		brcm_pm_set_ddr_timeout(val);
	return count;
}

ssize_t brcm_pm_show_standby_flags(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%lx\n", brcm_pm_standby_flags);
}

ssize_t brcm_pm_store_standby_flags(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	brcm_pm_standby_flags = val;
	return count;
}

ssize_t brcm_pm_show_standby_timeout(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", brcm_pm_standby_timeout);
}

ssize_t brcm_pm_store_standby_timeout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	if (sscanf(buf, "%lu", &val) != 1)
		return -EINVAL;

	brcm_pm_standby_timeout = val;
	return count;
}
#if defined(CONFIG_BRCM_HAS_1GB_MEMC1) || defined(CONFIG_BCM7420)
/*
 * brcm_pm_memc1_power
 * Power state of secondary memory controller
 * 0 - complete power down (with loss of content)
 * 1 - full power mode
 * 2 - SSPD (content preserved)
 * Direct transition between 0 and 2 is not supported
 */
#define BRCM_PM_MEMC1_OFF	0
#define BRCM_PM_MEMC1_ON	1
#define BRCM_PM_MEMC1_SSPD	2

static int brcm_pm_memc1_power = BRCM_PM_MEMC1_ON;
int __weak brcm_pm_memc1_suspend(void) { return 0; }
int __weak brcm_pm_memc1_resume(void) { return 0; }
int __weak brcm_pm_memc1_powerdown(void) { return 0; }
int  __weak brcm_pm_memc1_powerup(void) { return 0; }

ssize_t brcm_pm_show_memc1_power(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", brcm_pm_memc1_power);
}

ssize_t brcm_pm_store_memc1_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	/*
	 * if memc1 memory has been added to system memory pool,
	 * disable memc1 dynamic PM
	 */
	if (brcm_dram1_linux_mb && brcm_dram1_linux_mb <= brcm_dram1_size_mb) {
		sysfs_chmod_file(&dev->kobj,
			&attr->attr, attr->attr.mode & ~S_IWUGO);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	/* check for no change */
	if (val == brcm_pm_memc1_power)
		return count;

	switch (val) {
	case BRCM_PM_MEMC1_OFF:
		if (brcm_pm_memc1_power == BRCM_PM_MEMC1_ON)
			brcm_pm_memc1_powerdown();
		else
			return -EINVAL;
		break;
	case BRCM_PM_MEMC1_ON:
		if (brcm_pm_memc1_power == BRCM_PM_MEMC1_OFF) {
			if (brcm_pm_memc1_powerup())
				return -EINVAL;
		} else if (brcm_pm_memc1_power == BRCM_PM_MEMC1_SSPD)
			brcm_pm_memc1_resume();
		else
			return -EINVAL;
		break;
	case BRCM_PM_MEMC1_SSPD:
		if (brcm_pm_memc1_power == BRCM_PM_MEMC1_ON)
			brcm_pm_memc1_suspend();
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	brcm_pm_memc1_power = val;
	return count;
}

#endif

static u32 brcm_pm_time_at_wakeup[2];
ssize_t brcm_pm_show_time_at_wakeup(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x 0x%x\n",
		brcm_pm_time_at_wakeup[0], brcm_pm_time_at_wakeup[1]);
}

static int brcm_pm_halt_mode;

ssize_t brcm_pm_show_halt_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", brcm_pm_halt_mode);
}

ssize_t brcm_pm_store_halt_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	brcm_pm_halt_mode = !!val;
	return count;
}

/* Boot time functions */

static int __init brcm_pm_init(void)
{
	if (!brcm_pm_enabled)
		return 0;
	if (!brcm_moca_enabled)
		brcm_pm_moca_disable(0);
	/* chip specific initialization */
	brcm_pm_initialize();
	return 0;
}

early_initcall(brcm_pm_init);

static int nopm_setup(char *str)
{
	brcm_pm_enabled = 0;
	return 0;
}

__setup("nopm", nopm_setup);

int brcm_pm_hash_enabled = 1;

static int nohash_setup(char *str)
{
	brcm_pm_hash_enabled = 0;
	return 0;
}

__setup("nohash", nohash_setup);

/***********************************************************************
 * USB / ENET / GENET / MoCA / SATA PM external API
 ***********************************************************************/

/* internal functions assume the lock is held */
static int __clk_enable(struct clk *clk, u32 flags)
{
	BUG_ON(clk->refcnt < 0);
	if (++(clk->refcnt) == 1 && brcm_pm_enabled) {
		if (clk->parent)
			__clk_enable(clk->parent, clk->flags | flags);
		printk(KERN_DEBUG "%s: %s [%d]\n",
			__func__, clk->name, clk->refcnt);
		if (clk->enable)
			clk->enable(clk->flags | flags);
	}
	return 0;
}

static void __clk_disable(struct clk *clk, u32 flags)
{
	if (--(clk->refcnt) == 0 && brcm_pm_enabled) {
		printk(KERN_DEBUG "%s: %s [%d]\n",
			__func__, clk->name, clk->refcnt);
		if (clk->disable)
			clk->disable(clk->flags | flags);
		if (clk->parent)
			__clk_disable(clk->parent, clk->flags | flags);
	}
	BUG_ON(clk->refcnt < 0);
}

int clk_enable(struct clk *clk)
{
	unsigned long flags;

	if (clk && !IS_ERR(clk)) {
		spin_lock_irqsave(&brcm_pm_clk_lock, flags);
		__clk_enable(clk, 0);
		spin_unlock_irqrestore(&brcm_pm_clk_lock, flags);
		return 0;
	} else {
		return -EINVAL;
	}
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (clk && !IS_ERR(clk)) {
		spin_lock_irqsave(&brcm_pm_clk_lock, flags);
		__clk_disable(clk, 0);
		spin_unlock_irqrestore(&brcm_pm_clk_lock, flags);
	}
}
EXPORT_SYMBOL(clk_disable);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long flags;
	spinlock_t *lock = &brcm_pm_clk_lock;
	if (clk && !IS_ERR(clk)) {
		spin_lock_irqsave(lock, flags);
		clk->parent = parent;
		spin_unlock_irqrestore(lock, flags);
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	if (clk && !IS_ERR(clk))
		return clk->parent;

	return NULL;
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	int ret;
	spinlock_t *lock = &brcm_pm_clk_lock;

	if (clk && !IS_ERR(clk) && clk->set_rate) {
		spin_lock_irqsave(lock, flags);
		ret = clk->set_rate(rate);
		spin_unlock_irqrestore(lock, flags);
		return ret;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

int brcm_pm_register_cb(char *name, int (*fn)(int, void *), void *arg)
{
	struct clk *clk = brcm_pm_clk_find(name);
	unsigned long flags;

	if (!clk)
		return -ENOENT;

	spin_lock_irqsave(&brcm_pm_clk_lock, flags);
	BUG_ON(fn && clk->cb);
	clk->cb = fn;
	clk->cb_arg = arg;
	spin_unlock_irqrestore(&brcm_pm_clk_lock, flags);

	return 0;
}
EXPORT_SYMBOL(brcm_pm_register_cb);

int brcm_pm_unregister_cb(char *name)
{
	return brcm_pm_register_cb(name, NULL, NULL);
}
EXPORT_SYMBOL(brcm_pm_unregister_cb);

#ifdef CONFIG_BRCM_HAS_STANDBY
/***********************************************************************
 * Wakeup source management
 *   All kernel drivers can use this api to register their wakeup
 *   sources. This will enable 'lightweight' resume feature, when
 *   wakeup event is quickly qualified immediately after resume using
 *   driver-supplied poll() method. If none of registered poll() method
 *   recognize the wakeup event, system may be brought back to sleep.
 *   To allow for other components to use their wakeup capabilities, system
 *   suspend code will save the pre-suspend PM_L2 mask and, if all poll()
 *   callbacks deny the wakeup event, check if one of events enabled in the
 *   saved mask has occurred.
 ***********************************************************************/
struct brcm_wakeup_source {
	struct brcm_wakeup_ops		*ops;
	void				*ref;
	struct list_head		list;
	struct kref			kref;
	char				name[16];
};

struct brcm_wakeup_control {
	spinlock_t			lock;
	struct list_head		head;
	int				count;
};

static struct brcm_wakeup_control bwc;

int brcm_pm_wakeup_register(struct brcm_wakeup_ops *ops, void *ref, char *name)
{
	struct brcm_wakeup_source *bws;
	unsigned long flags;

	list_for_each_entry(bws, &bwc.head, list) {
		if (bws->ops == ops && bws->ref == ref) {
			/* already registered */
			spin_lock_irqsave(&bwc.lock, flags);
			kref_get(&bws->kref);
			spin_unlock_irqrestore(&bwc.lock, flags);
			return 0;
		}
	}

	bws = kmalloc(sizeof(struct brcm_wakeup_source), GFP_ATOMIC);
	if (!bws)
		return -1;

	bws->ops = ops;
	bws->ref = ref;
	if (name)
		strncpy(bws->name, name, 16);

	kref_init(&bws->kref);
	kref_get(&bws->kref);

	spin_lock_irqsave(&bwc.lock, flags);
	list_add_tail(&bws->list, &bwc.head);
	spin_unlock_irqrestore(&bwc.lock, flags);

	return 0;
}
EXPORT_SYMBOL(brcm_pm_wakeup_register);

/* This function is called with bwc lock held*/
static void brcm_pm_wakeup_cleanup(struct kref *kref)
{
	struct brcm_wakeup_source *bws =
		container_of(kref, struct brcm_wakeup_source, kref);
	list_del(&bws->list);
	kfree(bws);
}

int brcm_pm_wakeup_unregister(struct brcm_wakeup_ops *ops, void *ref)
{
	struct brcm_wakeup_source *bws;
	unsigned long flags;

	spin_lock_irqsave(&bwc.lock, flags);

	list_for_each_entry(bws, &bwc.head, list)
		if (bws->ops == ops && bws->ref == ref)
			kref_put(&bws->kref, brcm_pm_wakeup_cleanup);

	spin_unlock_irqrestore(&bwc.lock, flags);

	return 0;
}

static int brcm_pm_wakeup_enable(void)
{
	struct brcm_wakeup_source *bws;
	unsigned long flags;

	spin_lock_irqsave(&bwc.lock, flags);

	list_for_each_entry(bws, &bwc.head, list) {
		if (bws->ops && bws->ops->enable)
			bws->ops->enable(bws->ref);
	}
	spin_unlock_irqrestore(&bwc.lock, flags);
	return 0;
}

static int brcm_pm_wakeup_disable(void)
{
	struct brcm_wakeup_source *bws;
	unsigned long flags;

	spin_lock_irqsave(&bwc.lock, flags);

	list_for_each_entry(bws, &bwc.head, list) {
		if (bws->ops && bws->ops->disable)
			bws->ops->disable(bws->ref);
	}
	spin_unlock_irqrestore(&bwc.lock, flags);
	return 0;
}

/*
Function asks all registered objects if a wakeup event has happened.
If no registered object claims the event, it checks for all interrupts
enabled prior to suspend.
It returns 0 if no valid wake up event has occurred, 1 otherwise.
*/
static int brcm_pm_wakeup_poll(u32 mask)
{
	int ev_occurred = 0;
	struct brcm_wakeup_source *bws;
	unsigned long flags;

	spin_lock_irqsave(&bwc.lock, flags);

	list_for_each_entry(bws, &bwc.head, list) {
		if (bws->ops && bws->ops->poll)
			ev_occurred |= bws->ops->poll(bws->ref);
	}
	spin_unlock_irqrestore(&bwc.lock, flags);

	/* Check for events outside of kernel control */
	if (!ev_occurred)
		ev_occurred |= brcm_pm_wakeup_get_status(~mask);

	return ev_occurred;
}

int brcm_pm_wakeup_init(void)
{
	spin_lock_init(&bwc.lock);
	INIT_LIST_HEAD(&bwc.head);
	return 0;
}
early_initcall(brcm_pm_wakeup_init);

#endif

/***********************************************************************
 * USB / ENET / GENET / MoCA / SATA PM implementations (per-chip)
 ***********************************************************************/
static __maybe_unused void brcm_ddr_phy_initialize(void);

/*
 * Per-block power management operations pair.
 * Parameter flags is used to control wake-up capabilities
 */

struct brcm_chip_pm_block_ops {
	void (*enable)(u32 flags);
	void (*disable)(u32 flags);
	int (*set_cpu_rate)(unsigned long rate);
	int (*set_phy_rate)(unsigned long rate);
};

static u32 brcm_pm_flags;

struct brcm_chip_pm_ops {
	/* SATA power/clock gating */
	struct brcm_chip_pm_block_ops sata;
	/* genet0 power/clock gating */
	struct brcm_chip_pm_block_ops genet;
	/* genet1 power/clock gating */
	struct brcm_chip_pm_block_ops genet1;
	/* MoCA power/clock gating */
	struct brcm_chip_pm_block_ops moca;
	/* USB power/clock gating */
	struct brcm_chip_pm_block_ops usb;
	/* genet0/genet1/MoCA common power/clock gating */
	struct brcm_chip_pm_block_ops network;
	/* system-wide initialization code */
	void (*initialize)(void);
	/* system suspend code */
	void (*suspend)(u32 flags);
	/* system resume code */
	void (*resume)(u32 flags);
	/*
	 * System suspend code executed immediately before
	 * initiating low-level suspend sequence.
	 * For example, shutting down secondary memory controller
	 * can be done here when system no longer uses highmem regions
	 */
	void (*late_suspend)(void);
	void (*early_resume)(void);
	/* for chip specific clock mappings ( see #SWLINUX-1764 ) */
	struct clk* (*clk_get)(struct device *dev, const char *id);
};

#define DEF_BLOCK_PM_OP(block, chip) \
	.block.enable	= bcm##chip##_pm_##block##_enable, \
	.block.disable	= bcm##chip##_pm_##block##_disable
#define DEF_SYSTEM_PM_OP(chip) \
	.suspend	= bcm##chip##_pm_suspend, \
	.resume		= bcm##chip##_pm_resume
#define DEF_SYSTEM_LATE_PM_OP(chip) \
	.late_suspend	= bcm##chip##_pm_late_suspend, \
	.early_resume	= bcm##chip##_pm_early_resume

#define PLL_CH_DIS(x)		BDEV_WR_RB(BCHP_##x, 0x04)
#define PLL_CH_ENA(x)		do { \
					BDEV_WR_RB(BCHP_##x, 0x03); \
					mdelay(1); \
				} while (0)

#define PLL_DIS(x)		BDEV_WR_RB(BCHP_##x, 0x04)
#define PLL_ENA(x)		do { \
					BDEV_WR_RB(BCHP_##x, 0x03); \
					mdelay(1); \
				} while (0)

static __maybe_unused struct clk *brcm_pm_clk_get(struct device *dev,
	const char *id)
{
	if (!strncmp(id, "enet", 4) && dev) {
		struct platform_device *pdev = to_platform_device(dev);
		if (pdev->id == 0) /* enet */
			return brcm_pm_clk_find(id);
		if (pdev->id == 1) /* moca_genet */ {
			if (strstr(id, "-wol"))
				return brcm_pm_clk_find("moca-wol");
			return brcm_pm_clk_find("moca");
		}
	}
	return NULL;
}

/***********************************************************************
 * Encryption setup for S3 suspend
 ***********************************************************************/
static struct brcm_dram_encoder_ops *dram_encoder_ops;

void brcm_pm_set_dram_encoder(struct brcm_dram_encoder_ops *ops)
{
	dram_encoder_ops = ops;
}
EXPORT_SYMBOL(brcm_pm_set_dram_encoder);

#ifdef CONFIG_BRCM_HAS_AON

int brcm_pm_dram_encoder_prepare(struct brcm_mem_transfer *param)
{
	if (dram_encoder_ops && dram_encoder_ops->prepare)
		return dram_encoder_ops->prepare(param);
	return -1;
}
EXPORT_SYMBOL(brcm_pm_dram_encoder_prepare);

int brcm_pm_dram_encoder_complete(struct brcm_mem_transfer *param)
{
	if (dram_encoder_ops && dram_encoder_ops->complete)
		return dram_encoder_ops->complete(param);
	return -1;
}
EXPORT_SYMBOL(brcm_pm_dram_encoder_complete);

void brcm_pm_dram_encoder_start(void)
{
	if (dram_encoder_ops && dram_encoder_ops->start)
		dram_encoder_ops->start();
}
EXPORT_SYMBOL(brcm_pm_dram_encoder_start);
#endif /* CONFIG_BRCM_HAS_AON */

#if defined(CONFIG_BCM7125)
static void bcm7125_pm_sata_disable(u32 flags)
{
	BDEV_WR_F_RB(SUN_TOP_CTRL_GENERAL_CTRL_1, sata_ana_pwrdn, 1);
	BDEV_WR_F_RB(CLKGEN_SATA_CLK_PM_CTRL, DIS_CLK_99P7, 1);
	BDEV_WR_F_RB(CLKGEN_SATA_CLK_PM_CTRL, DIS_CLK_216, 1);
	BDEV_WR_F_RB(CLKGEN_SATA_CLK_PM_CTRL, DIS_CLK_108, 1);
	PLL_CH_DIS(VCXO_CTL_MISC_RAP_AVD_PLL_CHL_4);
}

static void bcm7125_pm_sata_enable(u32 flags)
{
	PLL_CH_ENA(VCXO_CTL_MISC_RAP_AVD_PLL_CHL_4);
	BDEV_WR_F_RB(CLKGEN_SATA_CLK_PM_CTRL, DIS_CLK_108, 0);
	BDEV_WR_F_RB(CLKGEN_SATA_CLK_PM_CTRL, DIS_CLK_216, 0);
	BDEV_WR_F_RB(CLKGEN_SATA_CLK_PM_CTRL, DIS_CLK_99P7, 0);
	BDEV_WR_F_RB(SUN_TOP_CTRL_GENERAL_CTRL_1, sata_ana_pwrdn, 0);
}

static void bcm7125_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,
			CLOCK_SEL_ENET_CG_MOCA, 1);
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,
			CLOCK_SEL_GMII_CG_MOCA, 1);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_HFB, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_L2_INTR, 1);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL,
			DIS_CLK_UNIMAC_SYS_TX, 1);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_216, 1);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL,
			DIS_CLK_250_GENET_MOCA, 1);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_54, 1);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_108, 1);
		return;
	}
	BDEV_SET_RB(BCHP_CLKGEN_MOCA_CLK_PM_CTRL, 0xf77);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH3_PM_CTRL, 0x04);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH4_PM_CTRL, 0x04);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH5_PM_CTRL, 0x04);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH6_PM_CTRL, 0x04);
	BDEV_SET_RB(BCHP_CLKGEN_PLL_MOCA_CTRL, 0x13);
}

static void bcm7125_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,
			CLOCK_SEL_ENET_CG_MOCA, 0);
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,
			CLOCK_SEL_GMII_CG_MOCA, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_L2_INTR, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL,
			DIS_CLK_UNIMAC_SYS_TX, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_216, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL,
			DIS_CLK_250_GENET_MOCA, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_54, 0);
		BDEV_WR_F_RB(CLKGEN_MOCA_CLK_PM_CTRL, DIS_CLK_108, 0);
		return;
	}
	BDEV_UNSET_RB(BCHP_CLKGEN_PLL_MOCA_CTRL, 0x13);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH6_PM_CTRL, 0x01);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH5_PM_CTRL, 0x01);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH4_PM_CTRL, 0x01);
	BDEV_WR_RB(BCHP_CLKGEN_PLL_MOCA_CH3_PM_CTRL, 0x01);
	BDEV_UNSET_RB(BCHP_CLKGEN_MOCA_CLK_PM_CTRL, 0xf77);
}

static void bcm7125_pm_usb_disable(u32 flags)
{
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 1);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0x00);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_216, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_108, 1);
	PLL_CH_DIS(CLKGEN_PLL_MAIN_CH4_PM_CTRL);
}

static void bcm7125_pm_usb_enable(u32 flags)
{
	PLL_CH_ENA(CLKGEN_PLL_MAIN_CH4_PM_CTRL);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_108, 0);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_216, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0x0f);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 1);
}

static void bcm7125_pm_suspend(u32 flags)
{
	/* PCI/EBI */
	BDEV_WR_RB(BCHP_HIF_TOP_CTRL_PM_CTRL, 0x3ff8);
	BDEV_WR_F_RB(CLKGEN_CLK_27_OUT_PM_CTRL, DIS_CLK_27_OUT, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_PM_DIS_CHL_1, DIS_CH, 1);
	BDEV_SET_RB(BCHP_CLKGEN_HIF_CLK_PM_CTRL, 0x3c);

	/* system PLLs */
	BDEV_SET_RB(BCHP_VCXO_CTL_MISC_VC0_CTRL, 0x0b);
	BDEV_UNSET_RB(BCHP_VCXO_CTL_MISC_VC0_CTRL, 0x04);
	BDEV_SET_RB(BCHP_VCXO_CTL_MISC_RAP_AVD_PLL_CTRL, 0x07);
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_216, 1);
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_108, 1);

	/* disable self-refresh since it interferes with suspend */
	brcm_pm_set_ddr_timeout(0);
}

static void bcm7125_pm_resume(u32 flags)
{
	/* system PLLs */
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_108, 0);
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_216, 0);
	BDEV_UNSET_RB(BCHP_VCXO_CTL_MISC_RAP_AVD_PLL_CTRL, 0x07);
	BDEV_SET_RB(BCHP_VCXO_CTL_MISC_VC0_CTRL, 0x04);
	BDEV_UNSET_RB(BCHP_VCXO_CTL_MISC_VC0_CTRL, 0x0b);

	/* PCI/EBI */
	BDEV_UNSET_RB(BCHP_CLKGEN_HIF_CLK_PM_CTRL, 0x3c);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_PM_DIS_CHL_1, DIS_CH, 0);
	BDEV_WR_F_RB(CLKGEN_CLK_27_OUT_PM_CTRL, DIS_CLK_27_OUT, 0);
	BDEV_WR_RB(BCHP_HIF_TOP_CTRL_PM_CTRL, 0x00);

	brcm_ddr_phy_initialize();
}

/* There is no non-MoCA GENET on 7125, so any request for 'enet' clock is
 * redirected to 'moca'
 */
static struct clk *bcm7125_pm_clk_get(struct device *dev, const char *id)
{
	if (!strncmp(id, "enet", 4)) {
		if (strstr(id, "-wol"))
			return brcm_pm_clk_find("moca-wol");
		else
			return brcm_pm_clk_find("moca");
	}
	return NULL;
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7125),
	DEF_BLOCK_PM_OP(sata, 7125),
	DEF_BLOCK_PM_OP(network, 7125),
	DEF_SYSTEM_PM_OP(7125),
	.clk_get		= bcm7125_pm_clk_get,
	.initialize		= brcm_ddr_phy_initialize,
};
#endif

#if defined(CONFIG_BCM7340)

static void bcm7340_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		/*
		 * TODO: Stop HFB clock and switch to 27MHz if ACPI
		 * detector is disabled
		 */
#if 0
		/* TODO: on 1000Mbit link use 54MHz clock and MAIN PLL */
		BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, MAIN_PLL, 1);
		mdelay(1);
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,
			CLOCK_SEL_CG_GENET, 1);
#endif
		/* switch to slower 27MHz clocks */
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,
			CLOCK_SEL_CG_GENET, 0);

		BDEV_SET_RB(BCHP_CLKGEN_GENET_CLK_PM_CTRL,
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_216_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK);
		return;
	}

	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH4_PM_CTRL, PWRDN_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH4_PM_CTRL, ENB_CLOCKOUT_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH4_PM_CTRL, EN_CMLBUF_CH4, 0);
	BDEV_SET_RB(BCHP_CLKGEN_GENET_CLK_PM_CTRL,
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_250_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_RX_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_HFB_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_25_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_54_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_108_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_216_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_27X_PM_MASK);
}

static void bcm7340_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_UNSET_RB(BCHP_CLKGEN_GENET_CLK_PM_CTRL,
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_HFB_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_216_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		    BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK);
		/* switch to faster 54MHz clocks */
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS,	CLOCK_SEL_CG_GENET, 1);
		return;
	}

	BDEV_UNSET_RB(BCHP_CLKGEN_GENET_CLK_PM_CTRL,
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_250_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_RX_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_HFB_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_25_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_54_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_108_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_216_MASK|
		BCHP_CLKGEN_GENET_CLK_PM_CTRL_DIS_CLK_27X_PM_MASK);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH4_PM_CTRL, EN_CMLBUF_CH4, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH4_PM_CTRL, ENB_CLOCKOUT_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH4_PM_CTRL, PWRDN_CH, 0);
}

static void bcm7340_pm_moca_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
#if 0
		/* TODO: on 1000Mbit link use 54MHz clock and MAIN PLL */
		BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, MAIN_PLL, 1);
		mdelay(1);
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS, CLOCK_SEL_ENET_CG_MOCA,
			1);
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS, CLOCK_SEL_GMII_CG_MOCA,
			0);
#endif
		/* switch to slower 27MHz clocks */
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS, CLOCK_SEL_ENET_CG_MOCA,
			0);
		BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS, CLOCK_SEL_GMII_CG_MOCA,
			1);
	    BDEV_SET_RB(BCHP_CLKGEN_MOCA_CLK_PM_CTRL,
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_250_GENET_RGMII_MOCA_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_GENET_RGMII_CG_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_54_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_108_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_MASK);
	    return;
	}

	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH1_PM_CTRL, PWRDN_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH1_PM_CTRL, ENB_CLOCKOUT_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH1_PM_CTRL, EN_CMLBUF_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH2_PM_CTRL, PWRDN_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH2_PM_CTRL, ENB_CLOCKOUT_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH2_PM_CTRL, EN_CMLBUF_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH3_PM_CTRL, PWRDN_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH3_PM_CTRL, ENB_CLOCKOUT_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH3_PM_CTRL, EN_CMLBUF_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH6_PM_CTRL, PWRDN_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH6_PM_CTRL, ENB_CLOCKOUT_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH6_PM_CTRL, EN_CMLBUF_CH6, 0);
	BDEV_SET_RB(BCHP_CLKGEN_MOCA_CLK_PM_CTRL,
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_250_GENET_RGMII_MOCA_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_GENET_RGMII_CG_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_RX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_HFB_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_27X_PM_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_54_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_108_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_MASK);
}

static void bcm7340_pm_moca_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		BDEV_UNSET_RB(BCHP_CLKGEN_MOCA_CLK_PM_CTRL,
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_250_GENET_RGMII_MOCA_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_GENET_RGMII_CG_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_RX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_HFB_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_27X_PM_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_54_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_108_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_MASK);

	    /* Restore fast clocks */
	    BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS, CLOCK_SEL_ENET_CG_MOCA, 1);
	    BDEV_WR_F_RB(CLKGEN_MISC_CLOCK_SELECTS, CLOCK_SEL_GMII_CG_MOCA, 0);

	    return;
	}

	BDEV_UNSET_RB(BCHP_CLKGEN_MOCA_CLK_PM_CTRL,
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_250_GENET_RGMII_MOCA_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_GENET_RGMII_CG_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_RX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_UNIMAC_SYS_TX_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_L2_INTR_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_HFB_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_GMII_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_27X_PM_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_54_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_108_MASK|
		BCHP_CLKGEN_MOCA_CLK_PM_CTRL_DIS_CLK_216_MASK);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH6_PM_CTRL, EN_CMLBUF_CH6, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH6_PM_CTRL, ENB_CLOCKOUT_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH6_PM_CTRL, PWRDN_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH3_PM_CTRL, EN_CMLBUF_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH3_PM_CTRL, ENB_CLOCKOUT_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH3_PM_CTRL, PWRDN_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH2_PM_CTRL, EN_CMLBUF_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH2_PM_CTRL, ENB_CLOCKOUT_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH2_PM_CTRL, PWRDN_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH1_PM_CTRL, EN_CMLBUF_CH, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH1_PM_CTRL, ENB_CLOCKOUT_CH, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CH1_PM_CTRL, PWRDN_CH, 0);
}

static void bcm7340_pm_usb_disable(u32 flags)
{
	/* reset and power down all 4 ports */
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0x0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 1);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_SOFT_RESETB, 0x0);

	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 0);
	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_216, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_108, 1);
	/* disable PLL */
	BDEV_WR_F_RB(CLKGEN_PLLMAIN_CH4_PM_CTRL, PWRDN_CH4_PLLMAIN, 1);
}

static void bcm7340_pm_usb_enable(u32 flags)
{
	BDEV_WR_F_RB(CLKGEN_PLLMAIN_CH4_PM_CTRL, PWRDN_CH4_PLLMAIN, 0);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_108, 0);
	BDEV_WR_F_RB(CLKGEN_USB_CLK_PM_CTRL, DIS_CLK_216, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 1);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 1);

	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_SOFT_RESETB, 0xF);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0xF);
}

static void bcm7340_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, DRESET, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, ARESET, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, PWRDN, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, PWRDN_LDO, 1);
}

static void bcm7340_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, PWRDN_LDO, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, PWRDN, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, ARESET, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMOCA_CTRL, DRESET, 0);
}

static void bcm7340_pm_suspend(u32 flags)
{
	PRINT_PM_CALLBACK;

	/* PCI/EBI */
	BDEV_WR_F_RB(CLKGEN_PAD_CLK_PM_CTRL, DIS_CLK_33_27_PCI, 1);
	BDEV_WR_F_RB(CLKGEN_PLLMAIN_CH5_PM_CTRL, PWRDN_CH5_PLLMAIN, 1);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_81, 1);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_27, 1);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_54, 1);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_216_CG, 1);
	BDEV_WR_F_RB(CLKGEN_HIF_CLK_PM_CTRL, DIS_CLK_SPI, 1);

	/* system PLLs */
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, DRESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, ARESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, POWERDOWN, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC1_CTRL, DRESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC1_CTRL, ARESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC1_CTRL, POWERDOWN, 1);

	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_PM_CH2_CTRL, EN_CMLBUF, 0);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_PM_CH2_CTRL, PWRDN, 1);

	BDEV_WR_F_RB(VCXO_CTL_MISC_RAP_AVD_PLL_CTRL, RESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_RAP_AVD_PLL_CTRL, POWERDOWN, 1);

	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, ENB_CLOCKOUT, 1);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, DRESET, 1);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, ARESET, 1);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, PWRDN, 1);

	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_216, 1);
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_108, 1);

	/* disable self-refresh since it interferes with suspend */
	brcm_pm_set_ddr_timeout(0);
}

static void bcm7340_pm_resume(u32 flags)
{
	PRINT_PM_CALLBACK;

	/* system PLLs */
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_108, 0);
	BDEV_WR_F_RB(CLKGEN_VCXO_CLK_PM_CTRL, DIS_CLK_216, 0);

	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, PWRDN, 0);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, ARESET, 0);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, DRESET, 0);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_CTRL, ENB_CLOCKOUT, 0);

	BDEV_WR_F_RB(VCXO_CTL_MISC_RAP_AVD_PLL_CTRL, RESET, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_RAP_AVD_PLL_CTRL, POWERDOWN, 0);

	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_PM_CH2_CTRL, PWRDN, 1);
	BDEV_WR_F_RB(CLKGEN_PLLAVD_RDSP_PM_CH2_CTRL, EN_CMLBUF, 0);

	BDEV_WR_F_RB(VCXO_CTL_MISC_VC1_CTRL, POWERDOWN, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC1_CTRL, ARESET, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC1_CTRL, DRESET, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, POWERDOWN, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, ARESET, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, DRESET, 0);

	/* PCI/EBI */
	BDEV_WR_F_RB(CLKGEN_HIF_CLK_PM_CTRL, DIS_CLK_SPI, 0);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_216_CG, 0);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_54, 0);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_27, 0);
	BDEV_WR_F_RB(CLKGEN_CG_CLK_PM_CTRL, DIS_CLK_81, 0);
	BDEV_WR_F_RB(CLKGEN_PLLMAIN_CH5_PM_CTRL, PWRDN_CH5_PLLMAIN, 0);
	BDEV_WR_F_RB(CLKGEN_PAD_CLK_PM_CTRL, DIS_CLK_33_27_PCI, 0);

	brcm_ddr_phy_initialize();
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7340),
	DEF_BLOCK_PM_OP(moca, 7340),
	DEF_BLOCK_PM_OP(genet, 7340),
	DEF_BLOCK_PM_OP(network, 7340),
	DEF_SYSTEM_PM_OP(7340),
	.clk_get		= brcm_pm_clk_get,
	.initialize		= brcm_ddr_phy_initialize,
};
#endif

#if defined(CONFIG_BCM7408)
static void bcm7408_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	/*
	 * 7408B0 does not support MoCA WOL due to a h/w bug. The
	 * following code is added in case the bug is fixed in the next
	 * revision
	 * http://jira.broadcom.com/browse/HW7408-186
	 */
	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_CLK_SEL, 1);
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_GMII_TX_CLK_SEL, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_GENET_RGMII_216M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_UNIMAC_SYS_TX_27_108M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_L2_INTR_27_108M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_GMII_TX_27_108M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_MOCA_54M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_216M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_108M_CLK, 1);
		return;
	}
	BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_MOCA_ENET_HFB_27_108M_CLK, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_3, DIS_CH, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_3, EN_CMLBUF, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_4, DIS_CH, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_4, EN_CMLBUF, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_5, DIS_CH, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_5, EN_CMLBUF, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_6, DIS_CH, 1);
	BDEV_SET_RB(BCHP_CLK_MOCA_CLK_PM_CTRL, 0x6ab);
}

static void bcm7408_pm_network_enable(u32 flags)
{
	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_GENET_RGMII_216M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_UNIMAC_SYS_TX_27_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_L2_INTR_27_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_GMII_TX_27_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_MOCA_54M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_216M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_CLK_SEL, 0);
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_GMII_TX_CLK_SEL, 0);
		return;
	}
	BDEV_UNSET_RB(BCHP_CLK_MOCA_CLK_PM_CTRL, 0x6ab);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_6, DIS_CH, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_5, DIS_CH, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_5, EN_CMLBUF, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_4, DIS_CH, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_4, EN_CMLBUF, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_3, DIS_CH, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_3, EN_CMLBUF, 1);
	BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_MOCA_ENET_HFB_27_108M_CLK, 0);
}

static void bcm7408_pm_usb_disable(u32 flags)
{
	/* reset and power down all 4 ports */
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0x0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 1);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_SOFT_RESETB, 0x0);

	/* disable the clocks */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 0);
	BDEV_SET_RB(BCHP_CLK_USB_CLK_PM_CTRL,
		BCHP_CLK_USB_CLK_PM_CTRL_DIS_54M_CLK_MASK|
		BCHP_CLK_USB_CLK_PM_CTRL_DIS_108M_CLK_MASK|
		BCHP_CLK_USB_CLK_PM_CTRL_DIS_216M_CLK_MASK);

	/* disable PLL */
	BDEV_WR_F_RB(CLK_SYS_PLL_0_4, DIS_CH, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_0_4, EN_CMLBUF, 0);
}

static void bcm7408_pm_usb_enable(u32 flags)
{
	/* enable PLL */
	BDEV_WR_F_RB(CLK_SYS_PLL_0_4, EN_CMLBUF, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_0_4, DIS_CH, 0);
	/* enable the clocks */
	BDEV_UNSET_RB(BCHP_CLK_USB_CLK_PM_CTRL,
		BCHP_CLK_USB_CLK_PM_CTRL_DIS_54M_CLK_MASK|
		BCHP_CLK_USB_CLK_PM_CTRL_DIS_108M_CLK_MASK|
		BCHP_CLK_USB_CLK_PM_CTRL_DIS_216M_CLK_MASK);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 1);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 1);

	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_SOFT_RESETB, 0xE);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 0);
	/* power up all 4 ports */
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0xF);
}

static void bcm7408_pm_suspend(u32 flags)
{
	/* UART */
	if (!(brcm_pm_standby_flags & BRCM_STANDBY_VERBOSE)) {
		BDEV_WR_F_RB(CLK_SUN_27M_CLK_PM_CTRL,
			DIS_SUN_27M_CLK, 1);
		BDEV_WR_F_RB(CLK_SUN_UART_CLK_PM_CTRL,
			DIS_SUN_UART_108M_CLK, 1);
	}
	/* PAD clocks */
	BDEV_WR_F_RB(CLK_MISC, VCXOA_OUTCLK_ENABLE, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_PM_DIS_CHL_1, DIS_CH, 1);

	/* system PLLs */
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, DRESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, ARESET, 1);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, POWERDOWN, 1);
	BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1,
		CML_2_N_P_EN, 1);
	BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1,
		FREQ_DOUBLER_POWER_DOWN, 1);

	BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, DRESET, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, ARESET, 1);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, POWERDOWN, 1);

	/* MEMC0 */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_DDR_PAD_CNTRL,
		DEVCLK_OFF_ON_SELFREF, 1);
	BDEV_WR_RB(BCHP_DDR23_PHY_CONTROL_REGS_0_IDLE_PAD_CONTROL, 0x132);
	BDEV_SET_RB(BCHP_DDR23_PHY_BYTE_LANE_0_0_IDLE_PAD_CONTROL, 0xfffff);
	BDEV_SET_RB(BCHP_DDR23_PHY_BYTE_LANE_1_0_IDLE_PAD_CONTROL, 0xfffff);

}

static void bcm7408_pm_resume(u32 flags)
{
	/* system PLLs */
	BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, DRESET, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, ARESET, 0);
	BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, POWERDOWN, 0);

	BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1,
		CML_2_N_P_EN, 0);
	BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1,
		FREQ_DOUBLER_POWER_DOWN, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, DRESET, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, ARESET, 0);
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_CTRL, POWERDOWN, 0);

	/* PAD clocks */
	BDEV_WR_F_RB(VCXO_CTL_MISC_VC0_PM_DIS_CHL_1, DIS_CH, 0);
	BDEV_WR_F_RB(CLK_MISC, VCXOA_OUTCLK_ENABLE, 1);

	/* UART */
	if (!(brcm_pm_standby_flags & BRCM_STANDBY_VERBOSE)) {
		BDEV_WR_F_RB(CLK_SUN_UART_CLK_PM_CTRL,
			DIS_SUN_UART_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_SUN_27M_CLK_PM_CTRL,
			DIS_SUN_27M_CLK, 0);
	}
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(network, 7408),
	DEF_BLOCK_PM_OP(usb, 7408),
	DEF_SYSTEM_PM_OP(7408),
};
#endif

#if defined(CONFIG_BCM7420)

static void bcm7420_pm_usb_disable(u32 flags)
{
	/* power down ports */
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_PWDNB, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI1_PWDNB, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY1_PWDNB, 0);
	/* power down USB PLL */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 0);
	/* disable the clocks */
	BDEV_SET_RB(BCHP_CLK_USB_PM_CTRL,
		BCHP_CLK_USB_PM_CTRL_DIS_108M_CLK_MASK|
		BCHP_CLK_USB_PM_CTRL_DIS_216M_CLK_MASK);
	/* power down system PLL */
	BDEV_WR_F_RB(CLK_SYS_PLL_0_PLL_4, DIS_CH, 1);
}

static void bcm7420_pm_usb_enable(u32 flags)
{
	/* power up system PLL */
	BDEV_WR_F_RB(CLK_SYS_PLL_0_PLL_4, DIS_CH, 0);
	/* enable the clocks */
	BDEV_UNSET_RB(BCHP_CLK_USB_PM_CTRL,
		BCHP_CLK_USB_PM_CTRL_DIS_108M_CLK_MASK|
		BCHP_CLK_USB_PM_CTRL_DIS_216M_CLK_MASK);
	/* power up PLL */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 1);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 1);
	/* power up ports */
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_PWDNB, 3);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 3);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI1_PWDNB, 1);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY1_PWDNB, 1);
}

static void bcm7420_pm_sata_disable(u32 flags)
{
	BDEV_WR_F_RB(SUN_TOP_CTRL_GENERAL_CTRL_1, sata_ana_pwrdn, 1);
	BDEV_WR_F_RB(CLK_SATA_CLK_PM_CTRL, DIS_SATA_PCI_CLK, 1);
	BDEV_WR_F_RB(CLK_SATA_CLK_PM_CTRL, DIS_108M_CLK, 1);
	BDEV_WR_F_RB(CLK_SATA_CLK_PM_CTRL, DIS_216M_CLK, 1);
	PLL_CH_DIS(CLK_GENET_NETWORK_PLL_4);
}

static void bcm7420_pm_sata_enable(u32 flags)
{
	PLL_CH_ENA(CLK_GENET_NETWORK_PLL_4);
	BDEV_WR_F_RB(CLK_SATA_CLK_PM_CTRL, DIS_216M_CLK, 0);
	BDEV_WR_F_RB(CLK_SATA_CLK_PM_CTRL, DIS_108M_CLK, 0);
	BDEV_WR_F_RB(CLK_SATA_CLK_PM_CTRL, DIS_SATA_PCI_CLK, 0);
	BDEV_WR_F_RB(SUN_TOP_CTRL_GENERAL_CTRL_1, sata_ana_pwrdn, 0);
}

static void bcm7420_pm_moca_disable(u32 flags)
{
	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_CLK_SEL, 1);
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_GMII_TX_CLK_SEL, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_L2_INTR_27_108M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_UNIMAC_SYS_TX_27_108M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_216M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_MOCA_54M_CLK, 1);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_108M_CLK, 1);
		return;
	}

	PLL_CH_DIS(CLK_SYS_PLL_1_3);
	PLL_CH_DIS(CLK_SYS_PLL_1_4);
	PLL_CH_DIS(CLK_SYS_PLL_1_5);
	PLL_CH_DIS(CLK_SYS_PLL_1_6);
	BDEV_SET_RB(BCHP_CLK_MOCA_CLK_PM_CTRL, 0x7bf);
}

static void bcm7420_pm_moca_enable(u32 flags)
{
	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_L2_INTR_27_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL,
			DIS_MOCA_ENET_UNIMAC_SYS_TX_27_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_216M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_MOCA_54M_CLK, 0);
		BDEV_WR_F_RB(CLK_MOCA_CLK_PM_CTRL, DIS_108M_CLK, 0);
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_CLK_SEL, 0);
		BDEV_WR_F_RB(CLK_MISC, MOCA_ENET_GMII_TX_CLK_SEL, 0);
		return;
	}

	BDEV_UNSET_RB(BCHP_CLK_MOCA_CLK_PM_CTRL, 0x7bf);
	PLL_CH_ENA(CLK_SYS_PLL_1_6);
	PLL_CH_ENA(CLK_SYS_PLL_1_5);
	PLL_CH_ENA(CLK_SYS_PLL_1_4);
	PLL_CH_ENA(CLK_SYS_PLL_1_3);
}

static void bcm7420_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_WR_F_RB(CLK_MISC, GENET_CLK_SEL, 1);
		BDEV_WR_F_RB(CLK_MISC, GENET_GMII_TX_CLK_SEL, 1);
		PLL_CH_DIS(CLK_GENET_NETWORK_PLL_5);
		BDEV_SET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x143);
		return;
	}
	PLL_CH_DIS(CLK_GENET_NETWORK_PLL_3);
	PLL_CH_DIS(CLK_GENET_NETWORK_PLL_5);
	PLL_CH_DIS(CLK_GENET_NETWORK_PLL_6);
	BDEV_SET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x77F);
}

static void bcm7420_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_UNSET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x143);
		PLL_CH_ENA(CLK_GENET_NETWORK_PLL_5);
		BDEV_WR_F_RB(CLK_MISC, GENET_CLK_SEL, 0);
		BDEV_WR_F_RB(CLK_MISC, GENET_GMII_TX_CLK_SEL, 0);
		return;
	}
	BDEV_UNSET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x77F);
	PLL_CH_ENA(CLK_GENET_NETWORK_PLL_3);
	PLL_CH_ENA(CLK_GENET_NETWORK_PLL_5);
	PLL_CH_ENA(CLK_GENET_NETWORK_PLL_6);
}

static void bcm7420_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	/* TODO - this clock is shared with HDMI */
	PLL_CH_DIS(CLK_GENET_NETWORK_PLL_1);
	BDEV_WR_F_RB(CLK_GENET_NETWORK_PLL_CTRL, POWERDOWN, 1);
	BDEV_WR_F_RB(CLK_GENET_NETWORK_PLL_CTRL, RESET, 1);
}

static void bcm7420_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	BDEV_WR_F_RB(CLK_GENET_NETWORK_PLL_CTRL, RESET, 0);
	BDEV_WR_F_RB(CLK_GENET_NETWORK_PLL_CTRL, POWERDOWN, 0);
	PLL_CH_ENA(CLK_GENET_NETWORK_PLL_1);
}

static void bcm7420_pm_suspend(u32 flags)
{
	/* BT */
	PLL_CH_DIS(CLK_SYS_PLL_1_1);
	BDEV_SET_RB(BCHP_CLK_BLUETOOTH_CLK_PM_CTRL, 1);

	if (!MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, POWERDOWN, 1);
		BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, RESET, 1);
	}

	/* PCI/EBI */
	BDEV_WR_RB(BCHP_HIF_TOP_CTRL_PM_CTRL, 0x3fff);
	BDEV_WR_RB(BCHP_CLK_PCI_OUT_CLK_PM_CTRL, 0x1);
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_0_PLL_5, 0x2);

	/* system PLLs */
	if (!ANY_WOL(flags)) {
		BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1,
			FREQ_DOUBLER_POWER_DOWN, 0);
		BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1, CML_2_N_P_EN, 1);
		BDEV_WR_F_RB(CLK_SCRATCH, CML_REPEATER_2_POWERDOWN, 1);
	}

	BDEV_WR_RB(BCHP_CLK_SYS_PLL_0_PLL_6, 0x2);

	/* disable self-refresh since it interferes with suspend */
	brcm_pm_set_ddr_timeout(0);

	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_0_WORDSLICE_CNTRL_1,
		PWRDN_DLL_ON_SELFREF,
		!(brcm_pm_standby_flags & BRCM_STANDBY_DDR_PLL_ON));
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_0_WORDSLICE_CNTRL_1,
		PWRDN_DLL_ON_SELFREF,
		!(brcm_pm_standby_flags & BRCM_STANDBY_DDR_PLL_ON));
}

static void bcm7420_pm_resume(u32 flags)
{
	brcm_ddr_phy_initialize();

	/* system PLLs */
	BDEV_WR_F_RB(CLK_SCRATCH, CML_REPEATER_2_POWERDOWN, 0);
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_0_PLL_6, 0x1);

	if (!ANY_WOL(flags)) {
		BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1, CML_2_N_P_EN, 0);
		BDEV_WR_F_RB(CLK_THIRD_OT_CONTROL_1,
			FREQ_DOUBLER_POWER_DOWN, 1);
	}

	/* Sundry */
	BDEV_WR_F(CLK_SUN_27M_CLK_PM_CTRL, DIS_SUN_27M_CLK, 0);

	/* PCI/EBI */
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_0_PLL_5, 0x1);
	BDEV_WR_RB(BCHP_CLK_PCI_OUT_CLK_PM_CTRL, 0x0);
	BDEV_WR_RB(BCHP_HIF_TOP_CTRL_PM_CTRL, 0x0);

	if (!MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, RESET, 0);
		BDEV_WR_F_RB(CLK_SYS_PLL_1_CTRL, POWERDOWN, 0);
	}

	/* BT */
	BDEV_UNSET_RB(BCHP_CLK_BLUETOOTH_CLK_PM_CTRL, 1);
	PLL_CH_ENA(CLK_SYS_PLL_1_1);
}

static void bcm7420_pm_late_suspend(void)
{
	/* if MEMC1 is powered down/suspended, do not touch it */
	if (brcm_pm_memc1_power == BRCM_PM_MEMC1_ON) {
		brcm_pm_memc1_suspend();
		brcm_pm_memc1_power = BRCM_PM_MEMC1_SSPD;
	}
}

static void bcm7420_pm_early_resume(void)
{
	/* if MEMC1 was powered down before standby, do not power it up */
	if (brcm_pm_memc1_power == BRCM_PM_MEMC1_SSPD) {
		brcm_pm_memc1_resume();
		brcm_pm_memc1_power = BRCM_PM_MEMC1_ON;
	}
}

static void bcm7420_pm_pcie_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	BDEV_WR_F_RB(HIF_RGR1_SW_RESET_1, PCIE_SW_PERST, 1);
	PLL_CH_DIS(CLK_SYS_PLL_1_2);
	BDEV_SET_RB(BCHP_CLK_PCIE_CLK_PM_CTRL, 1);
}

static void bcm7420_pm_pcie_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	BDEV_UNSET_RB(BCHP_CLK_PCIE_CLK_PM_CTRL, 1);
	PLL_CH_ENA(CLK_SYS_PLL_1_2);
	BDEV_WR_F_RB(HIF_RGR1_SW_RESET_1, PCIE_SW_PERST, 0);
}

static struct clk clk_pcie = {
	.name		= "pcie",
	.disable	= &bcm7420_pm_pcie_disable,
	.enable		= &bcm7420_pm_pcie_enable,
	.refcnt		= 1, /* enabled on boot */
};

static void bcm7420_initialize(void)
{
	struct clk *nclk, *sclk;
	brcm_ddr_phy_initialize();
	/* extra clock dependency. See #SWLINUX-1913 */
	nclk = clk_get(NULL, "network");
	sclk = clk_get(NULL, "sata");
	if (nclk && sclk)
		clk_set_parent(sclk, nclk);
	__clk_dyn_add(&clk_pcie);
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(sata, 7420),
	DEF_BLOCK_PM_OP(usb, 7420),
	DEF_BLOCK_PM_OP(moca, 7420),
	DEF_BLOCK_PM_OP(genet, 7420),
	DEF_BLOCK_PM_OP(network, 7420),
	DEF_SYSTEM_PM_OP(7420),
	DEF_SYSTEM_LATE_PM_OP(7420),
	.clk_get		= brcm_pm_clk_get,
	.initialize		= bcm7420_initialize,
};
#endif

#if defined(CONFIG_BCM7468)

static void bcm7468_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		/* switch to slower 27MHz clocks */
		BDEV_WR_F_RB(CLK_MISC, GENET_CLK_SEL, 1);
		BDEV_WR_F_RB(CLK_MISC, GENET_GMII_TX_CLK_SEL, 1);
		BDEV_SET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x503);
		return;
	}

	BDEV_WR_RB(BCHP_CLK_SYS_PLL_1_4, 1);
	BDEV_SET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x767);
}

static void bcm7468_pm_genet_enable(u32 flags)
{
	if (ENET_WOL(flags)) {
		BDEV_UNSET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x503);
		BDEV_WR_F_RB(CLK_MISC, GENET_CLK_SEL, 0);
		BDEV_WR_F_RB(CLK_MISC, GENET_GMII_TX_CLK_SEL, 0);
		return;
	}

	BDEV_UNSET_RB(BCHP_CLK_GENET_CLK_PM_CTRL, 0x767);
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_1_4, 0);
}

static void bcm7468_pm_usb_disable(u32 flags)
{
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0x00);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 1);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_SOFT_RESETB, 0x00);

	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 0);
	BDEV_SET_RB(BCHP_CLK_USB_CLK_PM_CTRL, 0x07);
}

static void bcm7468_pm_usb_enable(u32 flags)
{
	BDEV_UNSET_RB(BCHP_CLK_USB_CLK_PM_CTRL, 0x07);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, XTAL_PWRDWNB, 1);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL_1, PLL_PWRDWNB, 1);

	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_SOFT_RESETB, 0xF);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, UTMI_IDDQ, 0);
	BDEV_WR_F_RB(USB_CTRL_UTMI_CTL_1, PHY_PWDNB, 0xF);
}

static void bcm7468_pm_suspend(u32 flags)
{
	/* SDIO */
	BDEV_WR_F_RB(CLK_HIF_SDIO_CLK_PM_CTRL, DIS_HIF_SDIO_48M_CLK, 1);
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_1_1, 1);

	/* EBI */
	BDEV_SET_RB(BCHP_HIF_TOP_CTRL_PM_CTRL, 0x2ff0);

	/* system PLLs */
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_0_3, 0x02);

	if (!ENET_WOL(flags))
		/* this PLL is needed for EPHY */
		BDEV_SET_RB(BCHP_CLK_SYS_PLL_1_CTRL, 0x83);

	BDEV_WR_RB(BCHP_VCXO_CTL_MISC_AC1_CTRL, 0x06);
	BDEV_SET_RB(BCHP_VCXO_CTL_MISC_VC0_CTRL, 0x0b);
}

static void bcm7468_pm_resume(u32 flags)
{
	/* system PLLs */
	BDEV_UNSET_RB(BCHP_VCXO_CTL_MISC_VC0_CTRL, 0x0b);
	BDEV_WR_RB(BCHP_VCXO_CTL_MISC_AC1_CTRL, 0x00);
	BDEV_UNSET_RB(BCHP_CLK_SYS_PLL_1_CTRL, 0x83);
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_0_3, 0x01);

	/* EBI */
	BDEV_UNSET_RB(BCHP_HIF_TOP_CTRL_PM_CTRL, 0x2ff0);

	/* SDIO */
	BDEV_WR_RB(BCHP_CLK_SYS_PLL_1_1, 0);
	BDEV_WR_F_RB(CLK_HIF_SDIO_CLK_PM_CTRL, DIS_HIF_SDIO_48M_CLK, 0);
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7468),
	DEF_BLOCK_PM_OP(genet, 7468),
	DEF_SYSTEM_PM_OP(7468),
};
#endif

/* 40nm chips start here */

#if	defined(CONFIG_BCM7425) || \
	defined(CONFIG_BCM7429) || \
	defined(CONFIG_BCM7435) || \
	defined(CONFIG_BCM7346) || \
	defined(CONFIG_BCM7231) || \
	defined(CONFIG_BCM7552) || \
	defined(CONFIG_BCM7584) || \
	defined(CONFIG_BCM7563) || \
	defined(CONFIG_BCM7344) || \
	defined(CONFIG_BCM7358) || \
	defined(CONFIG_BCM7360) || \
	defined(CONFIG_BCM7362)
static __maybe_unused int mem_power_down = 1;

#undef PLL_CH_ENA
#undef PLL_CH_DIS
#undef PLL_ENA
#undef PLL_DIS

#define PLL_CH_DIS(pll, ch) do { \
	BDEV_WR_F_RB(pll ## _CH_ ## ch, POST_DIVIDER_HOLD_CH ## ch, 1); \
	BDEV_WR_F_RB(pll ## _CH_ ## ch, CLOCK_DIS_CH ## ch, 1); \
} while (0)

#define PLL_CH_ENA(pll, ch) do { \
	BDEV_WR_F_RB(pll ## _CH_ ## ch, CLOCK_DIS_CH ## ch, 0); \
	BDEV_WR_F_RB(pll ## _CH_ ## ch, POST_DIVIDER_HOLD_CH ## ch, 0); \
} while (0)

#define PLL_DIS(pll) do { \
	BDEV_WR_F_RB(pll ## _RESET, RESETD, 1); \
	BDEV_WR_F_RB(pll ## _RESET, RESETA, 1); \
	BDEV_WR_F_RB(pll ## _PWRDN, PWRDN_PLL, 1); \
} while (0)

#define PLL_ENA(pll) do { \
	BDEV_WR_F_RB(pll ## _PWRDN, PWRDN_PLL, 0); \
	BDEV_WR_F_RB(pll ## _RESET, RESETA, 0); \
	BDEV_WR_F_RB(pll ## _RESET, RESETD, 0); \
	do { \
		mdelay(1); \
	} while (!BDEV_RD_F(pll ## _LOCK_STATUS, LOCK)); \
} while (0)


/* IMPORTANT: must be done in TWO steps per RDB note */
#define POWER_UP_MEMORY_3(core, mask, inst) \
do { \
	BDEV_WR_F_RB(CLKGEN_ ## core ## _POWER_SWITCH_MEMORY ## inst,\
		mask ## _POWER_SWITCH_MEMORY ## inst, 2); \
	udelay(10); \
	BDEV_WR_F_RB(CLKGEN_ ## core ## _POWER_SWITCH_MEMORY ## inst,\
		mask ## _POWER_SWITCH_MEMORY ## inst, 0); \
} while (0)

#define POWER_UP_MEMORY_1(core) POWER_UP_MEMORY_3(core, core, \
	)
#define POWER_UP_MEMORY_1ii(core) \
	POWER_UP_MEMORY_2(core ## _INST, core ## _INST)
#define POWER_UP_MEMORY_2(core, mask) POWER_UP_MEMORY_3(core, mask, \
	)
#define POWER_UP_MEMORY_2i(core, mask) POWER_UP_MEMORY_3(core ## _INST, mask, \
	)
#define POWER_UP_MEMORY_3i(core, mask, inst) \
	POWER_UP_MEMORY_3(core ## _INST, mask, inst)

#define SRAM_OFF_3(core, mask, inst) \
do { \
	if (mem_power_down) \
		BDEV_WR_F_RB( \
			CLKGEN_ ## core ## _POWER_SWITCH_MEMORY ## inst, \
			mask ## _POWER_SWITCH_MEMORY ## inst, 3); \
	else \
		BDEV_WR_F_RB( \
			CLKGEN_ ## core ## _MEMORY_STANDBY_ENABLE ## inst, \
			mask ## _MEMORY_STANDBY_ENABLE ## inst, 1); \
} while (0)

#define SRAM_OFF_1i(core) SRAM_OFF_3(core ## _INST, core, \
	)
#define SRAM_OFF_1(core) SRAM_OFF_3(core, core, \
	)
#define SRAM_OFF_2i(core, mask) SRAM_OFF_3(core ## _INST, mask, \
	)
#define SRAM_OFF_3i(core, mask, inst) SRAM_OFF_3(core ## _INST, mask, inst)
#define SRAM_OFF_2(core, mask) SRAM_OFF_3(core, mask, \
	)

#define SRAM_ON_3(core, mask, inst) \
do { \
	if (mem_power_down) \
		POWER_UP_MEMORY_3(core, mask, inst); \
	else \
	BDEV_WR_F_RB(CLKGEN_ ## core ## _MEMORY_STANDBY_ENABLE ## inst, \
		mask ## _MEMORY_STANDBY_ENABLE ## inst, 0); \
} while (0)

#define SRAM_ON_1i(core) SRAM_ON_3(core ## _INST, core, \
	)
#define SRAM_ON_1(core) SRAM_ON_3(core, core, \
	)
#define SRAM_ON_2i(core, mask) SRAM_ON_3(core ## _INST, mask, \
	)
#define SRAM_ON_2(core, mask) SRAM_ON_3(core, mask, \
	)
#define SRAM_ON_3i(core, mask, inst) SRAM_ON_3(core ## _INST, mask, inst)
#endif

/******************************************************************
 *   USB recovery after S3 warm boot
 * These parameters are mostly configured by CFE on cold boot, some
 * of them are set during HC reset code which we do not execute on
 * warm boot because it re-allocates memory.
 * The code will probably work on all 40nm chips, but we will see...
 ******************************************************************/
static u32 usb_cfg[4];

static __maybe_unused void bcm40nm_pm_usb_disable_s3(void)
{
	if (!brcm_pm_deep_sleep())
		return;
#ifdef BCHP_USB_CTRL_SETUP
	usb_cfg[0] = BDEV_RD(BCHP_USB_CTRL_SETUP);
	usb_cfg[1] = BDEV_RD(BCHP_USB_CTRL_EBRIDGE);
#endif
#ifdef BCHP_USB1_CTRL_SETUP
	usb_cfg[2] = BDEV_RD(BCHP_USB1_CTRL_SETUP);
	usb_cfg[3] = BDEV_RD(BCHP_USB1_CTRL_EBRIDGE);
#endif
}

static __maybe_unused void bcm40nm_pm_usb_enable_s3(void)
{
	if (!brcm_pm_deep_sleep())
		return;
	printk(KERN_DEBUG "Restoring USB HC state from S3 suspend\n");
#ifdef BCHP_USB_CTRL_SETUP
	BDEV_WR_RB(BCHP_USB_CTRL_SETUP, usb_cfg[0]);
	BDEV_WR_RB(BCHP_USB_CTRL_EBRIDGE, usb_cfg[1]);
#endif
#ifdef BCHP_USB1_CTRL_SETUP
	BDEV_WR_RB(BCHP_USB1_CTRL_SETUP, usb_cfg[2]);
	BDEV_WR_RB(BCHP_USB1_CTRL_EBRIDGE, usb_cfg[3]);
#endif
	mdelay(500);
	bchip_usb_init();
}

#if defined(CONFIG_BCM7425) || defined(CONFIG_BCM7429) \
	|| defined(CONFIG_BCM7346) || defined(CONFIG_BCM7435)

#define BRCM_BASE_CLK		3600 /* base clock in MHz */

/*
 * work around RDB name inconsistencies.
 */
#if defined(BCHP_CLKGEN_USB0_INST_CLOCK_DISABLE_DISABLE_USB_54_MDIO_CLOCK_MASK)
#define BCHP_CLKGEN_USB0_INST_CLOCK_DISABLE_DISABLE_USB0_54_MDIO_CLOCK_MASK \
	BCHP_CLKGEN_USB0_INST_CLOCK_DISABLE_DISABLE_USB_54_MDIO_CLOCK_MASK
#define BCHP_CLKGEN_USB0_INST_CLOCK_DISABLE_DISABLE_USB0_54_MDIO_CLOCK_SHIFT \
	BCHP_CLKGEN_USB0_INST_CLOCK_DISABLE_DISABLE_USB_54_MDIO_CLOCK_SHIFT
#endif

#if defined(BCHP_CLKGEN_USB1_INST_CLOCK_DISABLE_DISABLE_USB_54_MDIO_CLOCK_MASK)
#define BCHP_CLKGEN_USB1_INST_CLOCK_DISABLE_DISABLE_USB1_54_MDIO_CLOCK_MASK \
	BCHP_CLKGEN_USB1_INST_CLOCK_DISABLE_DISABLE_USB_54_MDIO_CLOCK_MASK
#define BCHP_CLKGEN_USB1_INST_CLOCK_DISABLE_DISABLE_USB1_54_MDIO_CLOCK_SHIFT \
	BCHP_CLKGEN_USB1_INST_CLOCK_DISABLE_DISABLE_USB_54_MDIO_CLOCK_SHIFT
#endif

#if defined(BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB_SCB_CLOCK_ENABLE_MASK)
#define BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB0_SCB_CLOCK_ENABLE_MASK \
	BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB_SCB_CLOCK_ENABLE_MASK
#define BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB0_SCB_CLOCK_ENABLE_SHIFT \
	BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB_SCB_CLOCK_ENABLE_SHIFT

#define BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB0_108_CLOCK_ENABLE_MASK \
	BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB_108_CLOCK_ENABLE_MASK
#define BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB0_108_CLOCK_ENABLE_SHIFT \
	BCHP_CLKGEN_USB0_INST_CLOCK_ENABLE_USB_108_CLOCK_ENABLE_SHIFT
#endif

#if defined(BCHP_CLKGEN_MOCAMAC_TOP_INST_CLOCK_ENABLE)
#define BCHP_CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE \
	BCHP_CLKGEN_MOCAMAC_TOP_INST_CLOCK_ENABLE
#define BCHP_CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE_MOCA_SCB_CLOCK_ENABLE_MASK \
	BCHP_CLKGEN_MOCAMAC_TOP_INST_CLOCK_ENABLE_MOCA_SCB_CLOCK_ENABLE_MASK
#define BCHP_CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE_MOCA_SCB_CLOCK_ENABLE_SHIFT \
	BCHP_CLKGEN_MOCAMAC_TOP_INST_CLOCK_ENABLE_MOCA_SCB_CLOCK_ENABLE_SHIFT
#define BCHP_CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE_MOCA_108_CLOCK_ENABLE_MASK \
	BCHP_CLKGEN_MOCAMAC_TOP_INST_CLOCK_ENABLE_MOCA_108_CLOCK_ENABLE_MASK
#define BCHP_CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE_MOCA_108_CLOCK_ENABLE_SHIFT \
	BCHP_CLKGEN_MOCAMAC_TOP_INST_CLOCK_ENABLE_MOCA_108_CLOCK_ENABLE_SHIFT
#endif

static void bcm40nm_pm_usb_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	bcm40nm_pm_usb_disable_s3();

	/* USB0 */
	/* power down USB PHY */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power down USB PLL */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	SRAM_OFF_1i(USB0);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_DISABLE,
		DISABLE_USB0_54_MDIO_CLOCK, 1);

	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 0);

	/* USB1 */
	/* power down USB PHY */
	BDEV_SET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power down USB PLL */
	BDEV_UNSET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	SRAM_OFF_1i(USB1);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_DISABLE,
		DISABLE_USB1_54_MDIO_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_108_CLOCK_ENABLE, 0);

#if defined(BCHP_CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL_CH_4)
	/* power down PLL */
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
#endif
}

static void bcm40nm_pm_usb_enable(u32 flags)
{
	PRINT_PM_CALLBACK;
	/* power up PLL */
#if defined(BCHP_CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL_CH_4)
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
#endif

	/* USB0 */
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_DISABLE,
		DISABLE_USB0_54_MDIO_CLOCK, 0);

	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 1);

	SRAM_ON_1i(USB0);

	/* power up USB PLL */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);
	/* power up USB PHY */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	/* USB1 */
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_DISABLE,
		DISABLE_USB1_54_MDIO_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_108_CLOCK_ENABLE, 1);

	SRAM_ON_1i(USB1);

	/* power up USB PLL */
	BDEV_SET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power up USB PHY */
	BDEV_UNSET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	bcm40nm_pm_usb_enable_s3();
}

static void bcm40nm_pm_sata_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	SRAM_OFF_2i(SATA3_TOP, SATA3);

	/* gate the clocks */
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_INST_CLOCK_ENABLE,
		SATA3_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_INST_CLOCK_ENABLE,
		SATA3_108_CLOCK_ENABLE, 0);
}

static void bcm40nm_pm_sata_enable(u32 flags)
{
	PRINT_PM_CALLBACK;
	/* reenable the clocks */
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_INST_CLOCK_ENABLE,
		SATA3_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_INST_CLOCK_ENABLE,
		SATA3_108_CLOCK_ENABLE, 1);

	SRAM_ON_2i(SATA3_TOP, SATA3);

}


static void bcm40nm_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

#if defined(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_POWER_SWITCH_MEMORY_A)
	SRAM_OFF_3i(DUAL_GENET_TOP_DUAL_RGMII, GENET0, _A);
#endif

#if defined(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET0)
	if (ENET_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_CLOCK_SELECT_GENET0, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_GMII_CLOCK_SELECT_GENET0, 1);

		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x106);
	} else {
		/* every clock except AON */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    0xe);

		/* Every genet0 clock except 108 */
		BDEV_UNSET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x1fe);
	}

#elif defined(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET0)

	if (ENET_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_CLOCK_SELECT_GENET0, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_GMII_CLOCK_SELECT_GENET0, 1);
		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x106);
	} else {
		/* every clock except AON */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET0,
		    0x1e);

		/* Every genet0 clock except 108  */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x1fe);
	}
#else

	if (ENET_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET0_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET0_GMII_CLOCK_SELECT, 1);

		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x43); /* used to be 0x53 */
	} else {
		/* system slow clock, pm clock */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    3);

		/* Every gnet0 clock */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x7F);
	}
#endif
}

static void bcm40nm_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

#if defined(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_POWER_SWITCH_MEMORY_A)
	SRAM_ON_3i(DUAL_GENET_TOP_DUAL_RGMII, GENET0, _A);
#endif

#if defined(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET0)
	if (ENET_WOL(flags)) {
		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_CLOCK_SELECT_GENET0, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_GMII_CLOCK_SELECT_GENET0, 0);

		/* 250, EEE, UNIMAC-TX */
		BDEV_SET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x106);
	} else {
		/* every genet0 clock */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    0xf);

		/* Every genet0 clock */
		BDEV_SET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x1ff);
	}
#elif defined(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET0)
	if (ENET_WOL(flags)) {
		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_CLOCK_SELECT_GENET0, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET0,
		    GENET0_GMII_CLOCK_SELECT_GENET0, 0);
		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x106);
	} else {
		/* every genet0 clock */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET0,
		    0x1f);

		/* Every genet0 clock */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET0,
		    0x1ff);
	}
#else
	if (ENET_WOL(flags)) {
		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET0_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET0_GMII_CLOCK_SELECT, 0);

		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x43);
	} else {
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    3);

		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x7F);
	}
#endif

}

static void bcm40nm_pm_genet1_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

#if defined(CONFIG_BCM7425)
	if (mem_power_down)
		/* power down memory */
	    BDEV_WR_F_RB(
		CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_POWER_SWITCH_MEMORY_B,
		GENET1_POWER_SWITCH_MEMORY_B, 3);
	else
		/* memory to standby */
	    BDEV_WR_F_RB(
		CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_MEMORY_STANDBY_ENABLE_A,
		GENET1_MEMORY_STANDBY_ENABLE_A, 1);
#endif

#if defined(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET1)
	if (MOCA_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_CLOCK_SELECT_GENET1, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_GMII_CLOCK_SELECT_GENET1, 1);
		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x106);
	} else {
		/* every clock except AON */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    0xe0);

		/* Every genet1 clock except 108 */
		BDEV_UNSET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x1fe);
	}
#elif defined(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET1)
	if (MOCA_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_CLOCK_SELECT_GENET1, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_GMII_CLOCK_SELECT_GENET1, 1);
		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x106);
	} else {
		/* every clock except AON */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET1,
		    0x1e);

		/* Every genet1 clock except 108 */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x1fe);
	}
#else
	if (MOCA_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET1_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET1_GMII_CLOCK_SELECT, 1);

		/*
		 * Do not clear GENET1_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x2180); /* 0x2980 */
	} else {
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x3F80);
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    0xC);
	}
#endif
}

static void bcm40nm_pm_genet1_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

#if defined(CONFIG_BCM7425)
	if (mem_power_down)
		/* power up memory */
		POWER_UP_MEMORY_3i(DUAL_GENET_TOP_DUAL_RGMII, GENET1, _B);
	else
		/* memory from standby */
BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_MEMORY_STANDBY_ENABLE_A,
		GENET1_MEMORY_STANDBY_ENABLE_A, 0);
#endif

#if defined(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET1)
	if (MOCA_WOL(flags)) {
		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_CLOCK_SELECT_GENET1, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_GMII_CLOCK_SELECT_GENET1, 0);

		/* 250, EEE, UNIMAC-TX */
		BDEV_SET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x106);
	} else {
		/* every genet 1 clock */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		    0xf0);

		/* Every genet 1 clock */
		BDEV_SET(
		 BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x1ff);
	}
#elif defined(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET1)
	if (MOCA_WOL(flags)) {
		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_CLOCK_SELECT_GENET1, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT_GENET1,
		    GENET1_GMII_CLOCK_SELECT_GENET1, 0);
		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x106);
	} else {
		/* every genet 1 clock */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET1,
		    0x1f);

		/* Every genet 1 clock */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET1,
		    0x1ff);
	}
#else
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		0xC);

	if (MOCA_WOL(flags)) {
		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET1_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_SELECT,
		    GENET1_GMII_CLOCK_SELECT, 0);

		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x2180);
	} else {
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		    0x3F80);
	}
#endif

}

static void bcm40nm_pm_moca_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

#if defined(BCHP_CLKGEN_MOCA_TOP_INST_POWER_SWITCH_MEMORY)
	SRAM_OFF_2i(MOCA_TOP, MOCA);
#endif

	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_108_CLOCK_ENABLE, 0);

	if (!MOCA_WOL(flags)) {
		PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 0);
		PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 1);
		PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 2);
#if !defined(CONFIG_BCM7435)
		PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 3);
#endif
	}
}

static void bcm40nm_pm_moca_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (!MOCA_WOL(flags)) {
		PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 0);
		PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 1);
		PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 2);
#if !defined(CONFIG_BCM7435)
		PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 3);
#endif
	}

	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_108_CLOCK_ENABLE, 1);

#if defined(BCHP_CLKGEN_MOCA_TOP_INST_POWER_SWITCH_MEMORY)
	SRAM_ON_2i(MOCA_TOP, MOCA);
#endif
}

static int bcm40nm_pm_set_moca_cpu_rate(unsigned long rate)
{
#ifdef BCHP_CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_0
	BDEV_WR_F(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_0,
		MDIV_CH0, BRCM_BASE_CLK/(rate/1000000));
	return 0;
#else
	return -EINVAL;
#endif
}

static int bcm40nm_pm_set_moca_phy_rate(unsigned long rate)
{
#ifdef BCHP_CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_1
	BDEV_WR_F(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_1,
		MDIV_CH1, BRCM_BASE_CLK/(rate/1000000));
	return 0;
#else
	return -ENOENT;
#endif
}

#endif

#if defined(CONFIG_BCM7231)

static void bcm7231_pm_usb_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	bcm40nm_pm_usb_disable_s3();
	/* USB0 */
	/* power down USB PHY */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power down USB PLL */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	SRAM_OFF_1(USB0);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB0_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_USB0_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB0_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 0);

	/* USB1 */
	/* power down USB PHY */
	BDEV_SET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power down USB PLL */
	BDEV_UNSET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	SRAM_OFF_1(USB1);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB1_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_USB1_CLOCK_ENABLE, USB1_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB1_CLOCK_ENABLE, USB1_108_CLOCK_ENABLE, 0);

	/* power down PLL */
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
}

static void bcm7231_pm_usb_enable(u32 flags)
{
	PRINT_PM_CALLBACK;
	/* power up PLL */
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);

	/* USB0 */
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB0_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_USB0_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB0_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 1);

	SRAM_ON_1(USB0);

	/* power up USB PLL */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);
	/* power up USB PHY */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	/* USB1 */
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB1_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_USB1_CLOCK_ENABLE, USB1_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB1_CLOCK_ENABLE, USB1_108_CLOCK_ENABLE, 1);

	SRAM_ON_1(USB1);

	/* power up USB PLL */
	BDEV_SET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);
	/* power up USB PHY */
	BDEV_UNSET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	bcm40nm_pm_usb_enable_s3();
}

static void bcm7231_pm_sata_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	SRAM_OFF_2(SATA3_TOP, SATA3);

	/* gate the clocks */
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_108_CLOCK_ENABLE, 0);
}

static void bcm7231_pm_sata_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	/* reenable the clocks */
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_108_CLOCK_ENABLE, 1);

	SRAM_ON_2(SATA3_TOP, SATA3);

}

static void bcm7231_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 1);
		/* Disabling L2_INTR clock breaks HFB pattern matching ??? */
		BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
			   0x43); /* used to be 0x53 */
		return;
	}

	SRAM_OFF_2(DUAL_GENET_TOP_DUAL_RGMII, GENET0);

	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE, 7);
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE, 0x7F);
}

static void bcm7231_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
			   0x43); /* used to be 0x53 */
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 0);
		return;
	}

	SRAM_ON_2(DUAL_GENET_TOP_DUAL_RGMII, GENET0);

	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE, 7);
	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE, 0x7F);
}

static void bcm7231_pm_genet1_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET1_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET1_GMII_CLOCK_SELECT, 1);
		BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
			   0x2980);
		return;
	}

	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE, 0x38);
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE, 0x3F80);
}

static void bcm7231_pm_genet1_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
			   0x2980);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET1_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
			GENET1_GMII_CLOCK_SELECT, 0);
		return;
	}

	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE, 0x38);
	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE, 0x3F80);
}

static void bcm7231_pm_suspend(u32 flags)
{
}

static void bcm7231_pm_resume(u32 flags)
{
}

static void bcm7231_pm_initialize(void)
{
	/* test scan clocks */
	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE, 0xC0);
	brcm_ddr_phy_initialize();
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7231),
	DEF_BLOCK_PM_OP(sata, 7231),
	DEF_BLOCK_PM_OP(genet, 7231),
	DEF_BLOCK_PM_OP(genet1, 7231),
	DEF_SYSTEM_PM_OP(7231),
	.clk_get		= brcm_pm_clk_get,
	.initialize		= bcm7231_pm_initialize,
};
#endif

#if defined(CONFIG_BCM7344)
static void bcm7344_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 0);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 3);

	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xE0);
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0xC000);
}

static void bcm7344_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0xC000);
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xE0);

	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 0);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 3);
}

static void bcm7344_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 1);
		/*
		 * NOTE: disabling L2INTR clock
		 * breaks ACPI pattern detection
		 */
		BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
			   0x43);
		return;
	}

	SRAM_OFF_3(DUAL_GENET_TOP_RGMII_INST, GENET0, _A);

	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0x3);
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0x7F);
}

static void bcm7344_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		/* switch to fast clock */
		BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
			   0x43);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 0);
		return;
	}

	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0x3);
	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0x7F);

	SRAM_ON_3(DUAL_GENET_TOP_RGMII_INST, GENET0, _A);
}

static void bcm7344_pm_genet1_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		/* TODO */
		return;
	}

/*	SRAM_OFF_3(DUAL_GENET_TOP_RGMII_INST, GENET1, _B); */

	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0x1C);
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0x1F80);
}

static void bcm7344_pm_genet1_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		/* TODO */
		return;
	}

/*	SRAM_ON_3(DUAL_GENET_TOP_RGMII_INST, GENET1, _B); */

	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0x1C);
	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0x1F80);
}

static void bcm7344_pm_usb_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	bcm40nm_pm_usb_disable_s3();

	/* USB0 */

	/* power down PHY and PLL */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_IDDQ_PWRDN, 1);

	/* power down memory */
	SRAM_OFF_2i(USB0, USB0);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 0);

	/* USB1 */

	/* power down PHY and PLL */
	BDEV_UNSET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);
	BDEV_SET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	/* power down memory */
	SRAM_OFF_2i(USB1, USB1);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_108_CLOCK_ENABLE, 0);

	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 5);
}

static void bcm7344_pm_usb_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 5);

	/* USB0 */
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB0_INST_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 1);

	/* power up memory */
	SRAM_ON_2i(USB0, USB0);

	/* power up PHY and PLL */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_PWRDWNB, 1);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_IDDQ_PWRDN, 0);

	/* USB1 */
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_DISABLE,
		DISABLE_USB_54_MDIO_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB1_INST_CLOCK_ENABLE, USB1_108_CLOCK_ENABLE, 1);

	/* power up memory */
	SRAM_ON_2i(USB1, USB1);

	/* power up PHY and PLL */
	BDEV_SET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);
	BDEV_UNSET(BCHP_USB1_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	bcm40nm_pm_usb_enable_s3();
}

static void bcm7344_pm_moca_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		/* TODO */
		return;
	}

	SRAM_OFF_2i(MOCA_TOP, MOCA);

	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_108_CLOCK_ENABLE, 0);

	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 2);
	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 3);
}

static void bcm7344_pm_moca_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (MOCA_WOL(flags)) {
		/* TODO */
		return;
	}

	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 2);
	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 3);

	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_MOCA_TOP_INST_CLOCK_ENABLE,
		MOCA_108_CLOCK_ENABLE, 1);

	SRAM_ON_2i(MOCA_TOP, MOCA);
}

static void bcm7344_pm_suspend(u32 flags)
{
	PRINT_PM_CALLBACK;

	PLL_DIS(CLKGEN_PLL_RAAGA_PLL);
	PLL_CH_DIS(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 0);
	PLL_CH_DIS(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 2);
	PLL_DIS(CLKGEN_PLL_VCXO_PLL);

	if (!ANY_WOL(flags))
		PLL_DIS(CLKGEN_PLL_SYS1_PLL);

	if (!MOCA_WOL(flags))
		PLL_DIS(CLKGEN_PLL_MOCA_PLL);

	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 2);
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 5);
}

static void bcm7344_pm_resume(u32 flags)
{
	PRINT_PM_CALLBACK;

	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 5);
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 2);

	if (!MOCA_WOL(flags))
		PLL_ENA(CLKGEN_PLL_MOCA_PLL);

	if (!ANY_WOL(flags))
		PLL_ENA(CLKGEN_PLL_SYS1_PLL);

	PLL_ENA(CLKGEN_PLL_VCXO_PLL);
	PLL_CH_ENA(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 0);
	PLL_CH_ENA(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 2);
	PLL_ENA(CLKGEN_PLL_RAAGA_PLL);
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7344),
	DEF_BLOCK_PM_OP(network, 7344),
	DEF_BLOCK_PM_OP(genet, 7344),
	DEF_BLOCK_PM_OP(genet1, 7344),
	DEF_BLOCK_PM_OP(moca, 7344),
	DEF_SYSTEM_PM_OP(7344),
	.clk_get		= brcm_pm_clk_get,
	.initialize		= brcm_ddr_phy_initialize,
};
#endif

#if defined(CONFIG_BCM7346)

static void bcm7346_pm_network_disable(u32 flags)
{
	if (ANY_WOL(flags))
		return;

	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 0);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 3);

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET_ALWAYSON_CLOCK, 1);
}

static void bcm7346_pm_network_enable(u32 flags)
{
	if (ANY_WOL(flags))
		return;

	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 0);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 3);

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET_ALWAYSON_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 1);

}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 40nm),
	DEF_BLOCK_PM_OP(sata, 40nm),
	DEF_BLOCK_PM_OP(network, 7346),
	DEF_BLOCK_PM_OP(moca, 40nm),
	DEF_BLOCK_PM_OP(genet, 40nm),
	DEF_BLOCK_PM_OP(genet1, 40nm),
	.clk_get		= brcm_pm_clk_get,
	.initialize		= brcm_ddr_phy_initialize,
};

#endif

#if defined(CONFIG_BCM7425) || defined(CONFIG_BCM7435)

static void bcm7425_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;


	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 0);
#if defined(CONFIG_BCM7435)
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET0,
		GENET0_108_CLOCK_ENABLE_GENET0, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET1,
		GENET1_108_CLOCK_ENABLE_GENET1, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET0_ALWAYSON_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET1_ALWAYSON_CLOCK, 1);
#else
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET_ALWAYSON_CLOCK, 1);
#endif
}

static void bcm7425_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

#if defined(CONFIG_BCM7435)
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET0_ALWAYSON_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET1_ALWAYSON_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET0,
		GENET0_108_CLOCK_ENABLE_GENET0, 1);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE_GENET1,
		GENET1_108_CLOCK_ENABLE_GENET1, 1);
#else
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_DISABLE,
		DISABLE_GENET_ALWAYSON_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 1);
#endif

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 1);
}

/*
 * shared PLL usage.
 *
 * PLL  7429    7425    7435
 * moca
 *  0   moca    moca    moca
 *  1   moca    moca    moca
 *  2   moca    moca    moca
 *  3   moca    moca    v3d
 *  4   sdio    sdio    m2mc,sdio
 *  5   usb0    spi     unused
 *
 * net
 *  0   genet   genet   genet
 *  1   genet   genet   genet
 *  2   genet   genet   genet
 *  3   spi     unused  spi
 */

static void bcm7425_pm_suspend(u32 flags)
{
	/*
	 * Some PLLs are shared between blocks, so disable them last.
	 * Do not disable network PLLs if we need WOL.
	 */
#if defined(CONFIG_BCM7435)
	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 3);
#endif

	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 4);

#if defined(BCHP_CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_5)
	PLL_CH_DIS(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 5);
#endif

#if defined(BCHP_CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL_CH_3)
	PLL_CH_DIS(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 3);
#endif

	if (ANY_WOL(flags) == 0) {
		PLL_CH_DIS(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 0);
		PLL_CH_DIS(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 1);
		PLL_CH_DIS(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 2);
		PLL_DIS(CLKGEN_PLL_NETWORK_PLL);
	}
}

static void bcm7425_pm_resume(u32 flags)
{
	/*
	 * some PLLs are shared between blocks, so enable them early
	 */
	PLL_ENA(CLKGEN_PLL_NETWORK_PLL);
	PLL_CH_ENA(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 0);
	PLL_CH_ENA(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 2);
#if defined(BCHP_CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL_CH_3)
	PLL_CH_ENA(CLKGEN_PLL_NETWORK_PLL_CHANNEL_CTRL, 3);
#endif

#if defined(CONFIG_BCM7435)
	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 3);
#endif
	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 4);

#if defined(BCHP_CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL_CH_5)
	PLL_CH_ENA(CLKGEN_PLL_MOCA_PLL_CHANNEL_CTRL, 5);
#endif
}

static void bcm7425_pm_late_suspend(void)
{
	/*
	 * 7425 will not go to S2/S3 standby unless both memory controllers
	 * are in SSPD mode. MEMC0 is handled by AON/BSP, but MEMC1 must
	 * be put to SSPD by the software.
	 * So, if MEMC1 was previously powered down, power it up and then
	 * suspend
	 */
	if (brcm_pm_memc1_power == BRCM_PM_MEMC1_OFF) {
		brcm_pm_memc1_powerup();
		brcm_pm_memc1_power = BRCM_PM_MEMC1_ON;
	}
	if (brcm_pm_memc1_power == BRCM_PM_MEMC1_ON) {
		brcm_pm_memc1_suspend();
		brcm_pm_memc1_power = BRCM_PM_MEMC1_SSPD;
	}
}

static void bcm7425_pm_early_resume(void)
{
	if (brcm_pm_memc1_power == BRCM_PM_MEMC1_SSPD) {
		brcm_pm_memc1_resume();
		brcm_pm_memc1_power = BRCM_PM_MEMC1_ON;
	}
}

static void bcm7425_pm_pcie_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	BDEV_WR_F(PCIE_MISC_HARD_PCIE_HARD_DEBUG, SERDES_IDDQ, 1);
	PLL_CH_DIS(CLKGEN_PLL_HIF_PLL_CHANNEL_CTRL, 0);
#if defined(BCHP_CLKGEN_PLL_HIF_PLL_CHANNEL_CTRL_CH_1)
	PLL_CH_DIS(CLKGEN_PLL_HIF_PLL_CHANNEL_CTRL, 1);
#endif
	PLL_DIS(CLKGEN_PLL_HIF_PLL);
}

static void bcm7425_pm_pcie_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	PLL_ENA(CLKGEN_PLL_HIF_PLL);
	PLL_CH_ENA(CLKGEN_PLL_HIF_PLL_CHANNEL_CTRL, 0);
#if defined(BCHP_CLKGEN_PLL_HIF_PLL_CHANNEL_CTRL_CH_1)
	PLL_CH_ENA(CLKGEN_PLL_HIF_PLL_CHANNEL_CTRL, 1);
#endif
	BDEV_WR_F(PCIE_MISC_HARD_PCIE_HARD_DEBUG, SERDES_IDDQ, 0);
}

static struct clk clk_pcie = {
	.name		= "pcie",
	.disable	= &bcm7425_pm_pcie_disable,
	.enable		= &bcm7425_pm_pcie_enable,
	.refcnt		= 1, /* enabled on boot */
};

static void bcm7425_initialize(void)
{
	brcm_ddr_phy_initialize();
	__clk_dyn_add(&clk_pcie);
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 40nm),
	DEF_BLOCK_PM_OP(sata, 40nm),
	DEF_BLOCK_PM_OP(network, 7425),
	DEF_BLOCK_PM_OP(moca, 40nm),
	DEF_BLOCK_PM_OP(genet, 40nm),
	DEF_BLOCK_PM_OP(genet1, 40nm),
	DEF_SYSTEM_PM_OP(7425),
	DEF_SYSTEM_LATE_PM_OP(7425),
	.moca.set_cpu_rate	= bcm40nm_pm_set_moca_cpu_rate,
	.moca.set_phy_rate	= bcm40nm_pm_set_moca_phy_rate,
	.clk_get		= brcm_pm_clk_get,
	.initialize		= bcm7425_initialize,
};
#endif

#if defined(CONFIG_BCM7429)

static void bcm7429_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	/* need to control network PLLs for 7429 */

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 0);

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET0,
		GENET0_108_CLOCK_ENABLE_GENET0, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET1,
		GENET1_108_CLOCK_ENABLE_GENET1, 0);

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET0,
		DISABLE_GENET0_ALWAYSON_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET1,
		DISABLE_GENET1_ALWAYSON_CLOCK, 1);
}

static void bcm7429_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET0,
		DISABLE_GENET0_ALWAYSON_CLOCK, 0);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_DISABLE_GENET1,
		DISABLE_GENET1_ALWAYSON_CLOCK, 0);

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET0,
		GENET0_108_CLOCK_ENABLE_GENET0, 1);
	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE_GENET1,
		GENET1_108_CLOCK_ENABLE_GENET1, 1);

	BDEV_WR_F_RB(CLKGEN_DUAL_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 1);

	/* need to control network PLLs for 7429 */
}
static void bcm7429_initialize(void)
{
	brcm_ddr_phy_initialize();
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 40nm),
	DEF_BLOCK_PM_OP(sata, 40nm),
	DEF_BLOCK_PM_OP(network, 7429),
	DEF_BLOCK_PM_OP(moca, 40nm),
	DEF_BLOCK_PM_OP(genet, 40nm),
	DEF_BLOCK_PM_OP(genet1, 40nm),
	.moca.set_cpu_rate	= bcm40nm_pm_set_moca_cpu_rate,
	.moca.set_phy_rate	= bcm40nm_pm_set_moca_phy_rate,
	.clk_get		= brcm_pm_clk_get,
	.initialize		= bcm7429_initialize,
};
#endif

#if	defined(CONFIG_BCM7552) || \
	defined(CONFIG_BCM7358) || \
	defined(CONFIG_BCM7360) || \
	defined(CONFIG_BCM7362)
static void bcm7552_pm_usb_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	bcm40nm_pm_usb_disable_s3();

	/* power down PHY and PLL */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_PWRDWNB, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_IDDQ_PWRDN, 1);

	/* power down memory */
	SRAM_OFF_2(USB, USB0);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_DISABLE, DISABLE_USB_54_MDIO_CLOCK, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 0);
}

static void bcm7552_pm_usb_enable(u32 flags)
{
	PRINT_PM_CALLBACK;
	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_DISABLE, DISABLE_USB_54_MDIO_CLOCK, 0);

	/* power up memory */
	SRAM_ON_2(USB, USB0);

	/* power up PHY and PLL */
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_IDDQ_PWRDN, 0);
	BDEV_WR_F_RB(USB_CTRL_PLL_CTL, PLL_PWRDWNB, 1);

	bcm40nm_pm_usb_enable_s3();
}

static void bcm7552_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 1);
		BDEV_SET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xB);
		/*
		 * NOTE: disabling L2INTR clock breaks ACPI pattern detection
		 */
		BDEV_UNSET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
			0x1C7);
		PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
		return;
	}

	/* Stop GENET clocks */
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 0);
	BDEV_SET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xF);

	/* Power down PLL channels */
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 2);
}

static void bcm7552_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
		BDEV_SET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0x1C7);
		BDEV_UNSET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xB);
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 0);
		return;
	}

	/* Power up PLL channels */
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 2);

	/* Restart GENET clocks */
	BDEV_UNSET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xF);
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 1);
}

static void bcm7552_pm_suspend(u32 flags)
{
	/* VCXO channel 0, 1, 2 and PLL 0 */
	PLL_CH_DIS(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 0);
	PLL_CH_DIS(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 2);
	PLL_DIS(CLKGEN_PLL_VCXO_PLL);
	/* SYS PLL 1 */
	if (!ENET_WOL(flags))
		PLL_DIS(CLKGEN_PLL_SYS1_PLL);

	/* SYS PLL 0 channels 2, 4, 5 */
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 2);
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
	PLL_CH_DIS(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 5);
}

static void bcm7552_pm_resume(u32 flags)
{
	/* SYS PLL 0 channels 2, 4, 5 */
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 2);
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 4);
	PLL_CH_ENA(CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL, 5);
	/* SYS PLL 1 */
	if (!ENET_WOL(flags))
		PLL_ENA(CLKGEN_PLL_SYS1_PLL);

	/* VCX0 channel 0, 1, 2 and PLL 0 */
	PLL_ENA(CLKGEN_PLL_VCXO_PLL);
	PLL_CH_ENA(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 0);
	PLL_CH_ENA(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_VCXO_PLL_CHANNEL_CTRL, 2);
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7552),
	DEF_BLOCK_PM_OP(genet, 7552),
	DEF_SYSTEM_PM_OP(7552),
	.clk_get		= brcm_pm_clk_get,
	.initialize		= brcm_ddr_phy_initialize,
};
#endif

#if	defined(CONFIG_BCM7584)

static void bcm7584_pm_usb_disable(u32 flags)
{
	PRINT_PM_CALLBACK;
	bcm40nm_pm_usb_disable_s3();

	/* power down USB PHY */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power down USB PLL */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	/* power down memory */
	SRAM_OFF_2(USB, USB0);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 0);
}

static void bcm7584_pm_usb_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 1);

	/* power down memory */
	SRAM_ON_2(USB, USB0);

	/* power up USB PLL */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	/* power up USB PHY */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	bcm40nm_pm_usb_enable_s3();
}

static void bcm7584_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET0_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET0_GMII_CLOCK_SELECT, 1);

		/*
		 * Do not clear GENET0_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0x83);
	} else {
		/* disable genet0 clocks */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE,
		    0xf);

		/* Every genet0 clock */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0xFF);
	}

}
static void bcm7584_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0x83);

		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET0_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET0_GMII_CLOCK_SELECT, 0);

	} else {
		/* enable genet0 clocks */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE,
		    0xf);

		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0xFF);
	}

}

static void bcm7584_pm_genet1_disable(u32 flags)
{
	PRINT_PM_CALLBACK;
	if (ENET_WOL(flags)) {
		/* switch to slow clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET1_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET1_GMII_CLOCK_SELECT, 1);

		/*
		 * Do not clear GENET1_L2INTR_CLOCK_ENABLE - it screws up
		 * receiver after resume !!!
		 */
		/* 250, EEE, UNIMAC-TX */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0x4300);
	} else {
		/* Disable all genet1 clocks */
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0x7F00);
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE,
		    0xf0);
	}
}

static void bcm7584_pm_genet1_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0x4300);

		/* switch to fast clock */
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET1_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(
		    CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_SELECT,
		    GENET1_GMII_CLOCK_SELECT, 0);

	} else {
		/* enable genet1 clocks */
		BDEV_SET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		    0x7F00);
		BDEV_UNSET(
		    BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_DISABLE,
		    0xf0);
	}
}
static void bcm7584_pm_network_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	/* SCB, 108 clocks */
	BDEV_UNSET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		0x18000);

	SRAM_OFF_3(DUAL_GENET_TOP_DUAL_RGMII, GENET0, _A);
}

static void bcm7584_pm_network_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ANY_WOL(flags))
		return;

	SRAM_ON_3(DUAL_GENET_TOP_DUAL_RGMII, GENET0, _A);

	/* SCB, 108 clocks */
	BDEV_SET(BCHP_CLKGEN_DUAL_GENET_TOP_DUAL_RGMII_CLOCK_ENABLE,
		0x18000);
}

static void bcm7584_pm_sata_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	SRAM_OFF_2(SATA3_TOP, SATA3);

	/* gate the clocks */
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_108_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_DISABLE,
		DISABLE_SATA_LV_CLK_30, 1);
}

static void bcm7584_pm_sata_enable(u32 flags)
{
	PRINT_PM_CALLBACK;
	/* reenable the clocks */
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_DISABLE,
		DISABLE_SATA_LV_CLK_30, 0);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_108_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_SATA3_TOP_CLOCK_ENABLE,
		SATA3_SCB_CLOCK_ENABLE, 1);

	SRAM_ON_2(SATA3_TOP, SATA3);

}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7584),
	DEF_BLOCK_PM_OP(genet, 7584),
	DEF_BLOCK_PM_OP(genet1, 7584),
	DEF_BLOCK_PM_OP(network, 7584),
	DEF_BLOCK_PM_OP(sata, 7584),
	.clk_get		= brcm_pm_clk_get,
};
#endif

#if	defined(CONFIG_BCM7563)

static void bcm7563_pm_usb_disable(u32 flags)
{
	PRINT_PM_CALLBACK;
	bcm40nm_pm_usb_disable_s3();

	/* power down USB PHY */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);
	/* power down USB PLL */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	/* power down memory */
	SRAM_OFF_2(USB, USB0);

	/* disable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 0);
}

static void bcm7563_pm_usb_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	/* enable the clocks */
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_USB_CLOCK_ENABLE, USB0_108_CLOCK_ENABLE, 1);

	/* power down memory */
	SRAM_ON_2(USB, USB0);

	/* power up USB PLL */
	BDEV_SET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_PWRDWNB_MASK);

	/* power up USB PHY */
	BDEV_UNSET(BCHP_USB_CTRL_PLL_CTL,
		BCHP_USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK);

	bcm40nm_pm_usb_enable_s3();
}

static void bcm7563_pm_genet_disable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 1);
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 1);
		BDEV_SET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xB);
		/*
		 * NOTE: disabling L2INTR clock breaks ACPI pattern detection
		 */
		BDEV_UNSET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
			0x1C7);
		PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
		return;
	}

	/* Stop GENET clocks */
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 0);
	BDEV_SET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xF);

	/* Power down PLL channels */
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 0);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_DIS(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 2);
}
static void bcm7563_pm_genet_enable(u32 flags)
{
	PRINT_PM_CALLBACK;

	if (ENET_WOL(flags)) {
		PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
		BDEV_SET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE, 0x1C7);
		BDEV_UNSET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xB);
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_GMII_CLOCK_SELECT, 0);
		BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_SELECT,
			GENET0_CLOCK_SELECT, 0);
		return;
	}

	/* Power up PLL channels */
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 0);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 1);
	PLL_CH_ENA(CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL, 2);

	/* Restart GENET clocks */
	BDEV_UNSET(BCHP_CLKGEN_GENET_TOP_RGMII_INST_CLOCK_DISABLE, 0xF);
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_GENET_TOP_RGMII_INST_CLOCK_ENABLE,
		GENET_108_CLOCK_ENABLE, 1);
}

static void bcm7563_pm_suspend(u32 flags)
{
	/* disable self-refresh since it interferes with suspend */
	brcm_pm_set_ddr_timeout(0);
}

static void bcm7563_pm_resume(u32 flags)
{
}

#define PM_OPS_DEFINED
static struct brcm_chip_pm_ops chip_pm_ops = {
	DEF_BLOCK_PM_OP(usb, 7563),
	DEF_BLOCK_PM_OP(genet, 7563),
	DEF_SYSTEM_PM_OP(7563),
	.clk_get		= brcm_pm_clk_get,
};
#endif

#ifndef PM_OPS_DEFINED
/* default structure - no pm callbacks available */
static struct brcm_chip_pm_ops chip_pm_ops;
#endif

static __maybe_unused void brcm_ddr_phy_initialize(void)
{
#ifdef BCHP_MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL
	/* MEMC0 */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL,
		DEVCLK_OFF_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_0_POWERDOWN,
		PLLCLKS_OFF_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_0_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_0_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
#endif
#ifdef BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL
	/* MEMC1 */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		DEVCLK_OFF_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_POWERDOWN,
		PLLCLKS_OFF_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
#endif
	/* restore self-refresh mode */
	brcm_pm_set_ddr_timeout(brcm_pm_ddr_timeout);
}

struct clk *clk_get(struct device *dev, const char *id)
{
	if (chip_pm_ops.clk_get) {
		struct clk *c = chip_pm_ops.clk_get(dev, id);
		if (c)
			return c;
	}

	return brcm_pm_clk_find(id) ? : ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL(clk_get);

static void brcm_pm_sata_disable(u32 flags)
{
	if (chip_pm_ops.sata.disable)
		chip_pm_ops.sata.disable(brcm_pm_flags | flags);
}

static void brcm_pm_sata_enable(u32 flags)
{
	if (chip_pm_ops.sata.enable)
		chip_pm_ops.sata.enable(brcm_pm_flags | flags);
}

static void brcm_pm_genet1_disable(u32 flags)
{
	if (chip_pm_ops.genet1.disable)
		chip_pm_ops.genet1.disable(brcm_pm_flags | flags);
}

static void brcm_pm_genet1_enable(u32 flags)
{
	if (chip_pm_ops.genet1.enable)
		chip_pm_ops.genet1.enable(brcm_pm_flags | flags);
}

static void brcm_pm_network_disable(u32 flags)
{
	if (chip_pm_ops.network.disable)
		chip_pm_ops.network.disable(brcm_pm_flags | flags);
}

static void brcm_pm_network_enable(u32 flags)
{
	if (chip_pm_ops.network.enable)
		chip_pm_ops.network.enable(brcm_pm_flags | flags);
}

static void brcm_pm_genet_disable(u32 flags)
{
	if (chip_pm_ops.genet.disable)
		chip_pm_ops.genet.disable(brcm_pm_flags | flags);
}

static void brcm_pm_genet_enable(u32 flags)
{
	if (chip_pm_ops.genet.enable)
		chip_pm_ops.genet.enable(brcm_pm_flags | flags);
}

static void brcm_pm_genet_disable_wol(u32 flags)
{
	brcm_pm_flags &= ~BRCM_PM_FLAG_ENET_WOL;
}

static void brcm_pm_genet_enable_wol(u32 flags)
{
	brcm_pm_flags |= BRCM_PM_FLAG_ENET_WOL;
}

static void brcm_pm_moca_disable(u32 flags)
{
	if (chip_pm_ops.moca.disable)
		chip_pm_ops.moca.disable(brcm_pm_flags | flags);
}

static void brcm_pm_moca_enable(u32 flags)
{
	if (chip_pm_ops.moca.enable)
		chip_pm_ops.moca.enable(brcm_pm_flags | flags);
}

static int brcm_pm_moca_cpu_set_rate(unsigned long rate)
{
	if (chip_pm_ops.moca.set_cpu_rate)
		return chip_pm_ops.moca.set_cpu_rate(rate);

	return -ENOENT;
}

static int brcm_pm_moca_phy_set_rate(unsigned long rate)
{
	if (chip_pm_ops.moca.set_phy_rate)
		return chip_pm_ops.moca.set_phy_rate(rate);

	return -ENOENT;
}

static void brcm_pm_moca_disable_wol(u32 flags)
{
	brcm_pm_flags &= ~BRCM_PM_FLAG_MOCA_WOL;
}

static void brcm_pm_moca_enable_wol(u32 flags)
{
	brcm_pm_flags |= BRCM_PM_FLAG_MOCA_WOL;
}

static void brcm_pm_usb_disable(u32 flags)
{
	if (chip_pm_ops.usb.disable)
		chip_pm_ops.usb.disable(brcm_pm_flags | flags);
}

static void brcm_pm_usb_enable(u32 flags)
{
	if (chip_pm_ops.usb.enable)
		chip_pm_ops.usb.enable(brcm_pm_flags | flags);
}

static void brcm_pm_initialize(void)
{
	if (chip_pm_ops.initialize)
		chip_pm_ops.initialize();
}

static void brcm_pm_set_ddr_timeout(int val)
{
#if defined(BCHP_MEMC_DDR_0_SRPD_CONFIG) && defined(CONFIG_MIPS)
	if (val) {
#if defined(BCHP_MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL)
		BDEV_WR_F(MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL,
			IDDQ_MODE_ON_SELFREF, 1);
		BDEV_WR_F(MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL,
			HIZ_ON_SELFREF, 1);
		BDEV_WR_F(MEMC_DDR23_APHY_AC_0_DDR_PAD_CNTRL,
			DEVCLK_OFF_ON_SELFREF, 1);
		BDEV_WR_F(MEMC_DDR23_APHY_AC_0_POWERDOWN,
			PLLCLKS_OFF_ON_SELFREF, 1);
		BDEV_WR_F(MEMC_DDR23_APHY_WL0_0_DDR_PAD_CNTRL,
			IDDQ_MODE_ON_SELFREF, 1);
		BDEV_WR_F(MEMC_DDR23_APHY_WL1_0_DDR_PAD_CNTRL,
			IDDQ_MODE_ON_SELFREF, 1);
#endif
		BDEV_WR_F_RB(MEMC_DDR_0_SRPD_CONFIG, INACT_COUNT, 0xdff);
		BDEV_WR_F(MEMC_DDR_0_SRPD_CONFIG, SRPD_EN, 1);
	} else {
		unsigned long flags;

		local_irq_save(flags);
		BDEV_WR_F(MEMC_DDR_0_SRPD_CONFIG, INACT_COUNT, 0xffff);
		do {
			DEV_RD(KSEG1);
		} while (BDEV_RD_F(MEMC_DDR_0_POWER_DOWN_STATUS, SRPD));
		BDEV_WR_F(MEMC_DDR_0_SRPD_CONFIG, SRPD_EN, 0);
		local_irq_restore(flags);
	}
#endif
}

#ifdef CONFIG_BRCM_HAS_STANDBY

/***********************************************************************
 * Passive standby - per-chip
 ***********************************************************************/
static void brcm_system_standby(void)
{
	if (chip_pm_ops.suspend)
		chip_pm_ops.suspend(brcm_pm_flags);
}

static void brcm_system_resume(void)
{
	if (chip_pm_ops.resume)
		chip_pm_ops.resume(brcm_pm_flags);
}

static void brcm_system_late_standby(void)
{
	if (chip_pm_ops.late_suspend)
		chip_pm_ops.late_suspend();
}

static void brcm_system_early_resume(void)
{
	if (chip_pm_ops.early_resume)
		chip_pm_ops.early_resume();
}

/***********************************************************************
 * Passive standby - common functions
 ***********************************************************************/

static suspend_state_t suspend_state;

static void brcm_pm_handshake(void)
{
#if defined(CONFIG_BRCM_PWR_HANDSHAKE_V0)
	int i;
	unsigned long base = BCHP_BSP_CMDBUF_REG_START & ~0xffff;
	unsigned long cmdbuf = BCHP_BSP_CMDBUF_REG_START;
	u32 tmp;

	i = 0;
	while (!(BDEV_RD(base + 0xb008) & 0x02)) {
		if (i++ == 10) {
			printk(KERN_WARNING "%s: CMD_IDRY2 timeout\n",
				__func__);
			break;
		}
		msleep(20);
	}
	BDEV_WR_RB(cmdbuf + 0x180, 0x00000010);
	BDEV_WR_RB(cmdbuf + 0x184, 0x00000098);
	BDEV_WR_RB(cmdbuf + 0x188, 0xabcdef00);
	BDEV_WR_RB(cmdbuf + 0x18c, 0xb055aa4f);
	BDEV_WR_RB(cmdbuf + 0x190, 0x789a0004);
	BDEV_WR_RB(cmdbuf + 0x194, 0x00000000);

	BDEV_WR_RB(base + 0xb028, 1);

	i = 0;
	while (!(BDEV_RD(base + 0xb020) & 0x01)) {
		if (i++ == 10) {
			printk(KERN_WARNING "%s: CMD_OLOAD2 timeout\n",
				__func__);
			break;
		}
		mdelay(10);
	}

	BDEV_WR_RB(base + 0xb010, 0);
	BDEV_WR_RB(base + 0xb020, 0);
	tmp = BDEV_RD(cmdbuf + 0x494);
	if (tmp != 0 && tmp != 1) {
		printk(KERN_WARNING "%s: command failed: %08lx\n",
			__func__, (unsigned long)tmp);
		mdelay(10);
		return;
	}
	BDEV_UNSET_RB(base + 0xb038, 0xff00);
	printk(KERN_DEBUG "BSP power handshake complete OK: %x\n", tmp);
#elif defined(CONFIG_BRCM_PWR_HANDSHAKE_V1)
	BDEV_WR_F_RB(AON_CTRL_HOST_MISC_CMDS, pm_restore, 0);
	BDEV_WR_F_RB(AON_CTRL_PM_INITIATE, pm_initiate_0, 0);
	BDEV_WR_F_RB(AON_CTRL_PM_INITIATE, pm_initiate_0, 1);
	mdelay(10);
#endif /* CONFIG_BRCM_PWR_HANDSHAKE_V0 */
}

static int brcm_pm_prepare(void)
{
	DBG("%s:%d\n", __func__, __LINE__);
	return 0;
}

static void brcm_pm_set_pll_on(void)
{
	int mips_pll_on = !!(brcm_pm_standby_flags & BRCM_STANDBY_MIPS_PLL_ON);
	int ddr_pll_on = !!(brcm_pm_standby_flags & BRCM_STANDBY_DDR_PLL_ON);

#if defined(BCHP_CLK_PM_PLL_ALIVE_SEL_MIPS_PLL_MASK)
	BDEV_WR_F_RB(CLK_PM_PLL_ALIVE_SEL, MIPS_PLL, mips_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_MIPS_PLL_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, MIPS_PLL, mips_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_PLL_MIPS_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, PLL_MIPS, mips_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_PLL_AVD_MIPS_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, PLL_AVD_MIPS, mips_pll_on);
#endif

#if defined(BCHP_CLK_PM_PLL_ALIVE_SEL_DDR_PLL_MASK)
	BDEV_WR_F_RB(CLK_PM_PLL_ALIVE_SEL, DDR_PLL, ddr_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_DDR_PLL_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, DDR_PLL, ddr_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_PLL_DDR0_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, PLL_DDR0, ddr_pll_on);
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, PLL_DDR1, ddr_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_PLL_DDR_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, PLL_DDR, ddr_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_memsys_PLL_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, memsys_PLL, ddr_pll_on);
#elif defined(BCHP_CLKGEN_PM_PLL_ALIVE_SEL_memsys0_PLL_MASK)
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, memsys0_PLL, ddr_pll_on);
	BDEV_WR_F_RB(CLKGEN_PM_PLL_ALIVE_SEL, memsys1_PLL, ddr_pll_on);
#endif
}

void brcm_pm_wakeup_source_enable(u32 mask, int enable)
{
#ifdef BCHP_PM_L2_CPU_MASK_SET
	if (enable) {
		BDEV_WR_RB(BCHP_PM_L2_CPU_CLEAR, mask);
		BDEV_WR_RB(BCHP_PM_L2_PCI_CLEAR, mask);
		BDEV_WR_RB(BCHP_PM_L2_CPU_MASK_CLEAR, mask);
		BDEV_WR_RB(BCHP_PM_L2_PCI_MASK_CLEAR, mask);
	} else {
		BDEV_WR_RB(BCHP_PM_L2_CPU_MASK_SET, mask);
		BDEV_WR_RB(BCHP_PM_L2_PCI_MASK_SET, mask);
	}
#else
	if (enable) {
		BDEV_WR_RB(BCHP_AON_PM_L2_CPU_CLEAR, mask);
		BDEV_WR_RB(BCHP_AON_PM_L2_PCI_CLEAR, mask);
		BDEV_WR_RB(BCHP_AON_PM_L2_CPU_MASK_CLEAR, mask);
		BDEV_WR_RB(BCHP_AON_PM_L2_PCI_MASK_CLEAR, mask);
	} else {
		BDEV_WR_RB(BCHP_AON_PM_L2_CPU_MASK_SET, mask);
		BDEV_WR_RB(BCHP_AON_PM_L2_PCI_MASK_SET, mask);
	}
#endif
}
EXPORT_SYMBOL(brcm_pm_wakeup_source_enable);

int brcm_pm_wakeup_get_status(u32 mask)
{
#ifdef BCHP_PM_L2_CPU_STATUS
	return !!(BDEV_RD(BCHP_PM_L2_CPU_STATUS) & mask);
#else
	return !!(BDEV_RD(BCHP_AON_PM_L2_CPU_STATUS) &
		   ~BDEV_RD(BCHP_AON_PM_L2_CPU_MASK_STATUS)
		   & mask);
#endif
}
EXPORT_SYMBOL(brcm_pm_wakeup_get_status);

static u32 brcm_pm_wakeup_get_mask(void)
{
#ifdef BCHP_PM_L2_CPU_STATUS
	return BDEV_RD(BCHP_PM_L2_CPU_MASK_STATUS);
#else
	return BDEV_RD(BCHP_AON_PM_L2_CPU_MASK_STATUS);
#endif
}

static int brcm_pm_timer_wakeup_enable(void *ref)
{
#if !WATCHDOG_TIMER_WAKEUP_ALWAYS
	if (brcm_pm_standby_flags & BRCM_STANDBY_TEST)
#endif
		brcm_pm_wakeup_source_enable(TIMER_INTR_MASK, 1);
	return 0;
}

static int brcm_pm_timer_wakeup_disable(void *ref)
{
	brcm_pm_wakeup_source_enable(TIMER_INTR_MASK, 0);
	return 0;
}

static int brcm_pm_timer_wakeup_poll(void *ref)
{
	int retval = brcm_pm_wakeup_get_status(TIMER_INTR_MASK);
	printk(KERN_DEBUG "%s:  %d\n", __func__, retval);
	return retval;
}

static struct brcm_wakeup_ops brcm_timer_wakeup_ops = {
	.enable = brcm_pm_timer_wakeup_enable,
	.disable = brcm_pm_timer_wakeup_disable,
	.poll = brcm_pm_timer_wakeup_poll,
};

static void brcm_pm_set_alarm(int timeout)
{
	u32 tmp;
	BDEV_WR_RB(BCHP_WKTMR_EVENT, 1);
	if (timeout == 1) {
		/* Wait for next second to start - if too little time left
		 * before the counter increment we may receive WKTMR interrupt
		 * before system is fully suspended.
		 * One second should be enough to complete suspend
		 * from this point
		 */
		tmp = BDEV_RD(BCHP_WKTMR_COUNTER);
		while (BDEV_RD(BCHP_WKTMR_COUNTER) == tmp)
			;
	}
	BDEV_WR_RB(BCHP_WKTMR_ALARM, BDEV_RD(BCHP_WKTMR_COUNTER) + timeout);
}

static void brcm_pm_clear_alarm(void)
{
	BDEV_WR_RB(BCHP_WKTMR_EVENT, 1);
}

#if defined(CONFIG_BCM7468) || defined(CONFIG_BCM7550)
#define NON_RELOCATABLE_VEC
#endif
static int brcm_pm_standby(int mode)
{
	int ret = 0, valid_event = 1;
	u32 l2_mask;
	unsigned long restart_vec = BRCM_WARM_RESTART_VEC;
	unsigned long restart_vec_size = bmips_smp_int_vec_end -
		bmips_smp_int_vec;

	DBG("%s:%d\n", __func__, __LINE__);

	if (brcm_pm_standby_flags & BRCM_STANDBY_TEST)
		printk(KERN_INFO "%s: timeout %ld\n",
			__func__, brcm_pm_standby_timeout);

	do {

		brcm_irq_standby_enter(BRCM_IRQ_STANDBY);

#if defined(NON_RELOCATABLE_VEC)
	{
	u32 oldvec[5];
	const int vecsize = 0x14;
	void *vec = (void *)ebase + 0x200;

	restart_vec = (unsigned long)vec;

	memcpy(oldvec, vec, vecsize);
	memcpy(vec, bmips_smp_int_vec, vecsize);
	flush_icache_range(restart_vec, restart_vec + vecsize);
#else
	/* send all IRQs to BRCM_WARM_RESTART_VEC */
	clear_c0_cause(CAUSEF_IV);
	irq_disable_hazard();
	set_c0_status(ST0_BEV);
	irq_disable_hazard();
#endif

	brcm_system_standby();
	/*
	 * Save current wakeup mask -
	 * it may have been changed by usermode or drivers without
	 * brcm_pm_wakeup API
	 */
	l2_mask = brcm_pm_wakeup_get_mask();
	brcm_pm_wakeup_enable();

#ifdef DEBUG_M2M_DMA
	if (brcm_pm_standby_flags & 0x40) {
		unsigned char *tb = kzalloc(PAGE_SIZE*16, GFP_ATOMIC);
		int result, ii;
		/* Test 1: simple copy */
		if (1) {
			struct brcm_mem_transfer xfer = {
			.src		= tb,
			.dst		= tb+PAGE_SIZE,
			.pa_src		= 0,
			.pa_dst		= 0,
			.len		= PAGE_SIZE,
			.mode		= BRCM_MEM_DMA_SCRAM_NONE,
			.key		= 0,
			.next		= NULL
		};
		memset(tb, 0xa3, PAGE_SIZE);
		memset(tb+PAGE_SIZE, 0, PAGE_SIZE);

		brcm_mem_dma_simple_transfer(&xfer);
		result = memcmp(tb, tb+PAGE_SIZE, PAGE_SIZE);
		DBG("MEM DMA TEST 1: result %d\n", result);
		}
		/* Test 2: encryption/decryption */
		if (1) {
			struct brcm_mem_transfer xfer[] = {
			[0] = {
				.src		= tb,
				.dst		= tb+PAGE_SIZE*2,
				.pa_src		= 0,
				.pa_dst		= 0,
				.len		= PAGE_SIZE,
				.mode		= BRCM_MEM_DMA_SCRAM_BLOCK,
				.key		= 5,
				.next		= &xfer[1],
			},
			[1] = {
				.src		= tb+PAGE_SIZE,
				.dst		= tb+PAGE_SIZE*3,
				.pa_src		= 0,
				.pa_dst		= 0,
				.len		= PAGE_SIZE,
				.mode		= BRCM_MEM_DMA_SCRAM_BLOCK,
				.key		= 5,
				.next		= NULL,
			},
			[2] = {
				.src		= tb+PAGE_SIZE*2,
				.dst		= tb+PAGE_SIZE*4,
				.pa_src		= 0,
				.pa_dst		= 0,
				.len		= PAGE_SIZE,
				.mode		= BRCM_MEM_DMA_SCRAM_BLOCK,
				.key		= 6,
				.next		= &xfer[3],
			},
			[3] = {
				.src		= tb+PAGE_SIZE*3,
				.dst		= tb+PAGE_SIZE*5,
				.pa_src		= 0,
				.pa_dst		= 0,
				.len		= PAGE_SIZE,
				.mode		= BRCM_MEM_DMA_SCRAM_BLOCK,
				.key		= 6,
				.next		= NULL,
			},
		};
		memset(tb, 0xa3, PAGE_SIZE*2);
		memset(tb+PAGE_SIZE*2, 0x45, PAGE_SIZE*4);

		DBG("MEM DMA TEST 2:\ninput\n");
		for (ii = 0; ii < PAGE_SIZE; ii++) {
			DBG("%02x ", *(tb+ii));
			if (ii%32 == 31) {
				DBG("\n"); break;
			}
		}
		brcm_mem_dma_transfer(&xfer[0]);
		DBG("encrypted\n");
		for (ii = 0; ii < PAGE_SIZE; ii++) {
			DBG("%02x ", *(tb+PAGE_SIZE*2+ii));
			if (ii%32 == 31) {
				DBG("\n"); break;
			}
		}
		brcm_mem_dma_transfer(&xfer[2]);
		DBG("decrypted\n");
		for (ii = 0; ii < PAGE_SIZE; ii++) {
			DBG("%02x ", *(tb+PAGE_SIZE*4+ii));
			if (ii%32 == 31) {
				DBG("\n"); break;
			}
		}
		result = memcmp(tb, tb+(PAGE_SIZE*4), PAGE_SIZE*2);
		DBG("result %02x\n", (unsigned char)result);
		}
		kfree(tb);
	} else
#endif
	if (brcm_pm_standby_flags & BRCM_STANDBY_NO_SLEEP) {
		if (brcm_pm_standby_flags & BRCM_STANDBY_DELAY)
			mdelay(120000);
		else
			mdelay(5000);
	} else {
		if (brcm_pm_standby_flags & BRCM_STANDBY_TEST)
			brcm_pm_set_alarm(brcm_pm_standby_timeout ? : 1);
#if WATCHDOG_TIMER_WAKEUP_ALWAYS
		else
			brcm_pm_set_alarm(20);
#endif
		brcm_pm_set_pll_on();
		brcm_pm_handshake();
		brcm_system_late_standby();
#ifdef CONFIG_BRCM_HAS_AON
		if (mode)
			ret = brcm_pm_s3_standby(
				current_cpu_data.dcache.linesz,
				brcm_pm_standby_flags);
		else
#endif
			ret = brcm_pm_standby_asm(
				current_cpu_data.icache.linesz,
				restart_vec, restart_vec_size,
				brcm_pm_standby_flags);
		brcm_system_early_resume();
		brcm_pm_clear_alarm();
		valid_event = brcm_pm_wakeup_poll(l2_mask);
	}
	brcm_pm_wakeup_disable();
	brcm_system_resume();

#if defined(NON_RELOCATABLE_VEC)
	memcpy(vec, oldvec, vecsize);
	flush_icache_range(restart_vec, restart_vec + vecsize);
	}
#else
	/* send IRQs back to the normal runtime vectors */
	clear_c0_status(ST0_BEV);
	irq_disable_hazard();
	set_c0_cause(CAUSEF_IV);
	irq_disable_hazard();
#endif
	brcm_irq_standby_exit();
	} while (!ret && !valid_event);

	if (ret && !mode)
		printk(KERN_WARNING "%s: standby failed with code %d\n",
			__func__, ret);

#ifdef CONFIG_BRCM_HAS_AON
	brcm_pm_time_at_wakeup[0] = BDEV_RD(AON_RAM(0));
	brcm_pm_time_at_wakeup[1] = BDEV_RD(AON_RAM(1));
#endif
	return 0;
}

#ifdef CONFIG_BRCM_HAS_AON
#if defined(BCHP_AON_CTRL_PM_CTRL_pm_clk_divider_reset_en_MASK) || \
defined(BCHP_SUN_TOP_CTRL_PM_CTRL_pm_clk_divider_reset_en_MASK)
#define PM_CMD_BASE		0x1A
#else
#define PM_CMD_BASE		0x12
#endif

#if defined(CONFIG_CPU_BMIPS5000)
#define PM_USE_MIPS_READY	0x04
#else
#define PM_USE_MIPS_READY	0x00
#endif

#define PM_STANDBY_CONFIG	(PM_CMD_BASE|PM_USE_MIPS_READY)
#define PM_STANDBY_COMMAND	(PM_STANDBY_CONFIG|1)

void brcm_pm_s3_cold_boot(void)
{
	if (!brcm_pm_halt_mode)
		/* regular halt, go back */
		return;

	brcm_irq_standby_enter(BRCM_IRQ_STANDBY);
	brcm_pm_wakeup_enable();
	if (brcm_pm_standby_flags & BRCM_STANDBY_TEST)
		brcm_pm_set_alarm(brcm_pm_standby_timeout ? : 3);
	brcm_pm_handshake();
	BDEV_WR_RB(AON_RAM(0), 0);
	BDEV_WR_RB(BCHP_AON_CTRL_PM_MIPS_WAIT_COUNT, 0xffff);

	/* PD request is initiated on pm_start_pwrdn transition 0->1 */
	BDEV_WR_RB(BCHP_AON_CTRL_PM_CTRL, 0);
	BDEV_WR_RB(BCHP_AON_CTRL_PM_CTRL, PM_STANDBY_CONFIG);
	/* Separate PD request from the rest of PMSM setup */
	BDEV_WR_RB(BCHP_AON_CTRL_PM_CTRL, PM_STANDBY_COMMAND);

	__asm__ __volatile__(
	"	wait\n"
	: : : "memory");
}
#endif

static int brcm_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	DBG("%s:%d\n", __func__, __LINE__);
	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
		ret = brcm_pm_standby(0);
		break;
#ifdef CONFIG_BRCM_HAS_AON
	case PM_SUSPEND_MEM:
		ret = brcm_pm_standby(1);
		break;
#endif
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void brcm_pm_finish(void)
{
	DBG("%s:%d\n", __func__, __LINE__);
#ifdef CONFIG_BRCM_HAS_AON
	BDEV_WR_RB(AON_RAM(0), 0);
#endif
}

static int brcm_pm_begin(suspend_state_t state)
{
	DBG("%s:%d\n", __func__, __LINE__);
	suspend_state = state;
	if (state == PM_SUSPEND_MEM)
		brcm_pm_flags |= BRCM_PM_FLAG_S3;
	else
		brcm_pm_flags &= ~BRCM_PM_FLAG_S3;
	return 0;
}

static void brcm_pm_end(void)
{
	DBG("%s:%d\n", __func__, __LINE__);
	suspend_state = PM_SUSPEND_ON;
	return;
}

static int brcm_pm_valid(suspend_state_t state)
{
#ifdef CONFIG_BRCM_HAS_AON
	return (state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM);
#else
	return state == PM_SUSPEND_STANDBY;
#endif
}

static const struct platform_suspend_ops brcm_pm_ops = {
	.begin		= brcm_pm_begin,
	.end		= brcm_pm_end,
	.prepare	= brcm_pm_prepare,
	.enter		= brcm_pm_enter,
	.finish		= brcm_pm_finish,
	.valid		= brcm_pm_valid,
};

static int brcm_suspend_init(void)
{
	DBG("%s:%d\n", __func__, __LINE__);
	suspend_set_ops(&brcm_pm_ops);
	brcm_pm_wakeup_register(&brcm_timer_wakeup_ops, NULL, "WKTMR");
	return 0;
}
late_initcall(brcm_suspend_init);

#endif /* CONFIG_BRCM_HAS_STANDBY */

int brcm_pm_deep_sleep(void)
{
#ifdef CONFIG_BRCM_HAS_STANDBY
	return suspend_state == PM_SUSPEND_MEM;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(brcm_pm_deep_sleep);

void brcm_pm_sata3(int enable)
{
	struct clk *clk = brcm_pm_clk_find("sata");
	if (clk)
		enable ? clk_enable(clk) : clk_disable(clk);
}

void brcm_pm_save_restore_rts(unsigned long reg_addr, u32 *data, int restore)
{
	int ii = 0;

	reg_addr += 4;		/* skip debug register */
	if (restore)
		for (ii = 0; ii < NUM_MEMC_CLIENTS; ii++) {
			BDEV_WR_RB(reg_addr, data[ii]);
			reg_addr += 4;
		}
	else
		/* Save MEMC1 configuration */
		for (ii = 0; ii < NUM_MEMC_CLIENTS; ii++) {
			data[ii] = BDEV_RD(reg_addr);
			reg_addr += 4;
		}
}
