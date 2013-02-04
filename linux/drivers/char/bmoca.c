/*
 * Copyright (C) 2010 Broadcom Corporation
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

#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/scatterlist.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/printk.h>

#define DRV_VERSION		0x00040000
#define DRV_BUILD_NUMBER	0x20110831

#if defined(CONFIG_BRCMSTB)
#define MOCA6816		0
#include <linux/bmoca.h>
#elif defined(DSL_MOCA)
#define MOCA6816		1
#include "bmoca.h"
#include <boardparms.h>
#include <bcm3450.h>
#include <linux/netdevice.h>
#else
#define MOCA6816		1
#include <linux/bmoca.h>
#endif

#if defined(CONFIG_BRCMSTB)
#include <linux/brcmstb/brcmstb.h>
#endif

#define MOCA_ENABLE		1
#define MOCA_DISABLE		0

#define OFF_PKT_REINIT_MEM	0x00a08000
#define PKT_REINIT_MEM_SIZE	(32 * 1024)
#define PKT_REINIT_MEM_END	(OFF_PKT_REINIT_MEM  + PKT_REINIT_MEM_SIZE)

/* The mailbox layout is different for MoCA 2.0 compared to
   MoCA 1.1 */

/* MoCA 1.1 mailbox layout */
#define HOST_REQ_SIZE_11        304
#define HOST_RESP_SIZE_11       256
#define CORE_REQ_SIZE_11        400
#define CORE_RESP_SIZE_11       64

/* MoCA 1.1 offsets from the mailbox pointer */
#define HOST_REQ_OFFSET_11      0
#define HOST_RESP_OFFSET_11     (HOST_REQ_OFFSET_11 + HOST_REQ_SIZE_11)
#define CORE_REQ_OFFSET_11      (HOST_RESP_OFFSET_11 + HOST_RESP_SIZE_11)
#define CORE_RESP_OFFSET_11     (CORE_REQ_OFFSET_11 + CORE_REQ_SIZE_11)

/* MoCA 2.0 mailbox layout */
#define HOST_REQ_SIZE_20        512
#define HOST_RESP_SIZE_20       512
#define CORE_REQ_SIZE_20        512
#define CORE_RESP_SIZE_20       512

/* MoCA 2.0 offsets from the mailbox pointer */
#define HOST_REQ_OFFSET_20      0
#define HOST_RESP_OFFSET_20     (HOST_REQ_OFFSET_20 + 0)
#define CORE_REQ_OFFSET_20      (HOST_RESP_OFFSET_20 + HOST_RESP_SIZE_20)
#define CORE_RESP_OFFSET_20     (CORE_REQ_OFFSET_20 + 0)

#define HOST_REQ_SIZE_MAX       HOST_REQ_SIZE_20
#define CORE_REQ_SIZE_MAX       CORE_REQ_SIZE_20
#define CORE_RESP_SIZE_MAX      CORE_RESP_SIZE_20

/* local H2M, M2H buffers */
#define NUM_CORE_MSG		32
#define NUM_HOST_MSG		8

#define FW_CHUNK_SIZE		4096
#define MAX_BL_CHUNKS		8
#define MAX_FW_SIZE		(1024 * 1024)
#define MAX_FW_PAGES		((MAX_FW_SIZE >> PAGE_SHIFT) + 1)
#define MAX_LAB_PRINTF		104

#ifdef CONFIG_CPU_LITTLE_ENDIAN
#define M2M_WRITE		(BIT(31) | BIT(27) | BIT(28))
#define M2M_READ		(BIT(30) | BIT(27) | BIT(28))
#else
#define M2M_WRITE		(BIT(31) | BIT(27))
#define M2M_READ		(BIT(30) | BIT(27))
#endif

#define M2M_TIMEOUT_MS		10

#define NO_FLUSH_IRQ		0
#define FLUSH_IRQ		1
#define FLUSH_DMA_ONLY		2
#define FLUSH_REQRESP_ONLY	3

#define DEFAULT_PHY_CLOCK	(300 * 1000000)

/* DMA buffers may not share a cache line with anything else */
#define __DMA_ALIGN__		__attribute__((__aligned__(L1_CACHE_BYTES)))

struct moca_host_msg {
	u32			data[HOST_REQ_SIZE_MAX / 4] __DMA_ALIGN__;
	struct list_head	chain __DMA_ALIGN__;
	u32			len;
};

struct moca_core_msg {
	u32			data[CORE_REQ_SIZE_MAX / 4] __DMA_ALIGN__;
	struct list_head	chain __DMA_ALIGN__;
	u32			len;
};

struct moca_regs {
	unsigned int		data_mem_offset;
	unsigned int		data_mem_size;
	unsigned int		cntl_mem_size;
	unsigned int		cntl_mem_offset;
	unsigned int		gp0_offset;
	unsigned int		gp1_offset;
	unsigned int		ringbell_offset;
	unsigned int		l2_status_offset;
	unsigned int		l2_clear_offset;
	unsigned int		l2_mask_set_offset;
	unsigned int		l2_mask_clear_offset;
	unsigned int		sw_reset_offset;
	unsigned int		led_ctrl_offset;
	unsigned int		m2m_src_offset;
	unsigned int		m2m_dst_offset;
	unsigned int		m2m_cmd_offset;
	unsigned int		m2m_status_offset;
	unsigned int		moca2host_mmp_inbox_0_offset;
	unsigned int		moca2host_mmp_inbox_1_offset;
	unsigned int		moca2host_mmp_inbox_2_offset;
	unsigned int		h2m_resp_bit[2]; /* indexed by cpu */
	unsigned int		h2m_req_bit[2]; /* indexed by cpu */
	unsigned int		sideband_gmii_fc_offset;
};

struct moca_priv_data {
	struct platform_device	*pdev;
	struct device		*dev;

	unsigned int		minor;
	int			irq;
	struct work_struct	work;
	void __iomem		*base;
	void __iomem		*i2c_base;

	unsigned int		mbx_offset[2]; /* indexed by MoCA cpu */
	struct page		*fw_pages[MAX_FW_PAGES];
	struct scatterlist	fw_sg[MAX_FW_PAGES];
	struct completion	copy_complete;
	struct completion	chunk_complete;

	struct list_head	host_msg_free_list;
	struct list_head	host_msg_pend_list;
	struct moca_host_msg	host_msg_queue[NUM_HOST_MSG] __DMA_ALIGN__;
	wait_queue_head_t	host_msg_wq;

	struct list_head	core_msg_free_list;
	struct list_head	core_msg_pend_list;
	u32		core_resp_buf[CORE_RESP_SIZE_MAX / 4] __DMA_ALIGN__;
	struct moca_core_msg	core_msg_queue[NUM_CORE_MSG] __DMA_ALIGN__;
	struct moca_core_msg	core_msg_temp __DMA_ALIGN__;
	wait_queue_head_t	core_msg_wq;

	spinlock_t		list_lock;
	spinlock_t		clock_lock;
	spinlock_t		irq_status_lock;
	struct mutex		dev_mutex;
	struct mutex		copy_mutex;
	struct mutex		moca_i2c_mutex;
	int			host_mbx_busy;
	int			host_resp_pending;
	int			core_req_pending;
	int			assert_pending;
	int			wdt_pending;

	int			enabled;
	int			running;
	int			wol_enabled;
	struct clk		*clk;
	struct clk		*phy_clk;
	struct clk		*cpu_clk;

	int			refcount;
	unsigned long		start_time;
	dma_addr_t		tpcapBufPhys;

	unsigned int		bonded_mode;
	unsigned int		phy_freq;

	unsigned int		hw_rev;

	const struct moca_regs	*regs;

	/* MMP Parameters */
	unsigned int		mmp_20;
	unsigned int		host_req_size;
	unsigned int		host_resp_size;
	unsigned int		core_req_size;
	unsigned int		core_resp_size;
	unsigned int		host_req_offset;
	unsigned int		host_resp_offset;
	unsigned int		core_req_offset;
	unsigned int		core_resp_offset;
};

static const struct moca_regs regs_11_plus = {
	.data_mem_offset		= 0,
	.data_mem_size			= (256 * 1024),
	.cntl_mem_offset		= 0x00040000,
	.cntl_mem_size			= (128 * 1024),
	.gp0_offset			= 0x000a2050,
	.gp1_offset			= 0x000a2054,
	.ringbell_offset		= 0x000a2060,
	.l2_status_offset		= 0x000a2080,
	.l2_clear_offset		= 0x000a2088,
	.l2_mask_set_offset		= 0x000a2090,
	.l2_mask_clear_offset		= 0x000a2094,
	.sw_reset_offset		= 0x000a2040,
	.led_ctrl_offset		= 0x000a204c,
	.led_ctrl_offset		= 0x000a204c,
	.m2m_src_offset			= 0x000a2000,
	.m2m_dst_offset			= 0x000a2004,
	.m2m_cmd_offset			= 0x000a2008,
	.m2m_status_offset		= 0x000a200c,
	.h2m_resp_bit[1]		= 0x1,
	.h2m_req_bit[1]			= 0x2,
	.sideband_gmii_fc_offset	= 0x000a1420
};

static const struct moca_regs regs_11_lite = {
	.data_mem_offset		= 0,
	.data_mem_size			= (96 * 1024),
	.cntl_mem_offset		= 0x0004c000,
	.cntl_mem_size			= (80 * 1024),
	.gp0_offset			= 0x000a2050,
	.gp1_offset			= 0x000a2054,
	.ringbell_offset		= 0x000a2060,
	.l2_status_offset		= 0x000a2080,
	.l2_clear_offset		= 0x000a2088,
	.l2_mask_set_offset		= 0x000a2090,
	.l2_mask_clear_offset		= 0x000a2094,
	.sw_reset_offset		= 0x000a2040,
	.led_ctrl_offset		= 0x000a204c,
	.led_ctrl_offset		= 0x000a204c,
	.m2m_src_offset			= 0x000a2000,
	.m2m_dst_offset			= 0x000a2004,
	.m2m_cmd_offset			= 0x000a2008,
	.m2m_status_offset		= 0x000a200c,
	.h2m_resp_bit[1]		= 0x1,
	.h2m_req_bit[1]			= 0x2,
	.sideband_gmii_fc_offset	= 0x000a1420
};

static const struct moca_regs regs_11 = {
	.data_mem_offset		= 0,
	.data_mem_size			= (256 * 1024),
	.cntl_mem_offset		= 0x0004c000,
	.cntl_mem_size			= (80 * 1024),
	.gp0_offset			= 0x000a2050,
	.gp1_offset			= 0x000a2054,
	.ringbell_offset		= 0x000a2060,
	.l2_status_offset		= 0x000a2080,
	.l2_clear_offset		= 0x000a2088,
	.l2_mask_set_offset		= 0x000a2090,
	.l2_mask_clear_offset		= 0x000a2094,
	.sw_reset_offset		= 0x000a2040,
	.led_ctrl_offset		= 0x000a204c,
	.m2m_src_offset			= 0x000a2000,
	.m2m_dst_offset			= 0x000a2004,
	.m2m_cmd_offset			= 0x000a2008,
	.m2m_status_offset		= 0x000a200c,
	.h2m_resp_bit[1]		= 0x1,
	.h2m_req_bit[1]			= 0x2,
	.sideband_gmii_fc_offset	= 0x000a1420
};

static const struct moca_regs regs_20 = {
	.data_mem_offset		= 0,
	.data_mem_size			= (288 * 1024),
	.cntl_mem_offset		= 0x00120000,
	.cntl_mem_size			= (384 * 1024),
	.gp0_offset			= 0,
	.gp1_offset			= 0,
	.ringbell_offset		= 0x001ffd0c,
	.l2_status_offset		= 0x001ffc40,
	.l2_clear_offset		= 0x001ffc48,
	.l2_mask_set_offset		= 0x001ffc50,
	.l2_mask_clear_offset		= 0x001ffc54,
	.sw_reset_offset		= 0x001ffd00,
	.led_ctrl_offset		= 0,
	.m2m_src_offset			= 0x001ffc00,
	.m2m_dst_offset			= 0x001ffc04,
	.m2m_cmd_offset			= 0x001ffc08,
	.m2m_status_offset		= 0x001ffc0c,
	.moca2host_mmp_inbox_0_offset	= 0x001ffd58,
	.moca2host_mmp_inbox_1_offset	= 0x001ffd5c,
	.moca2host_mmp_inbox_2_offset	= 0x001ffd60,
	.h2m_resp_bit[1]		= 0x10,
	.h2m_req_bit[1]			= 0x20,
	.h2m_resp_bit[0]		= 0x1,
	.h2m_req_bit[0]			= 0x2,
	.sideband_gmii_fc_offset	= 0x001fec18
};

#define MOCA_FW_MAGIC		0x4d6f4341

struct moca_fw_hdr {
	uint32_t		jump[2];
	uint32_t		length;
	uint32_t		cpuid;
	uint32_t		magic;
	uint32_t		hw_rev;
	uint32_t		bl_chunks;
	uint32_t		res1;
};

struct bsc_regs {
	u32			chip_address;
	u32			data_in[8];
	u32			cnt_reg;
	u32			ctl_reg;
	u32			iic_enable;
	u32			data_out[8];
	u32			ctlhi_reg;
	u32			scl_param;
};

/* support for multiple MoCA devices */
#define NUM_MINORS		8
static struct moca_priv_data *minor_tbl[NUM_MINORS];
static struct class *moca_class;

/* character major device number */
#define MOCA_MAJOR		234
#define MOCA_CLASS		"bmoca"

#define M2H_RESP		BIT(0)
#define M2H_REQ			BIT(1)
#define M2H_ASSERT		BIT(2)
#define M2H_NEXTCHUNK		BIT(3)
#define M2H_NEXTCHUNK_CPU0	BIT(4)
#define M2H_WDT_CPU0		BIT(6)
#define M2H_WDT_CPU1		BIT(10)
#define M2H_DMA			BIT(11)

#define M2H_RESP_CPU0		BIT(13)
#define M2H_REQ_CPU0		BIT(14)
#define M2H_ASSERT_CPU0		BIT(15)

/* does this word contain a NIL byte (i.e. end of string)? */
#define HAS0(x)			((((x) & 0xff) == 0) || \
				 (((x) & 0xff00) == 0) || \
				 (((x) & 0xff0000) == 0) || \
				 (((x) & 0xff000000) == 0))

#define MOCA_SET(x, y)		do { \
	MOCA_WR(x, MOCA_RD(x) | (y)); \
} while (0)
#define MOCA_UNSET(x, y)	do { \
	MOCA_WR(x, MOCA_RD(x) & ~(y)); \
} while (0)

static void moca_3450_write_i2c(struct moca_priv_data *priv, u8 addr, u32 data);
static u32 moca_3450_read_i2c(struct moca_priv_data *priv, u8 addr);
static int moca_get_mbx_offset(struct moca_priv_data *priv);

#define INRANGE(x, a, b)	(((x) >= (a)) && ((x) < (b)))

static inline int moca_range_ok(struct moca_priv_data *priv,
	unsigned long offset, unsigned long len)
{
	const struct moca_regs *r = priv->regs;
	unsigned long lastad = offset + len - 1;

	if (lastad < offset)
		return -EINVAL;

	if (INRANGE(offset, r->cntl_mem_offset,
		    r->cntl_mem_offset + r->cntl_mem_size) &&
	    INRANGE(lastad, r->cntl_mem_offset,
		    r->cntl_mem_offset + r->cntl_mem_size))
		return 0;

	if (INRANGE(offset, r->data_mem_offset,
		    r->data_mem_offset + r->data_mem_size) &&
	    INRANGE(lastad, r->data_mem_offset,
		    r->data_mem_offset + r->data_mem_size))
		return 0;

	if (INRANGE(offset, OFF_PKT_REINIT_MEM, PKT_REINIT_MEM_END) &&
	    INRANGE(lastad, OFF_PKT_REINIT_MEM, PKT_REINIT_MEM_END))
		return 0;

	return -EINVAL;
}

static void moca_mmp_init(struct moca_priv_data *priv, int is20)
{
	if (is20) {
		priv->host_req_size    = HOST_REQ_SIZE_20;
		priv->host_resp_size   = HOST_RESP_SIZE_20;
		priv->core_req_size    = CORE_REQ_SIZE_20;
		priv->core_resp_size   = CORE_RESP_SIZE_20;
		priv->host_req_offset  = HOST_REQ_OFFSET_20;
		priv->host_resp_offset = HOST_RESP_OFFSET_20;
		priv->core_req_offset  = CORE_REQ_OFFSET_20;
		priv->core_resp_offset = CORE_RESP_OFFSET_20;
		priv->mmp_20 = 1;
	} else {
		priv->host_req_size    = HOST_REQ_SIZE_11;
		priv->host_resp_size   = HOST_RESP_SIZE_11;
		priv->core_req_size    = CORE_REQ_SIZE_11;
		priv->core_resp_size   = CORE_RESP_SIZE_11;
		priv->host_req_offset  = HOST_REQ_OFFSET_11;
		priv->host_resp_offset = HOST_RESP_OFFSET_11;
		priv->core_req_offset  = CORE_REQ_OFFSET_11;
		priv->core_resp_offset = CORE_RESP_OFFSET_11;
		priv->mmp_20 = 0;
	}
}

static int moca_is_20(struct moca_priv_data *priv)
{
	return (priv->hw_rev & MOCA_PROTVER_MASK) == MOCA_PROTVER_20;
}

#ifdef CONFIG_BRCM_MOCA_BUILTIN_FW
#error Not supported in this version
#else
static const char *bmoca_fw_image;
#endif

/*
 * LOW-LEVEL DEVICE OPERATIONS
 */

#define MOCA_RD(x)		__raw_readl((void __iomem *)(x))
#define MOCA_WR(x, y)		__raw_writel((y), (void __iomem *)(x))

#define I2C_RD(x)		MOCA_RD(x)
#define I2C_WR(x, y)		MOCA_WR(x, y)

static void moca_hw_reset(struct moca_priv_data *priv)
{
	const struct moca_regs *r = priv->regs;

	/* disable and clear all interrupts */
	MOCA_WR(priv->base + r->l2_mask_set_offset, 0xffffffff);
	MOCA_RD(priv->base + r->l2_mask_set_offset);

	/* assert resets */

	/* reset CPU first, both CPUs for MoCA 20 HW */
	if (moca_is_20(priv))
		MOCA_SET(priv->base + r->sw_reset_offset, 5);
	else
		MOCA_SET(priv->base + r->sw_reset_offset, 1);

	MOCA_RD(priv->base + r->sw_reset_offset);

	udelay(20);

	/* reset everything else except clocks */
	MOCA_SET(priv->base + r->sw_reset_offset, ~(BIT(3) | BIT(7)));
	MOCA_RD(priv->base + r->sw_reset_offset);

	udelay(20);

	/* disable clocks */
	MOCA_SET(priv->base + r->sw_reset_offset, ~BIT(3));
	MOCA_RD(priv->base + r->sw_reset_offset);

	MOCA_WR(priv->base + r->l2_clear_offset, 0xffffffff);
	MOCA_RD(priv->base + r->l2_clear_offset);
}

/* called any time we start/restart/stop MoCA */
static void moca_hw_init(struct moca_priv_data *priv, int action)
{
	const struct moca_regs *r = priv->regs;

	if (action == MOCA_ENABLE && !priv->enabled) {
		clk_enable(priv->clk);
		clk_enable(priv->phy_clk);
		clk_enable(priv->cpu_clk);
		priv->enabled = 1;
	}

	/* clock not enabled, register accesses will fail with bus error */
	if (!priv->enabled)
		return;

	moca_hw_reset(priv);
	udelay(1);

	if (action == MOCA_ENABLE) {
		/* deassert moca_sys_reset and clock */
		MOCA_UNSET(priv->base + r->sw_reset_offset, BIT(1) | BIT(7));

		if (priv->hw_rev >= HWREV_MOCA_20_GEN22) {
			/* Take PHY0 out of reset and enable clock */
			MOCA_UNSET(priv->base + r->sw_reset_offset,
				   BIT(4) | BIT(8));

			if (priv->bonded_mode) {
				/* Take PHY1 out of reset and enable clock */
				MOCA_UNSET(priv->base + r->sw_reset_offset,
					   BIT(5) | BIT(9));
			}
		}
		MOCA_RD(priv->base + r->sw_reset_offset);
	}

	if (!moca_is_20(priv)) {
		/* clear junk out of GP0/GP1 */
		MOCA_WR(priv->base + r->gp0_offset, 0xffffffff);
		MOCA_WR(priv->base + r->gp1_offset, 0x0);
		/* set up activity LED for 50% duty cycle */
		MOCA_WR(priv->base + r->led_ctrl_offset, 0x40004000);
	}

	/* enable DMA completion interrupts */
	MOCA_WR(priv->base + r->ringbell_offset, 0);
	MOCA_WR(priv->base + r->l2_mask_clear_offset, M2H_DMA);
	MOCA_RD(priv->base + r->l2_mask_clear_offset);

	if (action == MOCA_DISABLE && priv->enabled) {
		priv->enabled = 0;
		clk_disable(priv->cpu_clk);
		clk_disable(priv->phy_clk);
		clk_disable(priv->clk);
	}
}

static void moca_ringbell(struct moca_priv_data *priv, u32 mask)
{
	const struct moca_regs *r = priv->regs;

	MOCA_WR(priv->base + r->ringbell_offset, mask);
	MOCA_RD(priv->base + r->ringbell_offset);
}

static u32 moca_irq_status(struct moca_priv_data *priv, int flush)
{
	const struct moca_regs *r = priv->regs;
	u32 stat, dma_mask = M2H_DMA | M2H_NEXTCHUNK;
	unsigned long flags;

	if (moca_is_20(priv))
		dma_mask |= M2H_NEXTCHUNK_CPU0;

	spin_lock_irqsave(&priv->irq_status_lock, flags);

	stat = MOCA_RD(priv->base + priv->regs->l2_status_offset);

	if (flush == FLUSH_IRQ) {
		MOCA_WR(priv->base + r->l2_clear_offset, stat);
		MOCA_RD(priv->base + r->l2_clear_offset);
	}
	if (flush == FLUSH_DMA_ONLY) {
		MOCA_WR(priv->base + r->l2_clear_offset,
			stat & dma_mask);
		MOCA_RD(priv->base + r->l2_clear_offset);
	}
	if (flush == FLUSH_REQRESP_ONLY) {
		MOCA_WR(priv->base + r->l2_clear_offset,
			stat & (M2H_RESP | M2H_REQ |
			M2H_RESP_CPU0 | M2H_REQ_CPU0));
		MOCA_RD(priv->base + r->l2_clear_offset);
	}

	spin_unlock_irqrestore(&priv->irq_status_lock, flags);

	return stat;
}

static void moca_enable_irq(struct moca_priv_data *priv)
{
	const struct moca_regs *r = priv->regs;

	/* unmask everything */
	u32 mask = M2H_REQ | M2H_RESP | M2H_ASSERT | M2H_WDT_CPU1 |
		M2H_NEXTCHUNK | M2H_DMA;

	if (moca_is_20(priv))
		mask |= M2H_WDT_CPU0 | M2H_NEXTCHUNK_CPU0 |
			M2H_REQ_CPU0 | M2H_RESP_CPU0 | M2H_ASSERT_CPU0;

	MOCA_WR(priv->base + r->l2_mask_clear_offset, mask);
	MOCA_RD(priv->base + r->l2_mask_clear_offset);
}

static void moca_disable_irq(struct moca_priv_data *priv)
{
	const struct moca_regs *r = priv->regs;

	/* mask everything except DMA completions */
	u32 mask = M2H_REQ | M2H_RESP | M2H_ASSERT | M2H_WDT_CPU1 |
		M2H_NEXTCHUNK;

	if (moca_is_20(priv))
		mask |= M2H_WDT_CPU0 | M2H_NEXTCHUNK_CPU0 |
			M2H_REQ_CPU0 | M2H_RESP_CPU0 | M2H_ASSERT_CPU0;

	MOCA_WR(priv->base + r->l2_mask_set_offset, mask);
	MOCA_RD(priv->base + r->l2_mask_set_offset);
}

static u32 moca_start_mips(struct moca_priv_data *priv, u32 cpu)
{
	const struct moca_regs *r = priv->regs;

	if (moca_is_20(priv)) {
		if (cpu == 1)
			MOCA_UNSET(priv->base + r->sw_reset_offset, BIT(0));
		else {
			moca_mmp_init(priv, 1);
			MOCA_UNSET(priv->base + r->sw_reset_offset, BIT(2));
		}
	} else
		MOCA_UNSET(priv->base + r->sw_reset_offset, BIT(0));
	MOCA_RD(priv->base + r->sw_reset_offset);
	return 0;
}

static void moca_m2m_xfer(struct moca_priv_data *priv,
	u32 dst, u32 src, u32 ctl)
{
	const struct moca_regs *r = priv->regs;
	u32 status;

	MOCA_WR(priv->base + r->m2m_src_offset, src);
	MOCA_WR(priv->base + r->m2m_dst_offset, dst);
	MOCA_WR(priv->base + r->m2m_status_offset, 0);
	MOCA_RD(priv->base + r->m2m_status_offset);
	MOCA_WR(priv->base + r->m2m_cmd_offset, ctl);

	if (wait_for_completion_timeout(&priv->copy_complete,
		1000 * M2M_TIMEOUT_MS) <= 0) {
		dev_warn(priv->dev, "DMA interrupt timed out, status %x\n",
			 moca_irq_status(priv, NO_FLUSH_IRQ));
	}

	status = MOCA_RD(priv->base + r->m2m_status_offset);

	if (status & (3 << 29))
		dev_warn(priv->dev, "bad status %08x (s/d/c %08x %08x %08x)\n",
			 status, src, dst, ctl);
}

static void moca_write_mem(struct moca_priv_data *priv,
	u32 dst_offset, void *src, unsigned int len)
{
	dma_addr_t pa;

	if (moca_range_ok(priv, dst_offset, len) < 0) {
		dev_warn(priv->dev, "copy past end of cntl memory: %08x\n",
			 dst_offset);
		return;
	}

	pa = dma_map_single(&priv->pdev->dev, src, len, DMA_TO_DEVICE);
	moca_m2m_xfer(priv, dst_offset + priv->regs->data_mem_offset, (u32)pa,
		len | M2M_WRITE);
	dma_unmap_single(&priv->pdev->dev, pa, len, DMA_TO_DEVICE);
}

static void moca_read_mem(struct moca_priv_data *priv,
	void *dst, u32 src_offset, unsigned int len)
{
	int i;

	if (moca_range_ok(priv, src_offset, len) < 0) {
		dev_warn(priv->dev, "copy past end of cntl memory: %08x\n",
			 src_offset);
		return;
	}

	for (i = 0; i < len; i += 4)
		DEV_WR(dst + i, cpu_to_be32(
			MOCA_RD(priv->base + src_offset +
				priv->regs->data_mem_offset + i)));
}

static void moca_write_sg(struct moca_priv_data *priv,
	u32 dst_offset, struct scatterlist *sg, int nents)
{
	int j;
	uintptr_t addr = priv->regs->data_mem_offset + dst_offset;

	dma_map_sg(&priv->pdev->dev, sg, nents, DMA_TO_DEVICE);

	for (j = 0; j < nents; j++) {
		moca_m2m_xfer(priv, addr, (u32)sg[j].dma_address,
			sg[j].length | M2M_WRITE);

		addr += sg[j].length;
	}

	dma_unmap_sg(&priv->pdev->dev, sg, nents, DMA_TO_DEVICE);
}

static inline void moca_read_sg(struct moca_priv_data *priv,
	u32 src_offset, struct scatterlist *sg, int nents)
{
	int j;
	uintptr_t addr = priv->regs->data_mem_offset + src_offset;

	dma_map_sg(&priv->pdev->dev, sg, nents, DMA_FROM_DEVICE);

	for (j = 0; j < nents; j++) {
		moca_m2m_xfer(priv, (u32)sg[j].dma_address, addr,
			sg[j].length | M2M_READ);

		addr += sg[j].length;
		SetPageDirty(sg_page(&sg[j]));
	}

	dma_unmap_sg(&priv->pdev->dev, sg, nents, DMA_FROM_DEVICE);
}

#define moca_3450_write moca_3450_write_i2c
#define moca_3450_read moca_3450_read_i2c

static void moca_put_pages(struct moca_priv_data *priv, int pages)
{
	int i;

	for (i = 0; i < pages; i++)
		page_cache_release(priv->fw_pages[i]);
}

static int moca_get_pages(struct moca_priv_data *priv, unsigned long addr,
	int size, unsigned int moca_addr, int write)
{
	unsigned int pages, chunk_size;
	int ret, i;

	if (addr & 3)
		return -EINVAL;
	if ((size <= 0) || (size > MAX_FW_SIZE))
		return -EINVAL;

	pages = ((addr & ~PAGE_MASK) + size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	down_read(&current->mm->mmap_sem);
	ret = get_user_pages(current, current->mm, addr & PAGE_MASK, pages,
		write, 0, priv->fw_pages, NULL);
	up_read(&current->mm->mmap_sem);

	if (ret < 0)
		return ret;
	BUG_ON((ret > MAX_FW_PAGES) || (pages == 0));

	if (ret < pages) {
		dev_warn(priv->dev, "get_user_pages returned %d, expecting %d\n",
			 ret, pages);
		moca_put_pages(priv, ret);
		return -EFAULT;
	}

	chunk_size = PAGE_SIZE - (addr & ~PAGE_MASK);
	if (size < chunk_size)
		chunk_size = size;

	sg_set_page(&priv->fw_sg[0], priv->fw_pages[0], chunk_size,
		addr & ~PAGE_MASK);
	size -= chunk_size;

	for (i = 1; i < pages; i++) {
		sg_set_page(&priv->fw_sg[i], priv->fw_pages[i],
			size > PAGE_SIZE ? PAGE_SIZE : size, 0);
		size -= PAGE_SIZE;
	}
	return ret;
}

static int moca_write_img(struct moca_priv_data *priv, struct moca_xfer *x)
{
	int pages, i, ret = -EINVAL;
	struct moca_fw_hdr hdr;
	u32 bl_chunks;

	if (copy_from_user(&hdr, (void __user *)(unsigned long)x->buf,
			sizeof(hdr)))
		return -EFAULT;

	bl_chunks = be32_to_cpu(hdr.bl_chunks);
	if (!bl_chunks || (bl_chunks > MAX_BL_CHUNKS))
		bl_chunks = 1;

	pages = moca_get_pages(priv, (unsigned long)x->buf, x->len, 0, 0);
	if (pages < 0)
		return pages;
	if (pages < (bl_chunks + 2))
		goto out;

	/* host must use FW_CHUNK_SIZE MMU pages (for now) */
	BUG_ON(FW_CHUNK_SIZE != PAGE_SIZE);

	/* write the first two chunks, then start the MIPS */
	moca_write_sg(priv, 0, &priv->fw_sg[0], bl_chunks + 1);
	moca_enable_irq(priv);
	moca_start_mips(priv, be32_to_cpu(hdr.cpuid));
	ret = 0;

	/* wait for an ACK, then write each successive chunk */
	for (i = bl_chunks + 1; i < pages; i++) {
		if (wait_for_completion_timeout(&priv->chunk_complete,
				1000 * M2M_TIMEOUT_MS) <= 0) {
			moca_disable_irq(priv);
			dev_warn(priv->dev, "chunk ack timed out\n");
			ret = -EIO;
			break;
		}
		moca_write_sg(priv,
			priv->regs->data_mem_offset + FW_CHUNK_SIZE * bl_chunks,
			&priv->fw_sg[i], 1);
	}

  /* wait for ACK of last block.  Older firmware images didn't
     ACK the last block, so don't return an error */
	wait_for_completion_timeout(&priv->chunk_complete,
			1000 * M2M_TIMEOUT_MS / 10);

out:
	moca_put_pages(priv, pages);
	return ret;
}

/*
 * MESSAGE AND LIST HANDLING
 */

static void moca_handle_lab_printf(struct moca_priv_data *priv,
	struct moca_core_msg *m)
{
	u32 str_len;
	u32 str_addr;

	if (priv->mmp_20) {
		str_len = (be32_to_cpu(m->data[4]) + 3) & ~3;
		str_addr = be32_to_cpu(m->data[3]) & 0x1fffffff;

		if ((be32_to_cpu(m->data[0]) == 0x3) &&
		    (be32_to_cpu(m->data[1]) == 12) &&
		    ((be32_to_cpu(m->data[2]) & 0xffffff) == 0x090801) &&
		    (be32_to_cpu(m->data[4]) <= MAX_LAB_PRINTF)) {
			m->len = 3 + str_len;
			moca_read_mem(priv, &m->data[3], str_addr, str_len);

			m->data[1] = cpu_to_be32(m->len - 8);
		}
	} else {
		str_len = (be32_to_cpu(m->data[3]) + 3) & ~3;
		str_addr = be32_to_cpu(m->data[2]) & 0x1fffffff;

		if ((be32_to_cpu(m->data[0]) & 0xff0000ff) == 0x09000001 &&
			be32_to_cpu(m->data[1]) == 0x600b0008 &&
			(be32_to_cpu(m->data[3]) <= MAX_LAB_PRINTF)) {

			m->len = 8 + str_len;
			moca_read_mem(priv, &m->data[2], str_addr, str_len);

			m->data[1] = cpu_to_be32((MOCA_IE_DRV_PRINTF << 16) +
				m->len - 8);
		}
	}
}
static void moca_msg_reset(struct moca_priv_data *priv)
{
	int i;

	if (priv->running)
		moca_disable_irq(priv);
	priv->running = 0;
	priv->host_mbx_busy = 0;
	priv->host_resp_pending = 0;
	priv->core_req_pending = 0;
	priv->assert_pending = 0;
	priv->mbx_offset[0] = -1;
	priv->mbx_offset[1] = -1;

	spin_lock_bh(&priv->list_lock);
	INIT_LIST_HEAD(&priv->core_msg_free_list);
	INIT_LIST_HEAD(&priv->core_msg_pend_list);

	for (i = 0; i < NUM_CORE_MSG; i++)
		list_add_tail(&priv->core_msg_queue[i].chain,
			&priv->core_msg_free_list);

	INIT_LIST_HEAD(&priv->host_msg_free_list);
	INIT_LIST_HEAD(&priv->host_msg_pend_list);

	for (i = 0; i < NUM_HOST_MSG; i++)
		list_add_tail(&priv->host_msg_queue[i].chain,
			&priv->host_msg_free_list);
	spin_unlock_bh(&priv->list_lock);
}

static struct list_head *moca_detach_head(struct moca_priv_data *priv,
	struct list_head *h)
{
	struct list_head *r = NULL;

	spin_lock_bh(&priv->list_lock);
	if (!list_empty(h)) {
		r = h->next;
		list_del(r);
	}
	spin_unlock_bh(&priv->list_lock);

	return r;
}

static void moca_attach_tail(struct moca_priv_data *priv,
	struct list_head *elem, struct list_head *list)
{
	spin_lock_bh(&priv->list_lock);
	list_add_tail(elem, list);
	spin_unlock_bh(&priv->list_lock);
}

/* Must have dev_mutex when calling this function */
static int moca_recvmsg(struct moca_priv_data *priv, uintptr_t offset,
	u32 max_size, uintptr_t reply_offset, u32 cpuid)
{
	struct list_head *ml = NULL;
	struct moca_core_msg *m;
	unsigned int w, rw, num_ies;
	u32 data, size;
	char *msg;
	int err = -ENOMEM;
	u32 *reply = priv->core_resp_buf;
	int attach = 1;

	m = &priv->core_msg_temp;

	BUG_ON((uintptr_t)m->data & (L1_CACHE_BYTES - 1));

	/* make sure we have the mailbox offset before using it */
	moca_get_mbx_offset(priv);

	/* read only as much as is necessary.
	   The second word is the length for mmp_20 */
	if (priv->mmp_20) {
		moca_read_mem(priv, m->data,
			offset + priv->mbx_offset[cpuid], 8);

		size = (be32_to_cpu(m->data[1])+3) & 0xFFFFFFFC;
		/* if size is too large, this is a protocol error.
		   mocad will output the error message */
		if (size > max_size - 8)
			size = max_size - 8;

		moca_read_mem(priv, &m->data[2],
			offset + priv->mbx_offset[cpuid] + 8, size);
	} else
		moca_read_mem(priv, m->data,
			offset + priv->mbx_offset[cpuid], max_size);

	data = be32_to_cpu(m->data[0]);

	if (priv->mmp_20) {
		/* In MoCA 2.0, there is only 1 IE per message */
		num_ies = 1;
	} else {
		num_ies = data & 0xffff;
	}

	if (reply_offset) {
		if (priv->mmp_20) {
			/* In MoCA 2.0, the ACK is to simply set the
			   MSB in the incoming message and send it
			   back */
			reply[0] = cpu_to_be32(data | 0x80000000);
			rw = 1;
		} else {
			/* ACK + seq number + number of IEs */
			reply[0] = cpu_to_be32((data & 0x00ff0000) |
				0x04000000 | num_ies);
			rw = 1;
		}
	}

	err = -EINVAL;
	w = 1;
	max_size >>= 2;
	while (num_ies) {
		if (w >= max_size) {
			msg = "dropping long message";
			goto bad;
		}

		data = be32_to_cpu(m->data[w++]);

		if (reply_offset && !priv->mmp_20) {
			/*
			 * ACK each IE in the original message;
			 * return code is always 0
			 */
			if ((rw << 2) >= priv->core_resp_size)
				dev_warn(priv->dev, "Core ack buffer overflowed\n");
			else {
				reply[rw] = cpu_to_be32((data & ~0xffff) | 4);
				rw++;
				reply[rw] = cpu_to_be32(0);
				rw++;
			}
		}
		if (data & 3) {
			msg = "IE is not a multiple of 4 bytes";
			goto bad;
		}

		w += ((data & 0xffff) >> 2);

		if (w > max_size) {
			msg = "dropping long message";
			goto bad;
		}
		num_ies--;
	}
	m->len = w << 2;

	/* special case for lab_printf traps */
	moca_handle_lab_printf(priv, m);

	/*
	 * Check to see if we can add this new message to the current queue.
	 * The result will be a single message with multiple IEs.
	 */
	if (!priv->mmp_20) {
		spin_lock_bh(&priv->list_lock);
		if (!list_empty(&priv->core_msg_pend_list)) {
			ml = priv->core_msg_pend_list.prev;
			m = list_entry(ml, struct moca_core_msg, chain);

			if (m->len + priv->core_msg_temp.len > max_size)
				ml = NULL;
			else {
				u32 d0 = be32_to_cpu(
						priv->core_msg_temp.data[0]);

				/* Only concatenate traps from the core */
				if (((be32_to_cpu(m->data[0]) & 0xff000000) !=
					0x09000000) ||
					((d0 & 0xff000000) != 0x09000000))
					ml = NULL;
				else {
					/*
					 * We can add the message to the
					 * previous one. Update the num of IEs,
					 * update the length and copy the data.
					 */
					data = be32_to_cpu(m->data[0]);
					num_ies = data & 0xffff;
					num_ies += d0 & 0xffff;
					data &= 0xffff0000;
					data |= num_ies;
					m->data[0] = cpu_to_be32(data);

					/*
					 * Subtract 4 bytes from length for
					   message header
					 */
					memcpy(&m->data[m->len >> 2],
						&priv->core_msg_temp.data[1],
						priv->core_msg_temp.len - 4);
					m->len += priv->core_msg_temp.len - 4;
					attach = 0;
				}
			}
		}
		spin_unlock_bh(&priv->list_lock);
	}

	if (ml == NULL) {
		ml = moca_detach_head(priv, &priv->core_msg_free_list);
		if (ml == NULL) {
			msg = "no entries left on core_msg_free_list";
			err = -ENOMEM;
			goto bad;
		}
		m = list_entry(ml, struct moca_core_msg, chain);

		memcpy(m->data, priv->core_msg_temp.data,
			priv->core_msg_temp.len);
		m->len = priv->core_msg_temp.len;
	}

	if (reply_offset) {
		if ((cpuid == 1) &&
			(moca_irq_status(priv, NO_FLUSH_IRQ) & M2H_ASSERT)) {
			/* do not retry - message is gone forever */
			err = 0;
			msg = "core_req overwritten by assertion";
			goto bad;
		}
		if ((cpuid == 0) &&
			(moca_irq_status(priv, NO_FLUSH_IRQ)
			& M2H_ASSERT_CPU0)) {
			/* do not retry - message is gone forever */
			err = 0;
			msg = "core_req overwritten by assertion";
			goto bad;
		}
		moca_write_mem(priv, reply_offset + priv->mbx_offset[cpuid],
			reply, rw << 2);
		moca_ringbell(priv, priv->regs->h2m_resp_bit[cpuid]);
	}

	if (attach) {
		moca_attach_tail(priv, ml, &priv->core_msg_pend_list);
		wake_up(&priv->core_msg_wq);
	}

	return 0;

bad:
	dev_warn(priv->dev, "%s\n", msg);

	if (ml)
		moca_attach_tail(priv, ml, &priv->core_msg_free_list);

	return err;
}

static int moca_h2m_sanity_check(struct moca_priv_data *priv,
	struct moca_host_msg *m)
{
	unsigned int w, num_ies;
	u32 data;

	if (priv->mmp_20) {
		/* The length is stored in data[1]
		   plus 8 extra header bytes */
		data = be32_to_cpu(m->data[1]) + 8;
		if (data > priv->host_req_size)
			return -1;
		else
			return (int) data;
	} else {
		data = be32_to_cpu(m->data[0]);
		num_ies = data & 0xffff;

		w = 1;
		while (num_ies) {
			if (w >= (m->len << 2))
				return -1;

			data = be32_to_cpu(m->data[w++]);

			if (data & 3)
				return -1;
			w += (data & 0xffff) >> 2;
			num_ies--;
		}
		return w << 2;
	}
}

/* Must have dev_mutex when calling this function */
static int moca_sendmsg(struct moca_priv_data *priv, u32 cpuid)
{
	struct list_head *ml = NULL;
	struct moca_host_msg *m;

	if (priv->host_mbx_busy == 1)
		return -1;

	ml = moca_detach_head(priv, &priv->host_msg_pend_list);
	if (ml == NULL)
		return -EAGAIN;
	m = list_entry(ml, struct moca_host_msg, chain);

	moca_write_mem(priv, priv->mbx_offset[cpuid] + priv->host_req_offset,
		m->data, m->len);

	moca_ringbell(priv, priv->regs->h2m_req_bit[cpuid]);
	moca_attach_tail(priv, ml, &priv->host_msg_free_list);
	wake_up(&priv->host_msg_wq);

	return 0;
}

/* Must have dev_mutex when calling this function */
static int moca_wdt(struct moca_priv_data *priv, u32 cpu)
{
	struct list_head *ml = NULL;
	struct moca_core_msg *m;

	ml = moca_detach_head(priv, &priv->core_msg_free_list);
	if (ml == NULL) {
		dev_warn(priv->dev, "no entries left on core_msg_free_list\n");
		return -ENOMEM;
	}

	if (priv->mmp_20) {
		/*
		 * generate phony wdt message to pass to the user
		 * type = 0x03 (trap)
		 * IE type = 0x11003 (wdt), 4 bytes length
		 */
		m = list_entry(ml, struct moca_core_msg, chain);
		m->data[0] = cpu_to_be32(0x3);
		m->data[1] = cpu_to_be32(4);
		m->data[2] = cpu_to_be32(0x11003);
		m->len = 12;
	} else {
		/*
		 * generate phony wdt message to pass to the user
		 * type = 0x09 (trap)
		 * IE type = 0xff01 (wdt), 4 bytes length
		 */
		m = list_entry(ml, struct moca_core_msg, chain);
		m->data[0] = cpu_to_be32(0x09000001);
		m->data[1] = cpu_to_be32((MOCA_IE_WDT << 16) | 4);
		m->data[2] = cpu_to_be32(cpu);
		m->len = 12;
	}

	moca_attach_tail(priv, ml, &priv->core_msg_pend_list);
	wake_up(&priv->core_msg_wq);

	return 0;
}

static int moca_get_mbx_offset(struct moca_priv_data *priv)
{
	const struct moca_regs *r = priv->regs;
	uintptr_t base;

	if (priv->mbx_offset[1] == -1) {
		if (moca_is_20(priv))
			base = MOCA_RD(priv->base +
				r->moca2host_mmp_inbox_0_offset) &
				0x1fffffff;
		else
			base = MOCA_RD(priv->base + r->gp0_offset) &
				0x1fffffff;

		if ((base == 0) ||
			(base >= r->cntl_mem_size + r->cntl_mem_offset) ||
			(base & 0x07)) {
			dev_warn(priv->dev, "can't get mailbox base CPU 1 (%X)\n",
				 (int)base);
			return -1;
		}
		priv->mbx_offset[1] = base;
	}

	if ((priv->mbx_offset[0] == -1) && moca_is_20(priv) && priv->mmp_20) {
		base = MOCA_RD(priv->base +
			r->moca2host_mmp_inbox_2_offset) &
			0x1fffffff;
		if ((base == 0) ||
			(base >= r->cntl_mem_size + r->cntl_mem_offset) ||
			(base & 0x07)) {
			dev_warn(priv->dev, "can't get mailbox base CPU 0 (%X)\n",
				 (int)base);
			return -1;
		}

		priv->mbx_offset[0] = base;
	}

	return 0;
}

/*
 * INTERRUPT / WORKQUEUE BH
 */

static void moca_work_handler(struct work_struct *work)
{
	struct moca_priv_data *priv =
		container_of(work, struct moca_priv_data, work);
	u32 mask = 0;
	int ret, stopped = 0;

	if (priv->enabled) {
		mask = moca_irq_status(priv, FLUSH_IRQ);
		if (mask & M2H_DMA) {
			mask &= ~M2H_DMA;
			complete(&priv->copy_complete);
		}

		if (mask & M2H_NEXTCHUNK) {
			mask &= ~M2H_NEXTCHUNK;
			complete(&priv->chunk_complete);
		}

		if (moca_is_20(priv) && mask & M2H_NEXTCHUNK_CPU0) {
			mask &= ~M2H_NEXTCHUNK_CPU0;
			complete(&priv->chunk_complete);
		}

		if (mask == 0) {
			moca_enable_irq(priv);
			return;
		}

		if (mask & (M2H_REQ | M2H_RESP |
			M2H_REQ_CPU0 | M2H_RESP_CPU0)) {
			if (moca_get_mbx_offset(priv)) {
				/* mbx interrupt but mbx_offset is bogus?? */
				moca_enable_irq(priv);
				return;
			}
		}
	}

	mutex_lock(&priv->dev_mutex);

	if (!priv->running) {
		stopped = 1;
	} else {
		/* fatal events */
		if (mask & M2H_ASSERT) {
			ret = moca_recvmsg(priv, priv->core_req_offset,
				priv->core_req_size, 0, 1);
			if (ret == -ENOMEM)
				priv->assert_pending = 2;
		}
		if (mask & M2H_ASSERT_CPU0) {
			ret = moca_recvmsg(priv, priv->core_req_offset,
				priv->core_req_size, 0, 0);
			if (ret == -ENOMEM)
				priv->assert_pending = 1;
		}
		/* M2H_WDT_CPU1 is mapped to the only CPU for MoCA11 HW */
		if (mask & M2H_WDT_CPU1) {
			ret = moca_wdt(priv, 2);
			if (ret == -ENOMEM)
				priv->wdt_pending |= BIT(1);
			stopped = 1;
		}
		if (moca_is_20(priv) && mask & M2H_WDT_CPU0) {
			ret = moca_wdt(priv, 1);
			if (ret == -ENOMEM)
				priv->wdt_pending |= BIT(0);
			stopped = 1;
		}
	}
	if (stopped) {
		priv->running = 0;
		priv->core_req_pending = 0;
		priv->host_resp_pending = 0;
		priv->host_mbx_busy = 1;
		mutex_unlock(&priv->dev_mutex);
		wake_up(&priv->core_msg_wq);
		return;
	}

	/* normal events */
	if (mask & M2H_REQ) {
		ret = moca_recvmsg(priv, priv->core_req_offset,
			priv->core_req_size, priv->core_resp_offset, 1);
		if (ret == -ENOMEM)
			priv->core_req_pending = 2;
	}
	if (mask & M2H_RESP) {
		ret = moca_recvmsg(priv, priv->host_resp_offset,
			priv->host_resp_size, 0, 1);
		if (ret == -ENOMEM)
			priv->host_resp_pending = 2;
		if (ret == 0) {
			priv->host_mbx_busy = 0;
			moca_sendmsg(priv, 1);
		}
	}

	if (mask & M2H_REQ_CPU0) {
		ret = moca_recvmsg(priv, priv->core_req_offset,
			priv->core_req_size, priv->core_resp_offset, 0);
		if (ret == -ENOMEM)
			priv->core_req_pending = 1;
	}
	if (mask & M2H_RESP_CPU0) {
		ret = moca_recvmsg(priv, priv->host_resp_offset,
			priv->host_resp_size, 0, 0);
		if (ret == -ENOMEM)
			priv->host_resp_pending = 1;
		if (ret == 0) {
			priv->host_mbx_busy = 0;
			moca_sendmsg(priv, 0);
		}
	}
	mutex_unlock(&priv->dev_mutex);

	moca_enable_irq(priv);
}

static irqreturn_t moca_interrupt(int irq, void *arg)
{
	struct moca_priv_data *priv = arg;

	if (1) {
		u32 mask = moca_irq_status(priv, FLUSH_DMA_ONLY);

		/* need to handle DMA completions ASAP */
		if (mask & M2H_DMA) {
			complete(&priv->copy_complete);
			mask &= ~M2H_DMA;
		}
		if (mask & M2H_NEXTCHUNK) {
			complete(&priv->chunk_complete);
			mask &= ~M2H_NEXTCHUNK;
		}

		if (!mask)
			return IRQ_HANDLED;
	}
	moca_disable_irq(priv);
	schedule_work(&priv->work);

	return IRQ_HANDLED;
}

/*
 * BCM3450 ACCESS VIA I2C
 */

static int moca_3450_wait(struct moca_priv_data *priv)
{
	struct bsc_regs *bsc = priv->i2c_base;
	long timeout = HZ / 1000;	/* 1ms */
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wait);
	int i = 0;

	do {
		if (I2C_RD(&bsc->iic_enable) & 2) {
			I2C_WR(&bsc->iic_enable, 0);
			return 0;
		}
		if (i++ > 50) {
			I2C_WR(&bsc->iic_enable, 0);
			dev_warn(priv->dev, "3450 I2C timed out\n");
			return -1;
		}
		sleep_on_timeout(&wait, timeout ? timeout : 1);
	} while (1);
}

static void moca_3450_write_i2c(struct moca_priv_data *priv, u8 addr, u32 data)
{
	struct bsc_regs *bsc = priv->i2c_base;
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;

	I2C_WR(&bsc->iic_enable, 0);
	I2C_WR(&bsc->chip_address, pd->bcm3450_i2c_addr << 1);
	I2C_WR(&bsc->data_in[0], (addr >> 2) | (data << 8));
	I2C_WR(&bsc->data_in[1], data >> 24);
	I2C_WR(&bsc->cnt_reg, (5 << 0) | (0 << 6));	/* 5B out, 0B in */
	I2C_WR(&bsc->ctl_reg, (1 << 4) | (0 << 0));	/* write only, 390kHz */
	I2C_WR(&bsc->ctlhi_reg, (1 << 6));		/* 32-bit words */
	I2C_WR(&bsc->iic_enable, 1);

	moca_3450_wait(priv);
}

static u32 moca_3450_read_i2c(struct moca_priv_data *priv, u8 addr)
{
	struct bsc_regs *bsc = priv->i2c_base;
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;

	I2C_WR(&bsc->iic_enable, 0);
	I2C_WR(&bsc->chip_address, pd->bcm3450_i2c_addr << 1);
	I2C_WR(&bsc->data_in[0], (addr >> 2));
	I2C_WR(&bsc->cnt_reg, (1 << 0) | (4 << 6));	/* 1B out then 4B in */
	I2C_WR(&bsc->ctl_reg, (1 << 4) | (3 << 0));	/* write/read, 390kHz */
	I2C_WR(&bsc->ctlhi_reg, (1 << 6));		/* 32-bit words */
	I2C_WR(&bsc->iic_enable, 1);

	if (moca_3450_wait(priv) == 0)
		return I2C_RD(&bsc->data_out[0]);
	else
		return 0xffffffff;
}

#define BCM3450_CHIP_ID		0x00
#define BCM3450_CHIP_REV	0x04
#define BCM3450_LNACNTL		0x14
#define BCM3450_PACNTL		0x18
#define BCM3450_MISC		0x1c

static void moca_3450_init(struct moca_priv_data *priv, int action)
{
	u32 data;

	mutex_lock(&priv->moca_i2c_mutex);

	if (action == MOCA_ENABLE) {
		/* reset the 3450's I2C block */
		moca_3450_write(priv, BCM3450_MISC,
			moca_3450_read(priv, BCM3450_MISC) | 1);

		/* verify chip ID */
		data = moca_3450_read(priv, BCM3450_CHIP_ID);
		if (data != 0x3450)
			dev_warn(priv->dev, "invalid 3450 chip ID 0x%08x\n",
				 data);

		/* reset the 3450's deserializer */
		data = moca_3450_read(priv, BCM3450_MISC);
		data &= ~0x8000; /* power on PA/LNA */
		moca_3450_write(priv, BCM3450_MISC, data | 2);
		moca_3450_write(priv, BCM3450_MISC, data & ~2);

		/* set new PA gain */
		data = moca_3450_read(priv, BCM3450_PACNTL);

		moca_3450_write(priv, BCM3450_PACNTL, (data & ~0x02007ffd) |
			(0x09 << 11) |		/* RDEG */
			(0x38 << 5) |		/* CURR_CONT */
			(0x05 << 2));		/* CURR_FOLLOWER */

		/* Set LNACNTRL to default value */
		moca_3450_write(priv, BCM3450_LNACNTL, 0x4924);

	} else {
		/* power down the PA/LNA */
		data = moca_3450_read(priv, BCM3450_MISC);
		moca_3450_write(priv, BCM3450_MISC, data | 0x8000);

		data = moca_3450_read(priv, BCM3450_PACNTL);
		moca_3450_write(priv, BCM3450_PACNTL, data |
			BIT(0) |	/* PA_PWRDWN */
			BIT(25));	/* PA_SELECT_PWRUP_BSC */

		data = moca_3450_read(priv, BCM3450_LNACNTL);
		/* LNA_INBIAS=0, LNA_PWRUP_IIC=0: */
		data &= ~((7<<12) | (1<<28));
		/* LNA_SELECT_PWRUP_IIC=1: */
		moca_3450_write(priv, BCM3450_LNACNTL, data | (1<<29));

	}
	mutex_unlock(&priv->moca_i2c_mutex);
}

/*
 * FILE OPERATIONS
 */

static int moca_file_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct moca_priv_data *priv;

	if ((minor > NUM_MINORS) || minor_tbl[minor] == NULL)
		return -ENODEV;

	file->private_data = priv = minor_tbl[minor];

	mutex_lock(&priv->dev_mutex);
	priv->refcount++;
	mutex_unlock(&priv->dev_mutex);
	return 0;
}

static int moca_file_release(struct inode *inode, struct file *file)
{
	struct moca_priv_data *priv = file->private_data;

	mutex_lock(&priv->dev_mutex);
	priv->refcount--;
	if (priv->refcount == 0 && priv->running == 1) {
		/* last user closed the device */
		moca_msg_reset(priv);
		moca_hw_init(priv, MOCA_DISABLE);
	}
	mutex_unlock(&priv->dev_mutex);
	return 0;
}

static int moca_ioctl_readmem(struct moca_priv_data *priv,
	unsigned long xfer_uaddr)
{
	struct moca_xfer x;
	uintptr_t i, src;
	u32 *dst;

	if (copy_from_user(&x, (void __user *)xfer_uaddr, sizeof(x)))
		return -EFAULT;

	if (moca_range_ok(priv, x.moca_addr, x.len) < 0)
		return -EINVAL;

	src = (uintptr_t)priv->base + x.moca_addr;
	dst = (void *)(unsigned long)x.buf;

	for (i = 0; i < x.len; i += 4, src += 4, dst++)
		if (put_user(cpu_to_be32(MOCA_RD(src)), dst))
			return -EFAULT;

	return 0;
}

static int moca_ioctl_writemem(struct moca_priv_data *priv,
	unsigned long xfer_uaddr)
{
	struct moca_xfer x;
	uintptr_t i, dst;
	u32 *src;

	if (copy_from_user(&x, (void __user *)xfer_uaddr, sizeof(x)))
		return -EFAULT;

	if (moca_range_ok(priv, x.moca_addr, x.len) < 0)
		return -EINVAL;

	dst = (uintptr_t)priv->base + x.moca_addr;
	src = (void *)(unsigned long)x.buf;

	for (i = 0; i < x.len; i += 4, src++, dst += 4) {
		unsigned int x;
		if (get_user(x, src))
			return -EFAULT;

		MOCA_WR(dst, cpu_to_be32(x));
	}

	return 0;
}

/* legacy ioctl - DEPRECATED */
static int moca_ioctl_get_drv_info_v2(struct moca_priv_data *priv,
	unsigned long arg)
{
	struct moca_kdrv_info_v2 info;
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;

	memset(&info, 0, sizeof(info));
	info.version = DRV_VERSION;
	info.build_number = DRV_BUILD_NUMBER;
	info.builtin_fw = !!bmoca_fw_image;

	info.uptime = (jiffies - priv->start_time) / HZ;
	info.refcount = priv->refcount;
	if (moca_is_20(priv))
		info.gp1 = priv->running ? MOCA_RD(priv->base +
			priv->regs->moca2host_mmp_inbox_1_offset) : 0;
	else
		info.gp1 = priv->running ?
			MOCA_RD(priv->base + priv->regs->gp1_offset) : 0;

	memcpy(info.enet_name, pd->enet_name, MOCA_IFNAMSIZ);

	info.enet_id = pd->enet_id;
	info.macaddr_hi = pd->macaddr_hi;
	info.macaddr_lo = pd->macaddr_lo;
	info.hw_rev = pd->chip_id;
	info.rf_band = pd->rf_band;

	if (copy_to_user((void *)arg, &info, sizeof(info)))
		return -EFAULT;

	return 0;
}

static int moca_ioctl_get_drv_info(struct moca_priv_data *priv,
	unsigned long arg)
{
	struct moca_kdrv_info info;
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;

	memset(&info, 0, sizeof(info));
	info.version = DRV_VERSION;
	info.build_number = DRV_BUILD_NUMBER;
	info.builtin_fw = !!bmoca_fw_image;

	info.uptime = (jiffies - priv->start_time) / HZ;
	info.refcount = priv->refcount;
	if (moca_is_20(priv))
		info.gp1 = priv->running ? MOCA_RD(priv->base +
			priv->regs->moca2host_mmp_inbox_1_offset) : 0;
	else
		info.gp1 = priv->running ?
			MOCA_RD(priv->base + priv->regs->gp1_offset) : 0;

	memcpy(info.enet_name, pd->enet_name, MOCA_IFNAMSIZ);

	info.enet_id = pd->enet_id;
	info.macaddr_hi = pd->macaddr_hi;
	info.macaddr_lo = pd->macaddr_lo;
	info.chip_id = pd->chip_id;
	info.hw_rev = pd->hw_rev;
	info.rf_band = pd->rf_band;
	info.phy_freq = priv->phy_freq;

	if (copy_to_user((void *)arg, &info, sizeof(info)))
		return -EFAULT;

	return 0;
}

static int moca_ioctl_check_for_data(struct moca_priv_data *priv,
	unsigned long arg)
{
	int data_avail = 0;
	int ret;
	u32 mask;

	moca_disable_irq(priv);

	moca_get_mbx_offset(priv);

	/* If an IRQ is pending, process it here rather than waiting for it to
	   ensure the results are ready. Clear the ones we are currently
	   processing */
	mask = moca_irq_status(priv, FLUSH_REQRESP_ONLY);

	if (mask & M2H_REQ) {
		ret = moca_recvmsg(priv, priv->core_req_offset,
			priv->core_req_size, priv->core_resp_offset, 1);
		if (ret == -ENOMEM)
			priv->core_req_pending = 2;
	}
	if (mask & M2H_RESP) {
		ret = moca_recvmsg(priv, priv->host_resp_offset,
			priv->host_resp_size, 0, 1);
		if (ret == -ENOMEM)
			priv->host_resp_pending = 2;
		if (ret == 0) {
			priv->host_mbx_busy = 0;
			moca_sendmsg(priv, 1);
		}
	}

	if (mask & M2H_REQ_CPU0) {
		ret = moca_recvmsg(priv, priv->core_req_offset,
			priv->core_req_size, priv->core_resp_offset, 0);
		if (ret == -ENOMEM)
			priv->core_req_pending = 1;
	}
	if (mask & M2H_RESP_CPU0) {
		ret = moca_recvmsg(priv, priv->host_resp_offset,
			priv->host_resp_size, 0, 0);
		if (ret == -ENOMEM)
			priv->host_resp_pending = 1;
		if (ret == 0) {
			priv->host_mbx_busy = 0;
			moca_sendmsg(priv, 0);
		}
	}

	moca_enable_irq(priv);

	spin_lock_bh(&priv->list_lock);
	data_avail = !list_empty(&priv->core_msg_pend_list);
	spin_unlock_bh(&priv->list_lock);

	if (copy_to_user((void *)arg, &data_avail, sizeof(data_avail)))
		return -EFAULT;

	return 0;
}

static long moca_file_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct moca_priv_data *priv = file->private_data;
	struct moca_start start;
	long ret = -ENOTTY;

	mutex_lock(&priv->dev_mutex);

	switch (cmd) {
	case MOCA_IOCTL_START:
		ret = clk_set_rate(priv->phy_clk, DEFAULT_PHY_CLOCK);
		/* FIXME: this fails on some platforms, so ignore the value */
		ret = 0;
		if (ret < 0)
			break;

		if (copy_from_user(&start, (void __user *)arg, sizeof(start)))
			ret = -EFAULT;

		if (ret >= 0) {
			if (!priv->enabled) {
				moca_msg_reset(priv);
				moca_hw_init(priv, MOCA_ENABLE);
				moca_3450_init(priv, MOCA_ENABLE);
				moca_irq_status(priv, FLUSH_IRQ);
				moca_mmp_init(priv, 0);
			}
			priv->bonded_mode =
				(start.boot_flags & MOCA_BOOT_FLAGS_BONDED);
			ret = moca_write_img(priv, &start.x);
			if (ret >= 0)
				priv->running = 1;
		}
		break;
	case MOCA_IOCTL_STOP:
		moca_msg_reset(priv);
		moca_3450_init(priv, MOCA_DISABLE);
		moca_hw_init(priv, MOCA_DISABLE);
		ret = 0;
		break;
	case MOCA_IOCTL_READMEM:
		if (priv->running)
			ret = moca_ioctl_readmem(priv, arg);
		break;
	case MOCA_IOCTL_WRITEMEM:
		if (priv->running)
			ret = moca_ioctl_writemem(priv, arg);
		break;
	case MOCA_IOCTL_GET_DRV_INFO_V2:
		ret = moca_ioctl_get_drv_info_v2(priv, arg);
		break;
	case MOCA_IOCTL_GET_DRV_INFO:
		ret = moca_ioctl_get_drv_info(priv, arg);
		break;
	case MOCA_IOCTL_CHECK_FOR_DATA:
		if (priv->running)
			ret = moca_ioctl_check_for_data(priv, arg);
		else
			ret = -EIO;
		break;
	case MOCA_IOCTL_WOL:
		priv->wol_enabled = (int)arg;
		dev_info(priv->dev, "WOL is %s\n",
			priv->wol_enabled ? "enabled" : "disabled");
		ret = 0;
		break;
	case MOCA_IOCTL_SET_CPU_RATE:
		if (!priv->cpu_clk)
			ret = -EIO;
		else
			ret = clk_set_rate(priv->cpu_clk, (unsigned int)arg);
		break;
	case MOCA_IOCTL_SET_PHY_RATE:
		if (!priv->phy_clk)
			ret = -EIO;
		else
			ret = clk_set_rate(priv->phy_clk, (unsigned int)arg);
		break;
	}
	mutex_unlock(&priv->dev_mutex);

	return ret;
}

static ssize_t moca_file_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct moca_priv_data *priv = file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	struct list_head *ml = NULL;
	struct moca_core_msg *m = NULL;
	ssize_t ret;
	int empty_free_list = 0;

	if (count < priv->core_req_size)
		return -EINVAL;

	add_wait_queue(&priv->core_msg_wq, &wait);
	do {
		__set_current_state(TASK_INTERRUPTIBLE);

		ml = moca_detach_head(priv, &priv->core_msg_pend_list);
		if (ml != NULL) {
			m = list_entry(ml, struct moca_core_msg, chain);
			ret = 0;
			break;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		schedule();
	} while (1);
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&priv->core_msg_wq, &wait);

	if (ret < 0)
		return ret;

	if (copy_to_user(buf, m->data, m->len))
		ret = -EFAULT;	/* beware: message will be dropped */
	else
		ret = m->len;

	spin_lock_bh(&priv->list_lock);
	if (list_empty(&priv->core_msg_free_list))
		empty_free_list = 1;
	list_add_tail(ml, &priv->core_msg_free_list);
	spin_unlock_bh(&priv->list_lock);

	if (empty_free_list) {
		/*
		 * we just freed up space for another message, so if there was
		 * a backlog, clear it out
		 */
		mutex_lock(&priv->dev_mutex);

		if (moca_get_mbx_offset(priv)) {
			mutex_unlock(&priv->dev_mutex);
			return -EIO;
		}

		if (priv->assert_pending & 2) {
			if (moca_recvmsg(priv, priv->core_req_offset,
				priv->core_req_size, 0, 1) != -ENOMEM)
				priv->assert_pending &= ~2;
			else
				dev_warn(priv->dev, "moca_recvmsg assert failed\n");
		}
		if (priv->assert_pending & 1) {
			if (moca_recvmsg(priv, priv->core_req_offset,
				priv->core_req_size, 0, 0) != -ENOMEM)
				priv->assert_pending &= ~1;
			else
				dev_warn(priv->dev, "moca_recvmsg assert failed\n");
		}
		if (priv->wdt_pending)
			if (moca_wdt(priv, priv->wdt_pending) != -ENOMEM)
				priv->wdt_pending = 0;

		if (priv->core_req_pending & 1) {
			if (moca_recvmsg(priv, priv->core_req_offset,
				priv->core_req_size, priv->core_resp_offset, 0)
				!= -ENOMEM)
				priv->core_req_pending &= ~1;
			else
				dev_warn(priv->dev, "moca_recvmsg core_req failed\n");
		}
		if (priv->core_req_pending & 2) {
			if (moca_recvmsg(priv, priv->core_req_offset,
				priv->core_req_size, priv->core_resp_offset, 1)
				!= -ENOMEM)
				priv->core_req_pending &= ~2;
			else
				dev_warn(priv->dev, "moca_recvmsg core_req failed\n");
		}
		if (priv->host_resp_pending & 1) {
			if (moca_recvmsg(priv, priv->host_resp_offset,
				priv->host_resp_size, 0, 0) != -ENOMEM)
				priv->host_resp_pending &= ~1;
			else
				dev_warn(priv->dev, "moca_recvmsg host_resp failed\n");
		}
		if (priv->host_resp_pending & 2) {
			if (moca_recvmsg(priv, priv->host_resp_offset,
				priv->host_resp_size, 0, 1) != -ENOMEM)
				priv->host_resp_pending &= ~2;
			else
				dev_warn(priv->dev, "moca_recvmsg host_resp failed\n");
		}
		mutex_unlock(&priv->dev_mutex);
	}

	return ret;
}

static ssize_t moca_file_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	struct moca_priv_data *priv = file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	struct list_head *ml = NULL;
	struct moca_host_msg *m = NULL;
	ssize_t ret;
	u32 cpuid;

	if (count > priv->host_req_size)
		return -EINVAL;

	add_wait_queue(&priv->host_msg_wq, &wait);
	do {
		__set_current_state(TASK_INTERRUPTIBLE);

		ml = moca_detach_head(priv, &priv->host_msg_free_list);
		if (ml != NULL) {
			m = list_entry(ml, struct moca_host_msg, chain);
			ret = 0;
			break;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		schedule();
	} while (1);
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&priv->host_msg_wq, &wait);

	if (ret < 0)
		return ret;

	m->len = count;

	if (copy_from_user(m->data, buf, m->len)) {
		ret = -EFAULT;
		goto bad;
	}

	ret = moca_h2m_sanity_check(priv, m);
	if (ret < 0) {
		ret = -EINVAL;
		goto bad;
	}

	moca_attach_tail(priv, ml, &priv->host_msg_pend_list);

	if (!priv->mmp_20)
		cpuid = 1;
	else {
		if (cpu_to_be32(m->data[0]) & 0x10)
			cpuid = 0;
		else
			cpuid = 1;
	}
	mutex_lock(&priv->dev_mutex);
	if (priv->running) {
		if (moca_get_mbx_offset(priv))
			ret = -EIO;
		else
			moca_sendmsg(priv, cpuid);
	} else
		ret = -EIO;
	mutex_unlock(&priv->dev_mutex);

	return ret;

bad:
	moca_attach_tail(priv, ml, &priv->host_msg_free_list);

	return ret;
}

static unsigned int moca_file_poll(struct file *file, poll_table *wait)
{
	struct moca_priv_data *priv = file->private_data;
	unsigned int ret = 0;

	poll_wait(file, &priv->core_msg_wq, wait);
	poll_wait(file, &priv->host_msg_wq, wait);

	spin_lock_bh(&priv->list_lock);
	if (!list_empty(&priv->core_msg_pend_list))
		ret |= POLLIN | POLLRDNORM;
	if (!list_empty(&priv->host_msg_free_list))
		ret |= POLLOUT | POLLWRNORM;
	spin_unlock_bh(&priv->list_lock);

	return ret;
}

static const struct file_operations moca_fops = {
	.owner =		THIS_MODULE,
	.open =			moca_file_open,
	.release =		moca_file_release,
	.unlocked_ioctl =	moca_file_ioctl,
	.read =			moca_file_read,
	.write =		moca_file_write,
	.poll =			moca_file_poll,
};

/*
 * PLATFORM DRIVER
 */

static int moca_probe(struct platform_device *pdev)
{
	struct moca_priv_data *priv;
	struct resource *mres, *ires;
	int minor, err;
	struct moca_platform_data *pd = pdev->dev.platform_data;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, priv);
	priv->pdev = pdev;
	priv->start_time = jiffies;

	priv->clk = clk_get(&pdev->dev, "moca");
	priv->cpu_clk = clk_get(&pdev->dev, "moca-cpu");
	priv->phy_clk = clk_get(&pdev->dev, "moca-phy");

	priv->hw_rev = pd->hw_rev;
	if (pd->hw_rev == HWREV_MOCA_11_PLUS)
		priv->regs = &regs_11_plus;
	else if (pd->hw_rev == HWREV_MOCA_11_LITE)
		priv->regs = &regs_11_lite;
	else if (pd->hw_rev == HWREV_MOCA_11)
		priv->regs = &regs_11;
	else if ((pd->hw_rev == HWREV_MOCA_20_GEN21) ||
		(pd->hw_rev == HWREV_MOCA_20_GEN22))
		priv->regs = &regs_20;
	else {
		pr_err("unsupported MoCA HWREV: %x\n", pd->hw_rev);
		err = -EINVAL;
		goto bad;
	}

	init_waitqueue_head(&priv->host_msg_wq);
	init_waitqueue_head(&priv->core_msg_wq);
	init_completion(&priv->copy_complete);
	init_completion(&priv->chunk_complete);

	spin_lock_init(&priv->list_lock);
	spin_lock_init(&priv->clock_lock);
	spin_lock_init(&priv->irq_status_lock);

	mutex_init(&priv->dev_mutex);
	mutex_init(&priv->copy_mutex);
	mutex_init(&priv->moca_i2c_mutex);

	sg_init_table(priv->fw_sg, MAX_FW_PAGES);

	INIT_WORK(&priv->work, moca_work_handler);

	priv->minor = -1;
	for (minor = 0; minor < NUM_MINORS; minor++) {
		if (minor_tbl[minor] == NULL) {
			priv->minor = minor;
			break;
		}
	}

	if (priv->minor == -1) {
		pr_err("can't allocate minor device\n");
		err = -EIO;
		goto bad;
	}

	mres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ires = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!mres || !ires) {
		pr_err("can't get resources\n");
		err = -EIO;
		goto bad;
	}
	priv->base = ioremap(mres->start, mres->end - mres->start + 1);
	priv->irq = ires->start;
	priv->i2c_base = ioremap(pd->bcm3450_i2c_base, sizeof(struct bsc_regs));

	/* leave core in reset until we get an ioctl */

	moca_hw_reset(priv);

	if (request_irq(priv->irq, moca_interrupt, 0, "moca", priv) < 0) {
		pr_err("can't request interrupt\n");
		err = -EIO;
		goto bad2;
	}
	moca_hw_init(priv, MOCA_ENABLE);
	moca_disable_irq(priv);
	moca_msg_reset(priv);
	moca_hw_init(priv, MOCA_DISABLE);

	pr_info("adding minor #%d at base 0x%08llx, IRQ %d, "
		"I2C 0x%08llx/0x%02x\n", priv->minor,
		(unsigned long long)mres->start, ires->start,
		(unsigned long long)pd->bcm3450_i2c_base, pd->bcm3450_i2c_addr);

	minor_tbl[priv->minor] = priv;
	priv->dev = device_create(moca_class, NULL,
		MKDEV(MOCA_MAJOR, priv->minor), NULL, "bmoca%d", priv->minor);
	if (IS_ERR(priv->dev)) {
		pr_warn("can't register class device\n");
		priv->dev = NULL;
	}

	return 0;

bad2:
	if (priv->base)
		iounmap(priv->base);
	if (priv->i2c_base)
		iounmap(priv->i2c_base);
bad:
	kfree(priv);
	return err;
}

static int moca_remove(struct platform_device *pdev)
{
	struct moca_priv_data *priv = dev_get_drvdata(&pdev->dev);
	struct clk *clk = priv->clk;
	struct clk *phy_clk = priv->phy_clk;
	struct clk *cpu_clk = priv->cpu_clk;

	if (priv->dev)
		device_destroy(moca_class, MKDEV(MOCA_MAJOR, priv->minor));
	minor_tbl[priv->minor] = NULL;

	free_irq(priv->irq, priv);
	iounmap(priv->i2c_base);
	iounmap(priv->base);
	kfree(priv);

	clk_put(cpu_clk);
	clk_put(phy_clk);
	clk_put(clk);

	return 0;
}

#ifdef CONFIG_PM
static int moca_suspend(struct device *dev)
{
	/* do not do anything on suspend.
	MoCA core is not necessarily stopped */

	return 0;
}

static int moca_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops moca_pm_ops = {
	.suspend		= moca_suspend,
	.resume			= moca_resume,
};

#endif

static struct platform_driver moca_plat_drv = {
	.probe =		moca_probe,
	.remove =		moca_remove,
	.driver = {
		.name =		"bmoca",
		.owner =	THIS_MODULE,
#ifdef CONFIG_PM
		.pm =		&moca_pm_ops,
#endif
	},
};

static int moca_init(void)
{
	int ret;
	memset(minor_tbl, 0, sizeof(minor_tbl));
	ret = register_chrdev(MOCA_MAJOR, MOCA_CLASS, &moca_fops);
	if (ret < 0) {
		pr_err("can't register major %d\n", MOCA_MAJOR);
		goto bad;
	}

	moca_class = class_create(THIS_MODULE, MOCA_CLASS);
	if (IS_ERR(moca_class)) {
		pr_err("can't create device class\n");
		ret = PTR_ERR(moca_class);
		goto bad2;
	}

	ret = platform_driver_register(&moca_plat_drv);
	if (ret < 0) {
		pr_err("can't register platform_driver\n");
		goto bad3;
	}

	return 0;

bad3:
	class_destroy(moca_class);
bad2:
	unregister_chrdev(MOCA_MAJOR, MOCA_CLASS);
bad:
	return ret;
}

static void moca_exit(void)
{
	class_destroy(moca_class);
	unregister_chrdev(MOCA_MAJOR, MOCA_CLASS);
	platform_driver_unregister(&moca_plat_drv);

}

module_init(moca_init);
module_exit(moca_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("MoCA messaging driver");

