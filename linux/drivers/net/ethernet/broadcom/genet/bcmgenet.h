/*
 *
 * Copyright (c) 2002-2008 Broadcom Corporation
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
 *
 *
*/
#ifndef __BCMGENET_H__
#define __BCMGENET_H__

#define CARDNAME				"bcmgenet"
#define VERSION     "2.0"
#define VER_STR     "v" VERSION " " __DATE__ " " __TIME__

#define pr_fmt(fmt)				CARDNAME ": " fmt

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/mii.h>

#include "bcmgenet_map.h"

/* total number of Buffer Descriptors, same for Rx/Tx */
#define TOTAL_DESC				256
/* which ring is descriptor based */
#define DESC_INDEX				16
/* Body(1500) + EH_SIZE(14) + VLANTAG(4) + BRCMTAG(6) + FCS(4) = 1528.
 * 1536 is multiple of 256 bytes
 */
#define ENET_MAX_MTU_SIZE       1536
#define DMA_MAX_BURST_LENGTH    0x10

#define GENET_TX_RING_COUNT		16
#define GENET_RX_RING_COUNT		16
#define GENET_ALLOC_TX_RING		1
#define GENET_ALLOC_RX_RING		0

#define ETH_CRC_LEN             4

/* misc. configuration */
#define CLEAR_ALL_HFB			0xFF
#define DMA_FC_THRESH_HI		(TOTAL_DESC >> 4)
#define DMA_FC_THRESH_LO		5


struct Enet_CB {
	struct sk_buff      *skb;
	volatile struct DmaDesc    *BdAddr;
	DEFINE_DMA_UNMAP_ADDR(dma_addr);
	DEFINE_DMA_UNMAP_LEN(dma_len);
};

/* power management mode */
#define GENET_POWER_CABLE_SENSE	0
#define GENET_POWER_WOL_MAGIC	1
#define GENET_POWER_WOL_ACPI	2
#define GENET_POWER_PASSIVE		3

/*
 * device context
 */
struct BcmEnet_devctrl {
	struct net_device *dev;	/* ptr to net_device */
	struct net_device *next_dev;
	spinlock_t      lock;		/* Serializing lock */
	spinlock_t		bh_lock;	/* soft IRQ lock */
	struct mii_if_info mii;		/* mii interface info */
	struct mutex mdio_mutex;	/* mutex for mii_read/write */
	wait_queue_head_t	wq;		/* mii wait queue */
	struct timer_list timer;	/* Link status timer */

	struct napi_struct napi;	/* NAPI for descriptor based rx */
	struct napi_struct ring_napi;	/* NAPI for ring buffer */
	unsigned long base_addr;	/* GENET register start address. */
	volatile struct uniMacRegs *umac;	/* UNIMAC register */
	volatile struct rbufRegs	*rbuf;	/* rbuf registers */
	volatile struct intrl2Regs *intrl2_0;	/* INTR2_0 registers */
	volatile struct intrl2Regs *intrl2_1;	/* INTR2_1 registers */
	volatile struct SysRegs    *sys;	/* sys register */
	volatile struct GrBridgeRegs *grb;	/* Grb */
	volatile struct ExtRegs    *ext;	/* Extention register */
	volatile unsigned long *hfb;/* HFB registers */
#if CONFIG_BRCM_GENET_VERSION > 1
	volatile struct tbufRegs *tbuf;	/* tbuf register for GENET2 */
	volatile struct hfbRegs  *hfbReg;	/* hfb register for GENET2 */
#endif

	/* transmit variables */
	volatile struct tDmaRegs *txDma;/* location of tx Dma register */
	volatile struct DmaDesc *txBds;	/* location of tx Dma BD ring */
	struct Enet_CB *txCbs;	/* locaation of tx control block pool */
	int	nrTxBds;		/* number of transmit bds */
	int	txFreeBds;		/* # of free transmit bds */
	int	txLastCIndex;	/* consumer index for the last xmit call */

	struct Enet_CB *txRingCBs[16];	/* tx ring buffer control block*/
	unsigned int txRingSize[16];	/* size of each tx ring */
	unsigned int txRingCIndex[16];	/* last consumer index of each ring*/
	int txRingFreeBds[16];	/* # of free bds for each ring */
	unsigned char *txRingStart[16];	/* tx ring start addr */

	/* receive variables */
	volatile struct rDmaRegs *rxDma;	/* location of rx dma  */
	volatile struct DmaDesc *rxBds;		/* location of rx bd ring */
	volatile struct DmaDesc *rxBdAssignPtr;	/*next rx bd to assign buffer*/
	struct Enet_CB *rxCbs;	/* location of rx control block pool */
	int	nrRxBds;	/* number of receive bds */
	int	rxBufLen;	/* size of rx buffers for DMA */

	struct Enet_CB *rxRingCbs[16];	/* rx ring buffer control */
	unsigned int rxRingSize[16];	/* size of each ring */
	unsigned int rxRingCIndex[16];	/* consumer index for each ring */
	unsigned int rxRingDiscCnt[16];	/* # of discarded pkt for each ring */
	unsigned char *rxRingStart[16];	/* ring buffer start addr.*/

	/* other misc variables */
	int irq0;	/* regular IRQ */
	int	irq1;	/* ring buf IRQ */
	int phyAddr;
	int	phyType;
	int	phySpeed;
	int	extPhy;
	int	bIPHdrOptimize;
	unsigned int irq0_stat;	/* sw copy of irq0 status, for IRQ BH */
	unsigned int irq1_stat;	/* sw copy of irq1 status, for NAPI rx */

	struct work_struct bcmgenet_irq_work;	/* bottom half work */
	struct work_struct bcmgenet_link_work;	/* GPHY link status work */
	int    devnum;		/* 0=EMAC_0, 1=EMAC_1 */
	struct clk *clk;
	int dev_opened;		/* device opened. */
	int dev_asleep;		/* device is at sleep */
	struct platform_device *pdev;

	/* WOL */
	unsigned long	wol_enabled;
	struct	clk *clk_wol;
	int	clock_active;
	u32	wolopts;

	/* S3 warm boot */
	struct DmaDesc saved_rx_desc[TOTAL_DESC];
	u32 int_mask;
	u32 rbuf_ctrl;
};

#if defined(CONFIG_BCMGENET_DUMP_TRACE)
#define TRACE(x)        (printk x)
#else
#define TRACE(x)
#endif

/* These macros are defined to deal with register map change
 * between GENET1.1 and GENET2. Only those currently being used
 * by driver are defined.
 */
#if CONFIG_BRCM_GENET_VERSION > 1

#define GENET_TBUF_CTRL(pdev)			(pdev->tbuf->tbuf_ctrl)
#define GENET_TBUF_BP_MC(pdev)			(pdev->tbuf->tbuf_bp_mc)
#define GENET_TBUF_ENDIAN_CTRL(pdev)	(pdev->tbuf->tbuf_endian_ctrl)
#define GENET_TBUF_FLUSH_CTRL(pdev)		(pdev->sys->tbuf_flush_ctrl)
#define GENET_RBUF_FLUSH_CTRL(pdev)		(pdev->sys->rbuf_flush_ctrl)
#define GENET_RGMII_OOB_CTRL(pdev)		(pdev->ext->rgmii_oob_ctrl)
#define GENET_RGMII_IB_STATUS(pdev)		(pdev->ext->rgmii_ib_status)
#define GENET_RGMII_LED_STATUS(pdev)	(pdev->ext->rgmii_led_ctrl)
#define GENET_HFB_CTRL(pdev)			(pdev->hfbReg->hfb_ctrl)
#define GENET_HFB_FLTR_LEN(pdev, i)		(pdev->hfbReg->hfb_fltr_len[i])

#else

#define GENET_TBUF_CTRL(pdev)			(pdev->rbuf->tbuf_ctrl)
#define GENET_TBUF_BP_MC(pdev)			(pdev->rbuf->tbuf_bp_mc)
#define GENET_TBUF_ENDIAN_CTRL(pdev)		(pdev->rbuf->tbuf_endian_ctrl)
#define GENET_TBUF_FLUSH_CTRL(pdev)		(pdev->rbuf->tbuf_flush_ctrl)
#define GENET_RBUF_FLUSH_CTRL(pdev)		(pdev->rbuf->rbuf_flush_ctrl)
#define GENET_RGMII_OOB_CTRL(pdev)		(pdev->rbuf->rgmii_oob_ctrl)
#define GENET_RGMII_IB_STATUS(pdev)		(pdev->rbuf->rgmii_ib_status)
#define GENET_RGMII_LED_STATUS(pdev)		(pdev->rbuf->rgmii_led_ctrl)
#define GENET_HFB_CTRL(pdev)			(pdev->rbuf->rbuf_hfb_ctrl)
#define GENET_HFB_FLTR_LEN(pdev, i)		(pdev->rbuf->rbuf_fltr_len[i])

#endif /* CONFIG_BRCM_GENET_VERSION > 1 */

#if CONFIG_BRCM_GENET_VERSION < 3
#define GENET_BP_IN_EN_SHIFT			16
#define GENET_BP_MASK				0xffff
#else
#define GENET_BP_IN_EN_SHIFT			17
#define GENET_BP_MASK				0x1ffff
#endif

#endif /* __BCMGENET_H__ */
