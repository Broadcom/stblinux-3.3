/*
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
 * File Name  : bcmgenet.c
 *
 * Description: This is Linux driver for the broadcom GENET ethernet MAC core.

*/

#include "bcmgenet.h"
#include "bcmmii.h"
#include "if_net.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>

#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/in.h>

#include <asm/unaligned.h>

#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/brcmapi.h>

#ifdef CONFIG_NET_SCH_MULTIQ

#if CONFIG_BRCM_GENET_VERSION == 1
#error "This version of GENET doesn't support tx multi queue"
#endif
/* Default # of tx queues for multi queue support */
#define GENET_MQ_CNT		4
/* Default # of bds for each queue for multi queue support */
#define GENET_MQ_BD_CNT		32
/* Default highest priority queue for multi queue support */
#define GENET_Q0_PRIORITY	0

#define GENET_DEFAULT_BD_CNT	\
	(TOTAL_DESC - GENET_MQ_CNT * GENET_MQ_BD_CNT)

static void bcmgenet_init_multiq(struct net_device *dev);

#endif	/*CONFIG_NET_SCH_MULTIQ */

#define RX_BUF_LENGTH		2048
#define RX_BUF_BITS			12
#define SKB_ALIGNMENT		32
#define DMA_DESC_THRES		4
#define HFB_TCP_LEN			19
#define HFB_ARP_LEN			21

/* Tx/Rx DMA register offset, skip 256 descriptors */
#define GENET_TDMA_REG_OFF	(GENET_TDMA_OFF + \
		TOTAL_DESC*sizeof(struct DmaDesc))
#define GENET_RDMA_REG_OFF	(GENET_RDMA_OFF + \
		TOTAL_DESC*sizeof(struct DmaDesc))

/* --------------------------------------------------------------------------
External, indirect entry points.
--------------------------------------------------------------------------*/
static int bcmgenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
/* --------------------------------------------------------------------------
Called for "ifconfig ethX up" & "down"
--------------------------------------------------------------------------*/
static int bcmgenet_open(struct net_device *dev);
static int bcmgenet_close(struct net_device *dev);
/* --------------------------------------------------------------------------
Watchdog timeout
--------------------------------------------------------------------------*/
static void bcmgenet_timeout(struct net_device *dev);
/* --------------------------------------------------------------------------
Packet transmission.
--------------------------------------------------------------------------*/
static int bcmgenet_xmit(struct sk_buff *skb, struct net_device *dev);
/* --------------------------------------------------------------------------
Set address filtering mode
--------------------------------------------------------------------------*/
static void bcmgenet_set_rx_mode(struct net_device *dev);
/* --------------------------------------------------------------------------
Set the hardware MAC address.
--------------------------------------------------------------------------*/
static int bcmgenet_set_mac_addr(struct net_device *dev, void *p);

/* --------------------------------------------------------------------------
Interrupt routine, for all interrupts except ring buffer interrupts
--------------------------------------------------------------------------*/
static irqreturn_t bcmgenet_isr0(int irq, void *dev_id);
/*---------------------------------------------------------------------------
IRQ handler for ring buffer interrupt.
--------------------------------------------------------------------------*/
static irqreturn_t bcmgenet_isr1(int irq, void *dev_id);
/* --------------------------------------------------------------------------
dev->poll() method
--------------------------------------------------------------------------*/
static int bcmgenet_poll(struct napi_struct *napi, int budget);
/* --------------------------------------------------------------------------
Process recived packet for descriptor based DMA
--------------------------------------------------------------------------*/
static unsigned int bcmgenet_desc_rx(void *ptr, unsigned int budget);
/* --------------------------------------------------------------------------
Internal routines
--------------------------------------------------------------------------*/
/* Allocate and initialize tx/rx buffer descriptor pools */
static int bcmgenet_init_dev(struct BcmEnet_devctrl *pDevCtrl);
static void bcmgenet_uninit_dev(struct BcmEnet_devctrl *pDevCtrl);
/* Assign the Rx descriptor ring */
static int assign_rx_buffers(struct BcmEnet_devctrl *pDevCtrl);
/* Initialize the uniMac control registers */
static int init_umac(struct BcmEnet_devctrl *pDevCtrl);
/* Initialize DMA control register */
static void init_edma(struct BcmEnet_devctrl *pDevCtrl);
/* Interrupt bottom-half */
static void bcmgenet_irq_task(struct work_struct *work);
/* power management */
static void bcmgenet_power_down(struct BcmEnet_devctrl *pDevCtrl, int mode);
static void bcmgenet_power_up(struct BcmEnet_devctrl *pDevCtrl, int mode);
/* clock control */
static void bcmgenet_clock_enable(struct BcmEnet_devctrl *pDevCtrl);
static void bcmgenet_clock_disable(struct BcmEnet_devctrl *pDevCtrl);
/* S3 warm boot */
static void save_state(struct BcmEnet_devctrl *pDevCtrl);
static void restore_state(struct BcmEnet_devctrl *pDevCtrl);

static struct net_device *eth_root_dev;
static int DmaDescThres = DMA_DESC_THRES;

/*
 * HFB data for ARP request.
 * In WoL (Magic Packet or ACPI) mode, we need to response
 * ARP request, so dedicate an HFB to filter the ARP request.
 * NOTE: the last two words are to be filled by destination.
 */
static unsigned int hfb_arp[] = {
	0x000FFFFF, 0x000FFFFF, 0x000FFFFF,	0x00000000,
	0x00000000, 0x00000000, 0x000F0806,	0x000F0001,
	0x000F0800,	0x000F0604, 0x000F0001,	0x00000000,
	0x00000000,	0x00000000,	0x00000000, 0x00000000,
	0x000F0000,	0x000F0000,	0x000F0000,	0x000F0000,
	0x000F0000
};

static inline void handleAlignment(struct BcmEnet_devctrl *pDevCtrl,
		struct sk_buff *skb)
{
	/*
	 * We request to allocate 2048 + 32 bytes of buffers, and the
	 * dev_alloc_skb() added 16B for NET_SKB_PAD, so we totally
	 * requested 2048+32+16 bytes buffer, the size was aligned to
	 * SMP_CACHE_BYTES, which is 64B.(is it?), so we finnally ended
	 * up got 2112 bytes of buffer! Among which, the first 16B is
	 * reserved for NET_SKB_PAD, to make the skb->data aligned 32B
	 * boundary, we should have enough space to fullfill the 2KB
	 * buffer after alignment!
     */

	unsigned long boundary32, curData, resHeader;

	curData = (unsigned long) skb->data;
	boundary32 = (curData + (SKB_ALIGNMENT - 1)) & ~(SKB_ALIGNMENT - 1);
	resHeader = boundary32 - curData ;
	/* 4 bytes for skb pointer */
	if (resHeader < 4)
		boundary32 += 32;

	resHeader = boundary32 - curData - 4;
	/* We'd have minimum 16B reserved by default. */
	skb_reserve(skb, resHeader);

	*(unsigned int *)skb->data = (unsigned int)skb;
	skb_reserve(skb, 4);
	/*
	 * Make sure it is on 32B boundary, should never happen if our
	 * calculation was correct.
	 */
	if ((unsigned long) skb->data & (SKB_ALIGNMENT - 1)) {
		printk(KERN_WARNING "skb buffer is NOT aligned on %d boundary!\n",
			SKB_ALIGNMENT);
	}

	/*
	 *  we don't reserve 2B for IP Header optimization here,
	 *  use skb_pull when receiving packets
	 */
}
/* --------------------------------------------------------------------------
Name: bcmgenet_gphy_link_status
Purpose: GPHY link status monitoring task
-------------------------------------------------------------------------- */
static void bcmgenet_gphy_link_status(struct work_struct *work)
{
	struct BcmEnet_devctrl *pDevCtrl = container_of(work,
			struct BcmEnet_devctrl, bcmgenet_link_work);

	bcmgenet_mii_setup(pDevCtrl->dev);
}
/* --------------------------------------------------------------------------
Name: bcmgenet_gphy_link_timer
Purpose: GPHY link status monitoring timer function
-------------------------------------------------------------------------- */
static void bcmgenet_gphy_link_timer(unsigned long data)
{
	struct BcmEnet_devctrl *pDevCtrl = (struct BcmEnet_devctrl *)data;
	schedule_work(&pDevCtrl->bcmgenet_link_work);
	mod_timer(&pDevCtrl->timer, jiffies + HZ);
}

#ifdef CONFIG_BRCM_HAS_STANDBY
static int bcmgenet_wakeup_enable(void *ref)
{
	struct BcmEnet_devctrl *pDevCtrl = (struct BcmEnet_devctrl *)ref;
	u32 mask;
	if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA)
		mask = WOL_MOCA_MASK;
	else
		mask = pDevCtrl->devnum ? WOL_MOCA_MASK : WOL_ENET_MASK;
	if (device_may_wakeup(&pDevCtrl->dev->dev))
		brcm_pm_wakeup_source_enable(mask, 1);
	return 0;
}

static int bcmgenet_wakeup_disable(void *ref)
{
	struct BcmEnet_devctrl *pDevCtrl = (struct BcmEnet_devctrl *)ref;
	u32 mask;
	if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA)
		mask = WOL_MOCA_MASK;
	else
		mask = pDevCtrl->devnum ? WOL_MOCA_MASK : WOL_ENET_MASK;
	if (device_may_wakeup(&pDevCtrl->dev->dev))
		brcm_pm_wakeup_source_enable(mask, 0);
	return 0;
}

static int bcmgenet_wakeup_poll(void *ref)
{
	struct BcmEnet_devctrl *pDevCtrl = (struct BcmEnet_devctrl *)ref;
	int retval = 0;
	u32 mask = 0;

	if (device_may_wakeup(&pDevCtrl->dev->dev)) {
		if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA)
			mask = WOL_MOCA_MASK;
		else
			mask = pDevCtrl->devnum ? WOL_MOCA_MASK : WOL_ENET_MASK;
		retval = brcm_pm_wakeup_get_status(mask);
	}
	printk(KERN_DEBUG "%s %s(%08x): %d\n", __func__,
	       pDevCtrl->dev->name, mask, retval);
	return retval;
}

static struct brcm_wakeup_ops bcmgenet_wakeup_ops = {
	.enable = bcmgenet_wakeup_enable,
	.disable = bcmgenet_wakeup_disable,
	.poll = bcmgenet_wakeup_poll,
};
#endif

/* --------------------------------------------------------------------------
Name: bcmgenet_open
Purpose: Open and Initialize the EMAC on the chip
-------------------------------------------------------------------------- */
static int bcmgenet_open(struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	unsigned long dma_ctrl;
	volatile struct uniMacRegs *umac = pDevCtrl->umac;

	TRACE(("%s: bcmgenet_open\n", dev->name));

	bcmgenet_clock_enable(pDevCtrl);

	/* take MAC out of reset */
	GENET_RBUF_FLUSH_CTRL(pDevCtrl) &= ~BIT(1);
	udelay(10);

	/* disable ethernet MAC while updating its registers */
	umac->cmd &= ~(CMD_TX_EN | CMD_RX_EN);

	umac->mac_0 = (dev->dev_addr[0] << 24 |
			dev->dev_addr[1] << 16 |
			dev->dev_addr[2] << 8  |
			dev->dev_addr[3]);
	umac->mac_1 = dev->dev_addr[4] << 8 | dev->dev_addr[5];

	if (pDevCtrl->wol_enabled) {
		/* From WOL-enabled suspend, switch to regular clock */
		clk_disable(pDevCtrl->clk_wol);
		/* init umac registers to synchronize s/w with h/w */
		init_umac(pDevCtrl);
		/* Speed settings must be restored */
		bcmgenet_mii_init(dev);
		bcmgenet_mii_setup(dev);
	}

	if (pDevCtrl->phyType == BRCM_PHY_TYPE_INT)
		pDevCtrl->ext->ext_pwr_mgmt |= EXT_ENERGY_DET_MASK;

	if (test_and_clear_bit(GENET_POWER_WOL_MAGIC, &pDevCtrl->wol_enabled))
		bcmgenet_power_up(pDevCtrl, GENET_POWER_WOL_MAGIC);
	if (test_and_clear_bit(GENET_POWER_WOL_ACPI, &pDevCtrl->wol_enabled))
		bcmgenet_power_up(pDevCtrl, GENET_POWER_WOL_ACPI);

	/* disable DMA */
	dma_ctrl = 1 << (DESC_INDEX + DMA_RING_BUF_EN_SHIFT) | DMA_EN;
	pDevCtrl->txDma->tdma_ctrl &= ~dma_ctrl;
	pDevCtrl->rxDma->rdma_ctrl &= ~dma_ctrl;
	umac->tx_flush = 1;
	udelay(10);
	umac->tx_flush = 0;

	/* reset dma, start from beginning of the ring. */
	init_edma(pDevCtrl);
	/* reset internal book keeping variables. */
	pDevCtrl->txLastCIndex = 0;
	pDevCtrl->rxBdAssignPtr = pDevCtrl->rxBds;

	if (brcm_pm_deep_sleep())
		restore_state(pDevCtrl);
	else
		assign_rx_buffers(pDevCtrl);

	pDevCtrl->txFreeBds = pDevCtrl->nrTxBds;

	/*Always enable ring 16 - descriptor ring */
	pDevCtrl->rxDma->rdma_ctrl |= dma_ctrl;
	pDevCtrl->txDma->tdma_ctrl |= dma_ctrl;

	if (pDevCtrl->extPhy)
		mod_timer(&pDevCtrl->timer, jiffies);

	if (request_irq(pDevCtrl->irq0, bcmgenet_isr0, IRQF_SHARED,
				dev->name, pDevCtrl) < 0) {
		printk(KERN_ERR "can't request IRQ %d\n", pDevCtrl->irq0);
		goto err2;
	}
	if (request_irq(pDevCtrl->irq1, bcmgenet_isr1, IRQF_SHARED,
				dev->name, pDevCtrl) < 0) {
		printk(KERN_ERR "can't request IRQ %d\n", pDevCtrl->irq1);
		free_irq(pDevCtrl->irq0, pDevCtrl);
		goto err1;
	}
	/* Start the network engine */
	netif_tx_start_all_queues(dev);
	napi_enable(&pDevCtrl->napi);

	umac->cmd |= (CMD_TX_EN | CMD_RX_EN);

#ifdef CONFIG_BRCM_HAS_STANDBY
	brcm_pm_wakeup_register(&bcmgenet_wakeup_ops, pDevCtrl, dev->name);
	device_set_wakeup_capable(&dev->dev, 1);
#endif

	if (pDevCtrl->phyType == BRCM_PHY_TYPE_INT)
		bcmgenet_power_up(pDevCtrl, GENET_POWER_PASSIVE);

	return 0;
err1:
	free_irq(pDevCtrl->irq0, dev);
err2:
	free_irq(pDevCtrl->irq1, dev);
	del_timer_sync(&pDevCtrl->timer);
	netif_tx_stop_all_queues(dev);

	return -ENODEV;
}
/* --------------------------------------------------------------------------
Name: bcmgenet_close
Purpose: Stop communicating with the outside world
Note: Caused by 'ifconfig ethX down'
-------------------------------------------------------------------------- */
static int bcmgenet_close(struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	int timeout = 0;

	TRACE(("%s: bcmgenet_close\n", dev->name));

	/* Disable MAC receive */
	pDevCtrl->umac->cmd &= ~CMD_RX_EN;

	netif_tx_stop_all_queues(dev);

	/* Disable TDMA to stop add more frames in TX DMA */
	pDevCtrl->txDma->tdma_ctrl &= ~DMA_EN;
	/* Check TDMA status register to confirm TDMA is disabled */
	while (!(pDevCtrl->txDma->tdma_status & DMA_DISABLED)) {
		if (timeout++ == 5000) {
			netdev_warn(pDevCtrl->dev,
				"Timed out while disabling TX DMA\n");
			break;
		}
		udelay(1);
	}

	/* SWLINUX-2252: Workaround for rx flush issue causes rbuf overflow */
	/* Wait 10ms for packet drain in both tx and rx dma */
	usleep_range(10000, 20000);

	/* Disable RDMA */
	pDevCtrl->rxDma->rdma_ctrl &= ~DMA_EN;
	/* Check RDMA status register to confirm RDMA is disabled */
	while (!(pDevCtrl->rxDma->rdma_status & DMA_DISABLED)) {
		if (timeout++ == 5000) {
			netdev_warn(pDevCtrl->dev,
				"Timed out while disabling RX DMA\n");
			break;
		}
		udelay(1);
	}

	/* Disable MAC transmit. TX DMA disabled have to done before this */
	pDevCtrl->umac->cmd &= ~CMD_TX_EN;

	napi_disable(&pDevCtrl->napi);

	/* tx reclaim */
	bcmgenet_xmit(NULL, dev);
	free_irq(pDevCtrl->irq0, (void *)pDevCtrl);
	free_irq(pDevCtrl->irq1, (void *)pDevCtrl);
	if (pDevCtrl->extPhy) {
		del_timer_sync(&pDevCtrl->timer);
		cancel_work_sync(&pDevCtrl->bcmgenet_link_work);
	}
	/*
	 * Wait for pending work items to complete - we are stopping
	 * the clock now. Since interrupts are disabled, no new work
	 * will be scheduled.
	 */
	cancel_work_sync(&pDevCtrl->bcmgenet_irq_work);

	if (brcm_pm_deep_sleep())
		save_state(pDevCtrl);

	if (device_may_wakeup(&dev->dev) && pDevCtrl->dev_asleep) {
		if (pDevCtrl->wolopts & (WAKE_MAGIC|WAKE_MAGICSECURE))
			bcmgenet_power_down(pDevCtrl, GENET_POWER_WOL_MAGIC);
		if (pDevCtrl->wolopts & WAKE_ARP)
			bcmgenet_power_down(pDevCtrl, GENET_POWER_WOL_ACPI);
	} else if (pDevCtrl->phyType == BRCM_PHY_TYPE_INT)
		bcmgenet_power_down(pDevCtrl, GENET_POWER_PASSIVE);

	if (pDevCtrl->wol_enabled)
		clk_enable(pDevCtrl->clk_wol);

	bcmgenet_clock_disable(pDevCtrl);

	return 0;
}

/* --------------------------------------------------------------------------
Name: bcmgenet_net_timeout
Purpose:
-------------------------------------------------------------------------- */
static void bcmgenet_timeout(struct net_device *dev)
{
	BUG_ON(dev == NULL);

	TRACE(("%s: bcmgenet_timeout\n", dev->name));

	dev->trans_start = jiffies;

	dev->stats.tx_errors++;

	netif_tx_wake_all_queues(dev);
}

/* --------------------------------------------------------------------------
Name: bcmgenet_set_rx_mode
Purpose: ndo_set_rx_mode entry point, called when unicast or multicast
address list, or network interface flags are updated.
-------------------------------------------------------------------------- */
static void bcmgenet_set_rx_mode(struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	int i, mc;
#define MAX_MC_COUNT	16

	TRACE(("%s: %s: %08X\n",
		dev->name, __func__, dev->flags));

	/* Promiscous mode */
	if (dev->flags & IFF_PROMISC) {
		pDevCtrl->umac->cmd |= CMD_PROMISC;
		pDevCtrl->umac->mdf_ctrl = 0;
		return;
	} else
		pDevCtrl->umac->cmd &= ~CMD_PROMISC;


	/* UniMac doesn't support ALLMULTI */
	if (dev->flags & IFF_ALLMULTI)
		return;

	/* update MDF filter */
	i = 0;
	mc = 0;
	/* Broadcast */
	pDevCtrl->umac->mdf_addr[i] = 0xFFFF;
	pDevCtrl->umac->mdf_addr[i+1] = 0xFFFFFFFF;
	pDevCtrl->umac->mdf_ctrl |= (1 << (MAX_MC_COUNT - mc));
	i += 2;
	mc++;
	/* my own address.*/
	pDevCtrl->umac->mdf_addr[i] = (dev->dev_addr[0]<<8) | dev->dev_addr[1];
	pDevCtrl->umac->mdf_addr[i+1] = dev->dev_addr[2] << 24 |
		dev->dev_addr[3] << 16 |
		dev->dev_addr[4] << 8 |
		dev->dev_addr[5];
	pDevCtrl->umac->mdf_ctrl |= (1 << (MAX_MC_COUNT - mc));
	i += 2;
	mc++;
	/* Unicast list*/
	if (netdev_uc_count(dev) > (MAX_MC_COUNT - mc))
		return;

	if (!netdev_uc_empty(dev)) {
		netdev_for_each_uc_addr(ha, dev) {
			pDevCtrl->umac->mdf_addr[i] = (ha->addr[0]<<8) |
				ha->addr[1];
			pDevCtrl->umac->mdf_addr[i+1] = ha->addr[2] << 24 |
				ha->addr[3] << 16 |
				ha->addr[4] << 8 |
				ha->addr[5];
			pDevCtrl->umac->mdf_ctrl |=
				(1 << (MAX_MC_COUNT - mc));
			i += 2;
			mc++;
		}
	}
	/* Multicast */
	if (netdev_mc_empty(dev) || netdev_mc_count(dev) >= (MAX_MC_COUNT - mc))
		return;

	netdev_for_each_mc_addr(ha, dev) {
		pDevCtrl->umac->mdf_addr[i] = ha->addr[0] << 8 |
			ha->addr[1];
		pDevCtrl->umac->mdf_addr[i+1] = ha->addr[2] << 24 |
			ha->addr[3] << 16 | ha->addr[4] << 8 | ha->addr[5];
		pDevCtrl->umac->mdf_ctrl |= (1 << (MAX_MC_COUNT - mc));
		i += 2;
		mc++;
	}
}
/*
 * Set the hardware MAC address.
 */
static int bcmgenet_set_mac_addr(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	return 0;
}
/* --------------------------------------------------------------------------
Name: bcmgenet_select_queue
Purpose: select which xmit queue to use based on skb->queue_mapping.
-------------------------------------------------------------------------- */
static u16 __maybe_unused bcmgenet_select_queue(struct net_device *dev,
		struct sk_buff *skb)
{
	/*
	 * If multi-queue support is enabled, and NET_ACT_SKBEDIT is not
	 * defined, this function simply returns current queue_mapping set
	 * inside skb, this means other modules, (netaccel, for example),
	 * must provide a mechanism to set the queue_mapping before trying
	 * to send a packet.
	 */
	return skb->queue_mapping;
}

/* --------------------------------------------------------------------------
Name: bcmgenet_get_txcb
Purpose: return tx control data and increment write pointer.
-------------------------------------------------------------------------- */
static struct Enet_CB *bcmgenet_get_txcb(struct net_device *dev,
		int *pos, int index)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	struct Enet_CB *txCBPtr = NULL;
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	if (index == DESC_INDEX) {
		txCBPtr = pDevCtrl->txCbs;
		txCBPtr += (*pos - GENET_MQ_CNT*GENET_MQ_BD_CNT);
		txCBPtr->BdAddr = &pDevCtrl->txBds[*pos];
		if (*pos == (TOTAL_DESC - 1))
			*pos = (GENET_MQ_CNT*GENET_MQ_BD_CNT);
		else
			*pos += 1;

	} else {
		txCBPtr = pDevCtrl->txRingCBs[index];
		txCBPtr += (*pos - index*GENET_MQ_BD_CNT);
		txCBPtr->BdAddr = &pDevCtrl->txBds[*pos];
		if (*pos == (GENET_MQ_BD_CNT*(index+1) - 1))
			*pos = GENET_MQ_BD_CNT * index;
		else
			*pos += 1;
	}
#else
	txCBPtr = pDevCtrl->txCbs + *pos;
	txCBPtr->BdAddr = &pDevCtrl->txBds[*pos];
	/* Advancing local write pointer */
	if (*pos == (TOTAL_DESC - 1))
		*pos = 0;
	else
		*pos += 1;
#endif

	return txCBPtr;
}
/* --------------------------------------------------------------------------
Name: bcmgenet_tx_reclaim
Purpose: reclaim xmited skb
-------------------------------------------------------------------------- */
static void bcmgenet_tx_reclaim(struct net_device *dev, int index)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	unsigned int c_index;
	struct Enet_CB *txCBPtr;
	int lastTxedCnt = 0, lastCIndex = 0, nrTxBds = 0;

	/* Compute how many buffers are transmited since last xmit call */
	c_index = pDevCtrl->txDma->tDmaRings[index].tdma_consumer_index;


#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	if (index == DESC_INDEX) {
		lastCIndex = pDevCtrl->txLastCIndex;
		nrTxBds = GENET_DEFAULT_BD_CNT;
	} else {
		lastCIndex = pDevCtrl->txRingCIndex[index];
		nrTxBds = GENET_MQ_BD_CNT;
	}

#else
	lastCIndex = pDevCtrl->txLastCIndex;
	nrTxBds = TOTAL_DESC;
#endif
	c_index &= (nrTxBds - 1);

	if (c_index >= lastCIndex)
		lastTxedCnt = c_index - lastCIndex;
	else
		lastTxedCnt = nrTxBds - lastCIndex + c_index;


	TRACE(("%s: %s index=%d c_index=%d "
			"lastTxedCnt=%d txLastCIndex=%d\n",
			__func__, pDevCtrl->dev->name, index,
			c_index, lastTxedCnt, lastCIndex));

	/* Reclaim transmitted buffers */
	while (lastTxedCnt-- > 0) {
		if (index == DESC_INDEX)
			txCBPtr = &pDevCtrl->txCbs[lastCIndex];
		else
			txCBPtr = pDevCtrl->txRingCBs[index] + lastCIndex;
		if (txCBPtr->skb != NULL) {
			dma_unmap_single(&pDevCtrl->dev->dev,
					dma_unmap_addr(txCBPtr, dma_addr),
					txCBPtr->skb->len,
					DMA_TO_DEVICE);
			dev_kfree_skb_any(txCBPtr->skb);
			txCBPtr->skb = NULL;
			dma_unmap_addr_set(txCBPtr, dma_addr, 0);
		} else if (dma_unmap_addr(txCBPtr, dma_addr)) {
			dma_unmap_page(&pDevCtrl->dev->dev,
					dma_unmap_addr(txCBPtr, dma_addr),
					dma_unmap_len(txCBPtr, dma_len),
					DMA_TO_DEVICE);
			dma_unmap_addr_set(txCBPtr, dma_addr, 0);
		}
		if (index == DESC_INDEX)
			pDevCtrl->txFreeBds += 1;
		else
			pDevCtrl->txRingFreeBds[index] += 1;

		if (lastCIndex == (nrTxBds - 1))
			lastCIndex = 0;
		else
			lastCIndex++;
	}
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	if (index == DESC_INDEX) {
		if (pDevCtrl->txFreeBds > (MAX_SKB_FRAGS + 1)
			&& __netif_subqueue_stopped(dev, 0)) {
			pDevCtrl->intrl2_0->cpu_mask_set |=
				(UMAC_IRQ_TXDMA_BDONE | UMAC_IRQ_TXDMA_PDONE);
			netif_wake_subqueue(dev, 0);
		}
		pDevCtrl->txLastCIndex = c_index;
	} else{
		if (pDevCtrl->txRingFreeBds[index] > (MAX_SKB_FRAGS + 1)
			&& __netif_subqueue_stopped(dev, index+1)) {
			pDevCtrl->intrl2_1->cpu_mask_set = (1 << index);
			netif_wake_subqueue(dev, index+1);
		}
		pDevCtrl->txRingCIndex[index] = c_index;
	}
#else
	if (pDevCtrl->txFreeBds > (MAX_SKB_FRAGS + 1)
			&& netif_queue_stopped(dev)) {
		/* Disable txdma bdone/pdone interrupt if we have free tx bds */
		pDevCtrl->intrl2_0->cpu_mask_set |= (UMAC_IRQ_TXDMA_BDONE |
				UMAC_IRQ_TXDMA_PDONE);
		netif_wake_queue(dev);
	}
	pDevCtrl->txLastCIndex = c_index;
#endif
}

/* --------------------------------------------------------------------------
Name: bcmgenet_xmit
Purpose: Send ethernet traffic
-------------------------------------------------------------------------- */
static int bcmgenet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	volatile struct tDmaRingRegs *tDma_ring;
	dma_addr_t mapping;
	struct Enet_CB *txCBPtr;
	unsigned int write_ptr = 0;
	int i = 0;
	unsigned long flags;
	struct status_64 *Status = NULL;
	int nr_frags = 0, index = DESC_INDEX;

	spin_lock_irqsave(&pDevCtrl->lock, flags);

	if (!pDevCtrl->clock_active) {
		printk(KERN_WARNING "%s: transmitting with gated clock!\n",
		       dev_name(&dev->dev));
		dev_kfree_skb_any(skb);
		spin_unlock_irqrestore(&pDevCtrl->lock, flags);
		return NETDEV_TX_OK;
	}
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	if (skb) {
		index = skb_get_queue_mapping(skb);
		/*
		* Mapping strategy:
		* queue_mapping = 0, unclassfieid, packet xmited through ring16
		* queue_mapping = 1, goes to ring 0. (highest priority queue)
		* queue_mapping = 2, goes to ring 1.
		* queue_mapping = 3, goes to ring 2.
		* queue_mapping = 4, goes to ring 3.
		*/
		if (index == 0)
			index = DESC_INDEX;
		else
			index -= 1;
		if (index != DESC_INDEX && index > 3) {
			printk(KERN_ERR "%s: skb->queue_mapping %d is invalid\n",
					__func__, skb_get_queue_mapping(skb));
			spin_unlock_irqrestore(&pDevCtrl->lock, flags);
			dev->stats.tx_errors++;
			dev->stats.tx_dropped++;
			return 1;
		}
		nr_frags = skb_shinfo(skb)->nr_frags;
		if (index == DESC_INDEX) {
			if (pDevCtrl->txFreeBds <= nr_frags + 1) {
				netif_stop_subqueue(dev, 0);
				spin_unlock_irqrestore(&pDevCtrl->lock, flags);
				printk(KERN_ERR "%s: tx ring %d full when queue awake\n",
					__func__, index);
				return 1;
			}
		} else if (pDevCtrl->txRingFreeBds[index] <= nr_frags + 1) {
			netif_stop_subqueue(dev, index+1);
			spin_unlock_irqrestore(&pDevCtrl->lock, flags);
			printk(KERN_ERR "%s: tx ring %d full when queue awake\n",
					__func__, index);
			return 1;
		}
	}
	/* Reclaim xmited skb for each subqueue */
	for (i = 0; i < GENET_MQ_CNT; i++)
		bcmgenet_tx_reclaim(dev, i);
#else
	if (skb) {
		nr_frags = skb_shinfo(skb)->nr_frags;
		if (pDevCtrl->txFreeBds <= nr_frags + 1) {
			netif_stop_queue(dev);
			spin_unlock_irqrestore(&pDevCtrl->lock, flags);
			printk(KERN_ERR "%s: tx ring full when queue awake\n",
				__func__);
			return 1;
		}
	}
#endif

	if (!skb) {
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
		for (i = 0; i < GENET_MQ_CNT; i++)
			bcmgenet_tx_reclaim(dev, i);
#endif
		bcmgenet_tx_reclaim(dev, DESC_INDEX);
		spin_unlock_irqrestore(&pDevCtrl->lock, flags);
		return 0;
	}
	/*
	 * reclaim xmited skb every 8 packets.
	 */
	if ((index == DESC_INDEX) &&
		(pDevCtrl->txFreeBds < pDevCtrl->nrTxBds - 8))
		bcmgenet_tx_reclaim(dev, index);

#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	if ((index != DESC_INDEX) && (pDevCtrl->txRingFreeBds[index]
			< GENET_MQ_BD_CNT - 8))
		bcmgenet_tx_reclaim(dev, index);
#endif

	tDma_ring = &pDevCtrl->txDma->tDmaRings[index];
	/*
	 * If 64 byte status block enabled, must make sure skb has
	 * enough headroom for us to insert 64B status block.
	 */
	if (GENET_TBUF_CTRL(pDevCtrl) & RBUF_64B_EN) {
		if (likely(skb_headroom(skb) < 64)) {
			struct sk_buff *new_skb;
			new_skb = skb_realloc_headroom(skb, 64);
			if (new_skb  == NULL) {
				dev_kfree_skb(skb);
				dev->stats.tx_errors++;
				dev->stats.tx_dropped++;
				spin_unlock_irqrestore(&pDevCtrl->lock, flags);
				return 0;
			} else if (skb->sk) {
				skb_set_owner_w(new_skb, skb->sk);
			}
			dev_kfree_skb(skb);
			skb = new_skb;
		}
		skb_push(skb, 64);
		Status = (struct status_64 *)skb->data;
	}
	write_ptr = tDma_ring->tdma_write_pointer;

	/* Obtain transmit control block */
	txCBPtr = bcmgenet_get_txcb(dev, &write_ptr, index);

	if (unlikely(!txCBPtr))
		BUG();

	txCBPtr->skb = skb;

	if ((skb->ip_summed  == CHECKSUM_PARTIAL) &&
			(GENET_TBUF_CTRL(pDevCtrl) & RBUF_64B_EN)) {
		u16 offset;
		offset = skb->csum_start - skb_headroom(skb) - 64;
		/* Insert 64B TSB and set the flag */
		Status->tx_csum_info = (offset << STATUS_TX_CSUM_START_SHIFT) |
			(offset + skb->csum_offset) |
			STATUS_TX_CSUM_LV;
	}

	/*
	 * Add the buffer to the ring.
	 * Set addr and length of DMA BD to be transmitted.
	 */
	if (!nr_frags) {
		mapping = dma_map_single(&pDevCtrl->dev->dev,
				skb->data, skb->len, DMA_TO_DEVICE);
		if (dma_mapping_error(&pDevCtrl->dev->dev, mapping)) {
			dev_err(&pDevCtrl->dev->dev, "Tx DMA map failed\n");
			dev_kfree_skb(skb);
			spin_unlock_irqrestore(&pDevCtrl->lock, flags);
			return 0;
		}
		dma_unmap_addr_set(txCBPtr, dma_addr, mapping);
		dma_unmap_len_set(txCBPtr, dma_len, skb->len);
		dmadesc_set_addr(txCBPtr->BdAddr, mapping);
		txCBPtr->BdAddr->length_status = (
			((unsigned long)((skb->len < ETH_ZLEN) ?
			ETH_ZLEN : skb->len)) << 16) | DMA_SOP | DMA_EOP |
			(DMA_TX_QTAG_MASK << DMA_TX_QTAG_SHIFT) |
			DMA_TX_APPEND_CRC;

		if (skb->ip_summed  == CHECKSUM_PARTIAL)
			txCBPtr->BdAddr->length_status |= DMA_TX_DO_CSUM;

#ifdef CONFIG_BCMGENET_DUMP_DATA
		printk(KERN_NOTICE "%s: data 0x%p len %d",
				__func__, skb->data, skb->len);
		print_hex_dump(KERN_NOTICE, "", DUMP_PREFIX_ADDRESS,
				16, 1, skb->data, skb->len, 0);
#endif
		/* Decrement total BD count and advance our write pointer */
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
		if (index == DESC_INDEX)
			pDevCtrl->txFreeBds -= 1;
		else
			pDevCtrl->txRingFreeBds[index] -= 1;
#else
		pDevCtrl->txFreeBds -= 1;
#endif
		/* advance producer index and write pointer.*/
		tDma_ring->tdma_producer_index += 1;
		tDma_ring->tdma_write_pointer = write_ptr;
		/* update stats */
		dev->stats.tx_bytes += ((skb->len < ETH_ZLEN) ?
				ETH_ZLEN : skb->len);
		dev->stats.tx_packets++;

	} else {
		/* xmit head */
		mapping = dma_map_single(&pDevCtrl->dev->dev,
				skb->data, skb_headlen(skb), DMA_TO_DEVICE);
		if (dma_mapping_error(&pDevCtrl->dev->dev, mapping)) {
			dev_kfree_skb(skb);
			dev_err(&pDevCtrl->dev->dev, "Tx DMA map failed\n");
			spin_unlock_irqrestore(&pDevCtrl->lock, flags);
			return 0;
		}
		dma_unmap_addr_set(txCBPtr, dma_addr, mapping);
		dma_unmap_len_set(txCBPtr, dma_len, skb->len);
		dmadesc_set_addr(txCBPtr->BdAddr, mapping);
		txCBPtr->BdAddr->length_status = (skb_headlen(skb) << 16) |
			(DMA_TX_QTAG_MASK << DMA_TX_QTAG_SHIFT) |
			DMA_SOP | DMA_TX_APPEND_CRC;

		if (skb->ip_summed  == CHECKSUM_PARTIAL)
			txCBPtr->BdAddr->length_status |= DMA_TX_DO_CSUM;

#ifdef CONFIG_BCMGENET_DUMP_DATA
		printk(KERN_NOTICE "%s: frag head len %d",
				__func__, skb_headlen(skb));
		print_hex_dump(KERN_NOTICE, "", DUMP_PREFIX_ADDRESS,
				16, 1, skb->data, skb_headlen(skb), 0);
#endif
		/* Decrement total BD count and advance our write pointer */
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
		if (index == DESC_INDEX)
			pDevCtrl->txFreeBds -= 1;
		else
			pDevCtrl->txRingFreeBds[index] -= 1;
#else
		pDevCtrl->txFreeBds -= 1;
#endif
		/* advance producer index and write pointer.*/
		tDma_ring->tdma_producer_index += 1;
		tDma_ring->tdma_write_pointer = write_ptr;
		dev->stats.tx_bytes += skb_headlen(skb);

		/* xmit fragment */
		for (i = 0; i < nr_frags; i++) {
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

			txCBPtr = bcmgenet_get_txcb(dev, &write_ptr, index);

			if (unlikely(!txCBPtr))
				BUG();
			txCBPtr->skb = NULL;

			mapping = skb_frag_dma_map(&pDevCtrl->dev->dev, frag, 0,
				skb_frag_size(frag), DMA_TO_DEVICE);
			if (dma_mapping_error(&pDevCtrl->dev->dev, mapping)) {
				printk(KERN_ERR "%s: Tx DMA map failed\n",
						__func__);
				/*TODO: Handle frag failure.*/
				spin_unlock_irqrestore(&pDevCtrl->lock, flags);
				return 0;
			}
			dma_unmap_addr_set(txCBPtr, dma_addr, mapping);
			dma_unmap_len_set(txCBPtr, dma_len, frag->size);
			dmadesc_set_addr(txCBPtr->BdAddr, mapping);
#ifdef CONFIG_BCMGENET_DUMP_DATA
			printk(KERN_NOTICE "%s: frag%d len %d",
					__func__, i, frag->size);
			print_hex_dump(KERN_NOTICE, "", DUMP_PREFIX_ADDRESS,
				16, 1,
				page_address(frag->page)+frag->page_offset,
				frag->size, 0);
#endif
			txCBPtr->BdAddr->length_status =
					((unsigned long)frag->size << 16) |
					(DMA_TX_QTAG_MASK << DMA_TX_QTAG_SHIFT);
			if (i == nr_frags - 1)
				txCBPtr->BdAddr->length_status |= DMA_EOP;

#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
			if (index == DESC_INDEX)
				pDevCtrl->txFreeBds -= 1;
			else
				pDevCtrl->txRingFreeBds[index] -= 1;
#else
			pDevCtrl->txFreeBds -= 1;
#endif
			/* advance producer index and write pointer.*/
			tDma_ring->tdma_producer_index += 1;
			tDma_ring->tdma_write_pointer = write_ptr;
			/* update stats */
			dev->stats.tx_bytes += frag->size;
		}
		dev->stats.tx_packets++;
	}

#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	if (index == DESC_INDEX) {
		if (pDevCtrl->txFreeBds <= (MAX_SKB_FRAGS + 1)) {
			netif_stop_subqueue(dev, 0);
			pDevCtrl->intrl2_0->cpu_mask_clear =
				UMAC_IRQ_TXDMA_BDONE | UMAC_IRQ_TXDMA_PDONE;

		}
	} else if (pDevCtrl->txRingFreeBds[index] <= (MAX_SKB_FRAGS + 1)) {
		netif_stop_subqueue(dev, index+1);
		pDevCtrl->intrl2_1->cpu_mask_clear = (1 << index);
	}
#else
	if (pDevCtrl->txFreeBds <= (MAX_SKB_FRAGS + 1)) {
		/* Enable Tx bdone/pdone interrupt !*/
		pDevCtrl->intrl2_0->cpu_mask_clear |= UMAC_IRQ_TXDMA_BDONE |
			UMAC_IRQ_TXDMA_PDONE;
		netif_stop_queue(dev);
	}
#endif
	dev->trans_start = jiffies;

	spin_unlock_irqrestore(&pDevCtrl->lock, flags);

	return 0;
}

/* NAPI polling method*/
static int bcmgenet_poll(struct napi_struct *napi, int budget)
{
	struct BcmEnet_devctrl *pDevCtrl = container_of(napi,
			struct BcmEnet_devctrl, napi);
	volatile struct intrl2Regs *intrl2 = pDevCtrl->intrl2_0;
	volatile struct rDmaRingRegs *rDma_desc;
	unsigned int work_done;
	work_done = bcmgenet_desc_rx(pDevCtrl, budget);

	/* tx reclaim */
	/*bcmgenet_xmit(NULL, pDevCtrl->dev);*/
	/* Allocate new SKBs for the BD ring */
	assign_rx_buffers(pDevCtrl);
	/* Advancing our read pointer and consumer index*/
	rDma_desc = &pDevCtrl->rxDma->rDmaRings[DESC_INDEX];
	rDma_desc->rdma_consumer_index += work_done;
	rDma_desc->rdma_read_pointer =
		(work_done + rDma_desc->rdma_read_pointer) & (TOTAL_DESC - 1);
	if (work_done < budget) {
		napi_complete(napi);
		intrl2->cpu_mask_clear |= UMAC_IRQ_RXDMA_BDONE;
	}
	return work_done;
}

/*
 * Interrupt bottom half
 */
static void bcmgenet_irq_task(struct work_struct *work)
{
	struct BcmEnet_devctrl *pDevCtrl = container_of(
			work, struct BcmEnet_devctrl, bcmgenet_irq_work);
	struct net_device *dev;

	dev = pDevCtrl->dev;

	TRACE(("%s\n", __func__));
	/* Cable plugged/unplugged event */
	if (pDevCtrl->irq0_stat & UMAC_IRQ_PHY_DET_R) {
		pDevCtrl->irq0_stat &= ~UMAC_IRQ_PHY_DET_R;
		printk(KERN_CRIT "%s cable plugged in, powering up\n",
				pDevCtrl->dev->name);
		bcmgenet_power_up(pDevCtrl, GENET_POWER_CABLE_SENSE);
	} else if (pDevCtrl->irq0_stat & UMAC_IRQ_PHY_DET_F) {
		pDevCtrl->irq0_stat &= ~UMAC_IRQ_PHY_DET_F;
		printk(KERN_CRIT "%s cable unplugged, powering down\n",
				pDevCtrl->dev->name);
		bcmgenet_power_down(pDevCtrl, GENET_POWER_CABLE_SENSE);
	}
	if (pDevCtrl->irq0_stat & UMAC_IRQ_MPD_R) {
		pDevCtrl->irq0_stat &= ~UMAC_IRQ_MPD_R;
		printk(KERN_CRIT "%s magic packet detected, waking up\n",
				pDevCtrl->dev->name);
		/* disable mpd interrupt */
		pDevCtrl->intrl2_0->cpu_mask_set |= UMAC_IRQ_MPD_R;
		/* disable CRC forward.*/
		pDevCtrl->umac->cmd &= ~CMD_CRC_FWD;
		if (pDevCtrl->dev_asleep)
			bcmgenet_power_up(pDevCtrl, GENET_POWER_WOL_MAGIC);

	} else if (pDevCtrl->irq0_stat & (UMAC_IRQ_HFB_SM | UMAC_IRQ_HFB_MM)) {
		pDevCtrl->irq0_stat &= ~(UMAC_IRQ_HFB_SM | UMAC_IRQ_HFB_MM);
		printk(KERN_CRIT "%s ACPI pattern matched, waking up\n",
				pDevCtrl->dev->name);
		/* disable HFB match interrupts */
		pDevCtrl->intrl2_0->cpu_mask_set |= (UMAC_IRQ_HFB_SM |
				UMAC_IRQ_HFB_MM);
		if (pDevCtrl->dev_asleep)
			bcmgenet_power_up(pDevCtrl, GENET_POWER_WOL_ACPI);
	}

	/* Link UP/DOWN event */
	if (pDevCtrl->irq0_stat & (UMAC_IRQ_LINK_UP|UMAC_IRQ_LINK_DOWN)) {
		pDevCtrl->irq0_stat &= ~(UMAC_IRQ_LINK_UP|UMAC_IRQ_LINK_DOWN);
		bcmgenet_mii_setup(pDevCtrl->dev);
	}
}

/*
 * bcmgenet_isr1: interrupt handler for ring buffer.
 */
static irqreturn_t bcmgenet_isr1(int irq, void *dev_id)
{
	struct BcmEnet_devctrl *pDevCtrl = dev_id;
	volatile struct intrl2Regs *intrl2 = pDevCtrl->intrl2_1;
	unsigned int index;

	/* Save irq status for bottom-half processing. */
	pDevCtrl->irq1_stat = intrl2->cpu_stat & ~intrl2->cpu_mask_status;
	/* clear inerrupts*/
	intrl2->cpu_clear |= pDevCtrl->irq1_stat;

	TRACE(("%s: IRQ=0x%x\n", __func__, pDevCtrl->irq1_stat));
	/*
	 * Check the MBDONE interrupts.
	 * packet is done, reclaim descriptors
	 */
	if (pDevCtrl->irq1_stat & 0x0000ffff) {
		index = 0;
		for (index = 0; index < 16; index++) {
			if (pDevCtrl->irq1_stat & (1<<index))
				bcmgenet_tx_reclaim(pDevCtrl->dev, index);
		}
	}
	return IRQ_HANDLED;
}
/*
 * bcmgenet_isr0: Handle various interrupts.
 */
static irqreturn_t bcmgenet_isr0(int irq, void *dev_id)
{
	struct BcmEnet_devctrl *pDevCtrl = dev_id;
	volatile struct intrl2Regs *intrl2 = pDevCtrl->intrl2_0;

	/* Save irq status for bottom-half processing. */
	pDevCtrl->irq0_stat = intrl2->cpu_stat & ~intrl2->cpu_mask_status;
	/* clear inerrupts*/
	intrl2->cpu_clear |= pDevCtrl->irq0_stat;

	TRACE(("IRQ=0x%x\n", pDevCtrl->irq0_stat));
#ifndef CONFIG_BCMGENET_RX_DESC_THROTTLE
	if (pDevCtrl->irq0_stat & (UMAC_IRQ_RXDMA_BDONE|UMAC_IRQ_RXDMA_PDONE)) {
		/*
		 * We use NAPI(software interrupt throttling, if
		 * Rx Descriptor throttling is not used.
		 * Disable interrupt, will be enabled in the poll method.
		 */
		if (likely(napi_schedule_prep(&pDevCtrl->napi))) {
			intrl2->cpu_mask_set |= UMAC_IRQ_RXDMA_BDONE;
			__napi_schedule(&pDevCtrl->napi);
		}
	}
#else
	/* Multiple buffer done event. */
	if (pDevCtrl->irq0_stat & UMAC_IRQ_RXDMA_MBDONE) {
		unsigned int work_done;
		volatile struct rDmaRingRegs *rDma_desc;

		rDma_desc = &pDevCtrl->rxDma->rDmaRings[DESC_INDEX];
		pDevCtrl->irq0_stat &= ~UMAC_IRQ_RXDMA_MBDONE;
		TRACE(("%s: %d packets available\n", __func__, DmaDescThres));
		work_done = bcmgenet_desc_rx(pDevCtrl, DmaDescThres);
		/* Allocate new SKBs for the BD ring */
		assign_rx_buffers(pDevCtrl);
		rDma_desc->rdma_consumer_index += work_done;
		rDma_desc->rdma_read_pointer =
			(work_done + rDma_desc->rdma_read_pointer) &
			(TOTAL_DESC - 1);
	}
#endif
	if (pDevCtrl->irq0_stat &
			(UMAC_IRQ_TXDMA_BDONE | UMAC_IRQ_TXDMA_PDONE)) {
		/* Tx reclaim */
		bcmgenet_xmit(NULL, pDevCtrl->dev);
	}
	if (pDevCtrl->irq0_stat & (UMAC_IRQ_PHY_DET_R |
				UMAC_IRQ_PHY_DET_F |
				UMAC_IRQ_LINK_UP |
				UMAC_IRQ_LINK_DOWN |
				UMAC_IRQ_HFB_SM |
				UMAC_IRQ_HFB_MM |
				UMAC_IRQ_MPD_R)) {
		/* all other interested interrupts handled in bottom half */
		schedule_work(&pDevCtrl->bcmgenet_irq_work);
	}

	return IRQ_HANDLED;
}
/*
 *  bcmgenet_desc_rx - descriptor based rx process.
 *  this could be called from bottom half, or from NAPI polling method.
 */
static unsigned int bcmgenet_desc_rx(void *ptr, unsigned int budget)
{
	struct BcmEnet_devctrl *pDevCtrl = ptr;
	struct net_device *dev = pDevCtrl->dev;
	struct Enet_CB *cb;
	struct sk_buff *skb;
	unsigned long dmaFlag;
	int len;
	unsigned int rxpktprocessed = 0, rxpkttoprocess = 0;
	unsigned int p_index = 0, c_index = 0, read_ptr = 0;

	p_index = pDevCtrl->rxDma->rDmaRings[DESC_INDEX].rdma_producer_index;
	p_index &= DMA_P_INDEX_MASK;
	c_index = pDevCtrl->rxDma->rDmaRings[DESC_INDEX].rdma_consumer_index;
	c_index &= DMA_C_INDEX_MASK;
	read_ptr = pDevCtrl->rxDma->rDmaRings[DESC_INDEX].rdma_read_pointer;

	if (p_index < c_index)
		rxpkttoprocess = (DMA_C_INDEX_MASK+1) - c_index + p_index;
	else
		rxpkttoprocess = p_index - c_index;
	TRACE(("RDMA: rxpkttoprocess=%d\n", rxpkttoprocess));

	while ((rxpktprocessed < rxpkttoprocess) &&
			(rxpktprocessed < budget)) {

		dmaFlag = (pDevCtrl->rxBds[read_ptr].length_status & 0xffff);
		len = ((pDevCtrl->rxBds[read_ptr].length_status)>>16);

		TRACE(("%s:p_index=%d c_index=%d read_ptr=%d "
			"len_stat=0x%08lx\n",
			__func__, p_index, c_index, read_ptr,
			pDevCtrl->rxBds[read_ptr].length_status));

		rxpktprocessed++;

		cb = &pDevCtrl->rxCbs[read_ptr];
		skb = cb->skb;
		BUG_ON(skb == NULL);
		dma_unmap_single(&dev->dev, dma_unmap_addr(cb, dma_addr),
				pDevCtrl->rxBufLen, DMA_FROM_DEVICE);

		dmadesc_set_addr(&pDevCtrl->rxBds[read_ptr], 0);

		if (read_ptr == pDevCtrl->nrRxBds-1)
			read_ptr = 0;
		else
			read_ptr++;

		if (unlikely(!(dmaFlag & DMA_EOP) || !(dmaFlag & DMA_SOP))) {
			printk(KERN_WARNING "Droping fragmented packet!\n");
			dev->stats.rx_dropped++;
			dev->stats.rx_errors++;
			dev_kfree_skb_any(cb->skb);
			continue;
		}
		/* report errors */
		if (unlikely(dmaFlag & (DMA_RX_CRC_ERROR |
						DMA_RX_OV |
						DMA_RX_NO |
						DMA_RX_LG |
						DMA_RX_RXER))) {
			TRACE(("ERROR: dmaFlag=0x%x\n", (unsigned int)dmaFlag));
			if (dmaFlag & DMA_RX_CRC_ERROR)
				dev->stats.rx_crc_errors++;
			if (dmaFlag & DMA_RX_OV)
				dev->stats.rx_over_errors++;
			if (dmaFlag & DMA_RX_NO)
				dev->stats.rx_frame_errors++;
			if (dmaFlag & DMA_RX_LG)
				dev->stats.rx_length_errors++;
			dev->stats.rx_dropped++;
			dev->stats.rx_errors++;

			/* discard the packet and advance consumer index.*/
			dev_kfree_skb_any(cb->skb);
			cb->skb = NULL;
			continue;
		} /* error packet */

		skb_put(skb, len);
		if (pDevCtrl->rbuf->rbuf_ctrl & RBUF_64B_EN) {
			struct status_64 *status;
			status = (struct status_64 *)skb->data;
			/* we have 64B rx status block enabled.*/
			if (pDevCtrl->rbuf->rbuf_chk_ctrl & RBUF_RXCHK_EN) {
				if (status->rx_csum & STATUS_RX_CSUM_OK)
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				else
					skb->ip_summed = CHECKSUM_NONE;
			}
			skb_pull(skb, 64);
			len -= 64;
		}

		if (pDevCtrl->bIPHdrOptimize) {
			skb_pull(skb, 2);
			len -= 2;
		}

		if (pDevCtrl->umac->cmd & CMD_CRC_FWD) {
			skb_trim(skb, len - 4);
			len -= 4;
		}
#ifdef CONFIG_BCMGENET_DUMP_DATA
		printk(KERN_NOTICE "bcmgenet_desc_rx : len=%d", skb->len);
		print_hex_dump(KERN_NOTICE, "", DUMP_PREFIX_ADDRESS,
			16, 1, skb->data, skb->len, 0);
#endif

		/*Finish setting up the received SKB and send it to the kernel*/
		skb->dev = pDevCtrl->dev;
		skb->protocol = eth_type_trans(skb, pDevCtrl->dev);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += len;
		if (dmaFlag & DMA_RX_MULT)
			dev->stats.multicast++;

		/* Notify kernel */
#ifdef CONFIG_BCMGENET_RX_DESC_THROTTLE
		netif_rx(skb);
#else
		netif_receive_skb(skb);
#endif
		cb->skb = NULL;
		TRACE(("pushed up to kernel\n"));
	}

	return rxpktprocessed;
}


/*
 * assign_rx_buffers:
 * Assign skb to RX DMA descriptor.
 */
static int assign_rx_buffers(struct BcmEnet_devctrl *pDevCtrl)
{
	struct sk_buff *skb;
	unsigned short bdsfilled = 0;
	unsigned long flags;
	struct Enet_CB *cb;
	dma_addr_t mapping;

	TRACE(("%s\n", __func__));

	/* This function may be called from irq bottom-half. */

#ifndef CONFIG_BCMGENET_RX_DESC_THROTTLE
	(void)flags;
	spin_lock_bh(&pDevCtrl->bh_lock);
#else
	spin_lock_irqsave(&pDevCtrl->lock, flags);
#endif

	/* loop here for each buffer needing assign */
	while (dmadesc_get_addr(pDevCtrl->rxBdAssignPtr) == 0) {
		cb = &pDevCtrl->rxCbs[pDevCtrl->rxBdAssignPtr-pDevCtrl->rxBds];
		skb = netdev_alloc_skb(pDevCtrl->dev,
				pDevCtrl->rxBufLen + SKB_ALIGNMENT);
		if (!skb) {
			printk(KERN_ERR " failed to allocate skb for rx\n");
			break;
		}
		handleAlignment(pDevCtrl, skb);

		/* keep count of any BD's we refill */
		bdsfilled++;
		cb->skb = skb;
		mapping = dma_map_single(&pDevCtrl->dev->dev,
			skb->data, pDevCtrl->rxBufLen, DMA_FROM_DEVICE);
		if (dma_mapping_error(&pDevCtrl->dev->dev, mapping)) {
			dev_err(&pDevCtrl->dev->dev, "assign_rx_buff DMA map failed\n");
			break;
		}
		dma_unmap_addr_set(cb, dma_addr, mapping);
		/* assign packet, prepare descriptor, and advance pointer */
		dmadesc_set_addr(pDevCtrl->rxBdAssignPtr, mapping);
		pDevCtrl->rxBdAssignPtr->length_status =
			(pDevCtrl->rxBufLen << 16);

		/* turn on the newly assigned BD for DMA to use */
		if (pDevCtrl->rxBdAssignPtr ==
				pDevCtrl->rxBds+pDevCtrl->nrRxBds-1)
			pDevCtrl->rxBdAssignPtr = pDevCtrl->rxBds;
		else
			pDevCtrl->rxBdAssignPtr++;

	}

	/* Enable rx DMA incase it was disabled due to running out of rx BD */
	pDevCtrl->rxDma->rdma_ctrl |= DMA_EN;

#ifndef CONFIG_BCMGENET_RX_DESC_THROTTLE
	spin_unlock_bh(&pDevCtrl->bh_lock);
#else
	spin_unlock_irqrestore(&pDevCtrl->lock, flags);
#endif

	TRACE(("%s return bdsfilled=%d\n", __func__, bdsfilled));
	return bdsfilled;
}

static void save_state(struct BcmEnet_devctrl *pDevCtrl)
{
	int ii;
	volatile struct DmaDesc *rxBdAssignPtr = pDevCtrl->rxBds;

	/*
	 * FIXME: Why is this code saving/restoring descriptors across suspend?
	 * Most other drivers just shut down and restart the network i/f.
	 */
	for (ii = 0; ii < pDevCtrl->nrRxBds; ++ii, ++rxBdAssignPtr) {
		pDevCtrl->saved_rx_desc[ii].length_status =
			rxBdAssignPtr->length_status;
		dmadesc_set_addr(&pDevCtrl->saved_rx_desc[ii],
				 dmadesc_get_addr(rxBdAssignPtr));
	}

	pDevCtrl->int_mask = pDevCtrl->intrl2_0->cpu_mask_status;
	pDevCtrl->rbuf_ctrl = pDevCtrl->rbuf->rbuf_ctrl;
}

static void restore_state(struct BcmEnet_devctrl *pDevCtrl)
{
	int ii;
	volatile struct DmaDesc *rxBdAssignPtr = pDevCtrl->rxBds;

	pDevCtrl->intrl2_0->cpu_mask_clear = 0xFFFFFFFF ^ pDevCtrl->int_mask;
	pDevCtrl->rbuf->rbuf_ctrl = pDevCtrl->rbuf_ctrl;

	for (ii = 0; ii < pDevCtrl->nrRxBds; ++ii, ++rxBdAssignPtr) {
		rxBdAssignPtr->length_status =
			pDevCtrl->saved_rx_desc[ii].length_status;
		dmadesc_set_addr(rxBdAssignPtr,
			dmadesc_get_addr(&pDevCtrl->saved_rx_desc[ii]));
	}

	pDevCtrl->rxDma->rdma_ctrl |= DMA_EN;

}

/*
 * init_umac: Initializes the uniMac controller
 */
static int init_umac(struct BcmEnet_devctrl *pDevCtrl)
{
	volatile struct uniMacRegs *umac;
	volatile struct intrl2Regs *intrl2;

	umac = pDevCtrl->umac;
	intrl2 = pDevCtrl->intrl2_0;

	TRACE(("bcmgenet: init_umac "));

	/* 7358a0/7552a0: bad default in RBUF_FLUSH_CTRL.umac_sw_rst */
	GENET_RBUF_FLUSH_CTRL(pDevCtrl) = 0;
	udelay(10);

	/* disable MAC while updating its registers */
	umac->cmd = 0;

	/* issue soft reset, wait for it to complete */
	umac->cmd = CMD_SW_RESET;
	udelay(1000);
	umac->cmd = 0;
	/* clear tx/rx counter */
	umac->mib_ctrl = MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT;
	umac->mib_ctrl = 0;

#ifdef MAC_LOOPBACK
	/* Enable GMII/MII loopback */
	umac->cmd |= CMD_LCL_LOOP_EN;
#endif
	umac->max_frame_len = ENET_MAX_MTU_SIZE;
	/*
	 * init rx registers, enable ip header optimization.
	 */
	if (pDevCtrl->bIPHdrOptimize)
		pDevCtrl->rbuf->rbuf_ctrl |= RBUF_ALIGN_2B ;
#if CONFIG_BRCM_GENET_VERSION >= 3
	pDevCtrl->rbuf->rbuf_tbuf_size_ctrl = 1;
#endif

	/* Mask all interrupts.*/
	intrl2->cpu_mask_set = 0xFFFFFFFF;
	intrl2->cpu_clear = 0xFFFFFFFF;
	intrl2->cpu_mask_clear = 0x0;

#ifdef CONFIG_BCMGENET_RX_DESC_THROTTLE
	intrl2->cpu_mask_clear |= UMAC_IRQ_RXDMA_MBDONE;
#else
	intrl2->cpu_mask_clear |= UMAC_IRQ_RXDMA_BDONE;
	TRACE(("%s:Enabling RXDMA_BDONE interrupt\n", __func__));
#endif /* CONFIG_BCMGENET_RX_DESC_THROTTLE */

	/* Monitor cable plug/unpluged event for internal PHY */
	if (pDevCtrl->phyType == BRCM_PHY_TYPE_INT) {
#if !defined(CONFIG_BCM7445A0)
		/* HW7445-870: energy detect on A0 silicon is unreliable */
		intrl2->cpu_mask_clear |= (UMAC_IRQ_PHY_DET_R |
				UMAC_IRQ_PHY_DET_F);
#endif /* !defined(CONFIG_BCM7445A0) */
		intrl2->cpu_mask_clear |= (UMAC_IRQ_LINK_DOWN |
				UMAC_IRQ_LINK_UP);
		/* Turn on ENERGY_DET interrupt in bcmgenet_open()
		 * TODO: fix me for active standby.
		 */
	} else if (pDevCtrl->extPhy) {
		intrl2->cpu_mask_clear |= (UMAC_IRQ_LINK_DOWN |
				UMAC_IRQ_LINK_UP);
	} else if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA) {
		GENET_TBUF_BP_MC(pDevCtrl) |= BIT(GENET_BP_IN_EN_SHIFT);

		/* bp_mask: back pressure mask */
#if defined(CONFIG_NET_SCH_MULTIQ)
		GENET_TBUF_BP_MC(pDevCtrl) |= GENET_BP_MASK;
#else
		GENET_TBUF_BP_MC(pDevCtrl) &= ~GENET_BP_MASK;
#endif
	}

	/* Enable rx/tx engine.*/
	TRACE(("done init umac\n"));
	return 0;
}
/*
 * init_edma: Initialize DMA control register
 */
static void init_edma(struct BcmEnet_devctrl *pDevCtrl)
{
#ifdef CONFIG_BCMGENET_RX_DESC_THROTTLE
	int speeds[] = {10, 100, 1000, 2500};
	int sid = 1, timeout, __maybe_unused first_bd;
#endif
	volatile struct rDmaRingRegs *rDma_desc;
	volatile struct tDmaRingRegs *tDma_desc;
	TRACE(("bcmgenet: init_edma\n"));

	/* init rDma */
	pDevCtrl->rxDma->rdma_scb_burst_size = DMA_MAX_BURST_LENGTH;
	/* by default, enable ring 16 (descriptor based) */
	rDma_desc = &pDevCtrl->rxDma->rDmaRings[DESC_INDEX];
	rDma_desc->rdma_write_pointer = 0;
	rDma_desc->rdma_producer_index = 0;
	rDma_desc->rdma_consumer_index = 0;
	rDma_desc->rdma_ring_buf_size = ((TOTAL_DESC << DMA_RING_SIZE_SHIFT) |
		RX_BUF_LENGTH);
	rDma_desc->rdma_start_addr = 0;
	rDma_desc->rdma_end_addr = WORDS_PER_BD * TOTAL_DESC - 1;
	rDma_desc->rdma_xon_xoff_threshold = ((DMA_FC_THRESH_LO
			<< DMA_XOFF_THRESHOLD_SHIFT) |
			DMA_FC_THRESH_HI);
	rDma_desc->rdma_read_pointer = 0;

#ifdef CONFIG_BCMGENET_RX_DESC_THROTTLE
	/*
	 * Use descriptor throttle, fire irq when multiple packets are done!
	 */
	rDma_desc->rdma_mbuf_done_threshold = DMA_DESC_THRES;
	/*
	 * Enable push timer, force the IRQ_DESC_THROT to fire when timeout
	 * occurs, prevent system slow reponse when handling low throughput.
	 */
	sid = (pDevCtrl->umac->cmd >> CMD_SPEED_SHIFT) & CMD_SPEED_MASK;
	timeout = 2*(DMA_DESC_THRES*ENET_MAX_MTU_SIZE)/speeds[sid];
	pDevCtrl->rxDma->rdma_timeout[DESC_INDEX] = timeout & DMA_TIMEOUT_MASK;
#endif	/* CONFIG_BCMGENET_RX_DESC_THROTTLE */


	/* Init tDma */
	pDevCtrl->txDma->tdma_scb_burst_size = DMA_MAX_BURST_LENGTH;
	/* by default, enable ring DESC_INDEX (descriptor based) */
	tDma_desc = &pDevCtrl->txDma->tDmaRings[DESC_INDEX];
	tDma_desc->tdma_producer_index = 0;
	tDma_desc->tdma_consumer_index = 0;
	tDma_desc->tdma_mbuf_done_threshold = 1;
	/* Disable rate control for now */
	tDma_desc->tdma_flow_period = 0;
#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	/* Unclassified traffic goes to ring 16 */
	tDma_desc->tdma_ring_buf_size = ((GENET_DEFAULT_BD_CNT <<
				DMA_RING_SIZE_SHIFT) | RX_BUF_LENGTH);

	first_bd = GENET_MQ_CNT * GENET_MQ_BD_CNT;
	tDma_desc->tdma_start_addr = first_bd * WORDS_PER_BD;
	tDma_desc->tdma_read_pointer = first_bd * WORDS_PER_BD;
	tDma_desc->tdma_write_pointer = first_bd;
	tDma_desc->tdma_end_addr = TOTAL_DESC * WORDS_PER_BD - 1;
	pDevCtrl->txFreeBds = GENET_DEFAULT_BD_CNT;
	/* initiaize multi xmit queue */
	bcmgenet_init_multiq(pDevCtrl->dev);

#else
	tDma_desc->tdma_ring_buf_size = ((TOTAL_DESC << DMA_RING_SIZE_SHIFT) |
			RX_BUF_LENGTH);
	tDma_desc->tdma_start_addr = 0;
	tDma_desc->tdma_end_addr = WORDS_PER_BD * TOTAL_DESC - 1;
	tDma_desc->tdma_read_pointer = 0;

	/*
	 * FIXME: tdma_write_pointer and rdma_read_pointer are essentially
	 * scratch registers and the GENET block does not use them for
	 * anything.  We should maintain them in DRAM instead, because
	 * register accesses are expensive.
	 */
	tDma_desc->tdma_write_pointer = 0;
#endif

}

#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
/*
 * init multi xmit queues, only available for GENET2
 * the queue is partitioned as follows:
 *
 * queue 0 - 3 is priority based, each one has 48 descriptors,
 * with queue 0 being the highest priority queue.
 *
 * queue 16 is the default tx queue, with 64 descriptors.
 */
static void bcmgenet_init_multiq(struct net_device *dev)
{
	int i, dma_enable;
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);

	dma_enable = pDevCtrl->txDma->tdma_ctrl & DMA_EN;
	pDevCtrl->txDma->tdma_ctrl &= ~DMA_EN;
	/* Enable strict priority arbiter mode */
	pDevCtrl->txDma->tdma_arb_ctrl = 0x2;
	for (i = 0; i < GENET_MQ_CNT; i++) {
		int first_bd;

		/* first 64 txCbs are reserved for default tx queue (ring 16) */
		pDevCtrl->txRingCBs[i] = pDevCtrl->txCbs +
			GENET_DEFAULT_BD_CNT + i * GENET_MQ_BD_CNT;
		pDevCtrl->txRingSize[i] = GENET_MQ_BD_CNT;
		pDevCtrl->txRingCIndex[i] = 0;
		pDevCtrl->txRingFreeBds[i] = GENET_MQ_BD_CNT;

		pDevCtrl->txDma->tDmaRings[i].tdma_producer_index = 0;
		pDevCtrl->txDma->tDmaRings[i].tdma_consumer_index = 0;
		pDevCtrl->txDma->tDmaRings[i].tdma_ring_buf_size =
			(GENET_MQ_BD_CNT << DMA_RING_SIZE_SHIFT) |
			RX_BUF_LENGTH;

		first_bd = GENET_MQ_BD_CNT * i;
		pDevCtrl->txDma->tDmaRings[i].tdma_start_addr =
			first_bd * WORDS_PER_BD;
		pDevCtrl->txDma->tDmaRings[i].tdma_read_pointer =
			first_bd * WORDS_PER_BD;
		pDevCtrl->txDma->tDmaRings[i].tdma_write_pointer = first_bd;

		pDevCtrl->txDma->tDmaRings[i].tdma_end_addr =
			WORDS_PER_BD * (i + 1) * GENET_MQ_BD_CNT - 1;
		pDevCtrl->txDma->tDmaRings[i].tdma_flow_period =
			ENET_MAX_MTU_SIZE << 16;
		pDevCtrl->txDma->tDmaRings[i].tdma_mbuf_done_threshold = 1;

		/* Configure ring as decriptor ring and setup priority */
		pDevCtrl->txDma->tdma_ring_cfg |= (1 << i);
		pDevCtrl->txDma->tdma_priority[0] |=
			((GENET_Q0_PRIORITY + i) << 5*i);
		pDevCtrl->txDma->tdma_ctrl |=
			(1 << (i + DMA_RING_BUF_EN_SHIFT));
	}
	/* Set ring #16 priority */
	pDevCtrl->txDma->tdma_priority[2] |=
		((GENET_Q0_PRIORITY + GENET_MQ_CNT) << 20);
	if (dma_enable)
		pDevCtrl->txDma->tdma_ctrl |= DMA_EN;
}
#endif
/*
 * bcmgenet_init_dev: initialize uniMac devie
 * allocate Tx/Rx buffer descriptors pool, Tx control block pool.
 */
static int bcmgenet_init_dev(struct BcmEnet_devctrl *pDevCtrl)
{
	int i, ret;
	unsigned long base;
	void *ptxCbs, *prxCbs;
	volatile struct DmaDesc *lastBd;

	pDevCtrl->clk = clk_get(&pDevCtrl->pdev->dev, "enet");
	pDevCtrl->clk_wol = clk_get(&pDevCtrl->pdev->dev, "enet-wol");
	bcmgenet_clock_enable(pDevCtrl);

	TRACE(("%s\n", __func__));
	/* setup buffer/pointer relationships here */
	pDevCtrl->nrTxBds = pDevCtrl->nrRxBds = TOTAL_DESC;
	/* Always use 2KB buffer for 7420*/
	pDevCtrl->rxBufLen = RX_BUF_LENGTH;

	/* register block locations */
	base = pDevCtrl->dev->base_addr;
	pDevCtrl->sys = (struct SysRegs *)(base);
	pDevCtrl->grb = (struct GrBridgeRegs *)(base + GENET_GR_BRIDGE_OFF);
	pDevCtrl->ext = (struct ExtRegs *)(base + GENET_EXT_OFF);
#if CONFIG_BRCM_GENET_VERSION == 1
	/* SWLINUX-1813: EXT block is not available on MOCA_GENET */
#if !defined(CONFIG_BCM7125)
	if (pDevCtrl->devnum == 1)
#endif
		pDevCtrl->ext = NULL;
#endif
	pDevCtrl->intrl2_0 = (struct intrl2Regs *)(base + GENET_INTRL2_0_OFF);
	pDevCtrl->intrl2_1 = (struct intrl2Regs *)(base + GENET_INTRL2_1_OFF);
	pDevCtrl->rbuf = (struct rbufRegs *)(base + GENET_RBUF_OFF);
	pDevCtrl->umac = (struct uniMacRegs *)(base + GENET_UMAC_OFF);
	pDevCtrl->hfb = (unsigned long *)(base + GENET_HFB_OFF);
	pDevCtrl->txDma = (struct tDmaRegs *)(base + GENET_TDMA_REG_OFF);
	pDevCtrl->rxDma = (struct rDmaRegs *)(base + GENET_RDMA_REG_OFF);

#if CONFIG_BRCM_GENET_VERSION > 1
	pDevCtrl->tbuf = (struct tbufRegs *)(base + GENET_TBUF_OFF);
	pDevCtrl->hfbReg = (struct hfbRegs *)(base + GENET_HFB_REG_OFF);
#endif

	pDevCtrl->rxBds = (struct DmaDesc *)(base + GENET_RDMA_OFF);
	pDevCtrl->txBds = (struct DmaDesc *)(base + GENET_TDMA_OFF);

	TRACE(("%s: rxbds=0x%08x txbds=0x%08x\n", __func__,
		(unsigned int)pDevCtrl->rxBds, (unsigned int)pDevCtrl->txBds));

	/* alloc space for the tx control block pool */
	ptxCbs = kmalloc(pDevCtrl->nrTxBds*sizeof(struct Enet_CB), GFP_KERNEL);
	if (!ptxCbs) {
		bcmgenet_clock_disable(pDevCtrl);
		return -ENOMEM;
	}
	memset(ptxCbs, 0, pDevCtrl->nrTxBds*sizeof(struct Enet_CB));
	pDevCtrl->txCbs = (struct Enet_CB *)ptxCbs;

	/* initialize rx ring pointer variables. */
	pDevCtrl->rxBdAssignPtr = pDevCtrl->rxBds;
	prxCbs = kmalloc(pDevCtrl->nrRxBds*sizeof(struct Enet_CB), GFP_KERNEL);
	if (!prxCbs) {
		ret = -ENOMEM;
		goto error2;
	}
	memset(prxCbs, 0, pDevCtrl->nrRxBds*sizeof(struct Enet_CB));
	pDevCtrl->rxCbs = (struct Enet_CB *)prxCbs;

	/* init the receive buffer descriptor ring */
	for (i = 0; i < pDevCtrl->nrRxBds; i++) {
		pDevCtrl->rxBds[i].length_status = (pDevCtrl->rxBufLen<<16);
		dmadesc_set_addr(&pDevCtrl->rxBds[i], 0);
	}
	lastBd = pDevCtrl->rxBds + pDevCtrl->nrRxBds - 1;

	/* clear the transmit buffer descriptors */
	for (i = 0; i < pDevCtrl->nrTxBds; i++) {
		pDevCtrl->txBds[i].length_status = 0<<16;
		dmadesc_set_addr(&pDevCtrl->txBds[i], 0);
	}
	lastBd = pDevCtrl->txBds + pDevCtrl->nrTxBds - 1;
	pDevCtrl->txFreeBds = pDevCtrl->nrTxBds;

	/* fill receive buffers */
	if (assign_rx_buffers(pDevCtrl) == 0) {
		printk(KERN_ERR "Failed to assign rx buffers\n");
		ret = -ENOMEM;
		goto error1;
	}

	TRACE(("%s done!\n", __func__));
	/* init umac registers */
	if (init_umac(pDevCtrl)) {
		ret = -EFAULT;
		goto error1;
	}

	/* init dma registers */
	init_edma(pDevCtrl);

#if (CONFIG_BRCM_GENET_VERSION > 1) && defined(CONFIG_NET_SCH_MULTIQ)
	pDevCtrl->nrTxBds = GENET_DEFAULT_BD_CNT;
#endif

	TRACE(("%s done!\n", __func__));
	/* if we reach this point, we've init'ed successfully */
	return 0;
error1:
	kfree(prxCbs);
error2:
	kfree(ptxCbs);
	bcmgenet_clock_disable(pDevCtrl);

	TRACE(("%s Failed!\n", __func__));
	return ret;
}

/* Uninitialize tx/rx buffer descriptor pools */
static void bcmgenet_uninit_dev(struct BcmEnet_devctrl *pDevCtrl)
{
	int i;

	if (pDevCtrl) {
		/* disable DMA */
		pDevCtrl->rxDma->rdma_ctrl = 0;
		pDevCtrl->txDma->tdma_ctrl = 0;

		for (i = 0; i < pDevCtrl->nrTxBds; i++) {
			if (pDevCtrl->txCbs[i].skb != NULL) {
				dev_kfree_skb(pDevCtrl->txCbs[i].skb);
				pDevCtrl->txCbs[i].skb = NULL;
			}
		}
		for (i = 0; i < pDevCtrl->nrRxBds; i++) {
			if (pDevCtrl->rxCbs[i].skb != NULL) {
				dev_kfree_skb(pDevCtrl->rxCbs[i].skb);
				pDevCtrl->rxCbs[i].skb = NULL;
			}
		}

		/* free the transmit buffer descriptor */
		if (pDevCtrl->txBds)
			pDevCtrl->txBds = NULL;
		/* free the receive buffer descriptor */
		if (pDevCtrl->rxBds)
			pDevCtrl->rxBds = NULL;
		/* free the transmit control block pool */
		kfree(pDevCtrl->txCbs);
		/* free the transmit control block pool */
		kfree(pDevCtrl->rxCbs);

		clk_put(pDevCtrl->clk);
	}
}

#define FILTER_POS(i)	(((HFB_NUM_FLTRS-i-1)>>2))
#define SET_HFB_FILTER_LEN(dev, i, len) \
do { \
	u32 tmp = GENET_HFB_FLTR_LEN(dev, FILTER_POS(i)); \
	tmp &= ~(RBUF_FLTR_LEN_MASK << (RBUF_FLTR_LEN_SHIFT * (i & 0x03))); \
	tmp |= (len << (RBUF_FLTR_LEN_SHIFT * (i & 0x03))); \
	GENET_HFB_FLTR_LEN(dev, FILTER_POS(i)) = tmp; \
} while (0)

#define GET_HFB_FILTER_LEN(dev, i) \
	((GENET_HFB_FLTR_LEN(dev, FILTER_POS(i)) >> \
		(RBUF_FLTR_LEN_SHIFT * (i & 0x03))) & RBUF_FLTR_LEN_MASK)

#if CONFIG_BRCM_GENET_VERSION >= 3
/* The order of GENET_x_HFB_FLT_ENBLE_0/1 is reversed !!! */
#define	GET_HFB_FILTER_EN(dev, i) \
	(dev->hfbReg->hfb_flt_enable[i < 32] & (1 << (i % 32)))
#define HFB_FILTER_ENABLE(dev, i) \
	(dev->hfbReg->hfb_flt_enable[i < 32] |= (1 << (i % 32)))
#define HFB_FILTER_DISABLE(dev, i) \
	(dev->hfbReg->hfb_flt_enable[i < 32] &= ~(1 << (i % 32)))
#define	HFB_FILTER_DISABLE_ALL(dev) \
	do { \
		dev->hfbReg->hfb_flt_enable[0] = 0; \
		dev->hfbReg->hfb_flt_enable[1] = 0; \
	} while (0)
#else
#define GET_HFB_FILTER_EN(dev, i) \
	((GENET_HFB_CTRL(dev) >> (i + RBUF_HFB_FILTER_EN_SHIFT)) & 0x01)
#define HFB_FILTER_ENABLE(dev, i) \
	(GENET_HFB_CTRL(dev) |= 1 << (i + RBUF_HFB_FILTER_EN_SHIFT))
#define HFB_FILTER_DISABLE(dev, i) \
	(GENET_HFB_CTRL(dev) &= ~(1 << (i + RBUF_HFB_FILTER_EN_SHIFT)))
#define	HFB_FILTER_DISABLE_ALL(dev) \
	(GENET_HFB_CTRL(dev) &= ~(0xffff << (RBUF_HFB_FILTER_EN_SHIFT)))
#endif

/*
 * Program ACPI pattern into HFB. Return filter index if succesful.
 * if user == 1, the data will be copied from user space.
 */
int bcmgenet_update_hfb(struct net_device *dev, unsigned int *data,
		int len, int user)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	int filter, offset, count;
	unsigned int *tmp;

	TRACE(("Updating HFB len=0x%d\n", len));

	count = HFB_NUM_FLTRS;
	offset = 64;
	if (GENET_HFB_CTRL(pDevCtrl) & RBUF_HFB_256B) {
#if CONFIG_BRCM_GENET_VERSION < 3
		count >>= 1;
#endif
		offset = 128;
	}

	if (len > offset)
		return -EINVAL;

	/* find next unused filter */
	for (filter = 0; filter < count; filter++) {
		if (!GET_HFB_FILTER_EN(pDevCtrl, filter))
			break;
	}
	if (filter == count) {
		printk(KERN_ERR "no unused filter available!\n");
		return -EINVAL;	/* all filters have been enabled*/
	}

	if (user) {
		tmp = kmalloc(len*sizeof(unsigned int), GFP_KERNEL);
		if (tmp == NULL) {
			printk(KERN_ERR "%s: Malloc faild\n", __func__);
			return -EFAULT;
		}
		/* copy pattern data */
		if (copy_from_user(tmp, data, len*sizeof(unsigned int)) != 0) {
			printk(KERN_ERR "Failed to copy user data: src=%p, dst=%p\n",
				data, pDevCtrl->hfb + filter*offset);
			return -EFAULT;
		}
	} else {
		tmp = data;
	}
	/* Copy pattern data into HFB registers.*/
	for (count = 0; count < offset; count++) {
		if (count < len)
			pDevCtrl->hfb[filter * offset + count] = *(tmp + count);
		else
			pDevCtrl->hfb[filter * offset + count] = 0;
	}
	if (user)
		kfree(tmp);

	/* set the filter length*/
	SET_HFB_FILTER_LEN(pDevCtrl, filter, len*2);

	/*enable this filter.*/
	HFB_FILTER_ENABLE(pDevCtrl, filter);

	return filter;

}
EXPORT_SYMBOL(bcmgenet_update_hfb);
/*
 * read ACPI pattern data for a particular filter.
 */
static int bcmgenet_read_hfb(struct net_device *dev, struct acpi_data *u_data)
{
	int filter, offset, count, len;
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);

	if (get_user(filter, &(u_data->fltr_index))) {
		printk(KERN_ERR "Failed to get user data\n");
		return -EFAULT;
	}

	count = HFB_NUM_FLTRS;
	offset = 128;
	if (GENET_HFB_CTRL(pDevCtrl) & RBUF_HFB_256B) {
#if CONFIG_BRCM_GENET_VERSION < 3
		count >>= 1;
#endif
		offset = 256;
	}

	if (filter > count)
		return -EINVAL;

	/* see if this filter is enabled, if not, return length 0 */
	if (!GET_HFB_FILTER_EN(pDevCtrl, filter)) {
		len = 0;
		put_user(len , &u_data->count);
		return 0;
	}
	/* check the filter length, in bytes */
	len = GET_HFB_FILTER_LEN(pDevCtrl, filter);
	if (u_data->count < len)
		return -EINVAL;
	/* copy pattern data */
	if (copy_to_user((void *)(u_data->p_data),
			(void *)(pDevCtrl->hfb + filter*offset), len)) {
		printk(KERN_ERR "Failed to copy data to user space: src=%p, dst=%p\n",
				pDevCtrl->hfb+filter*offset, u_data->p_data);
		return -EFAULT;
	}
	return len;
}
/*
 * clear the HFB, disable filter indexed by "filter" argument.
 */
static inline void bcmgenet_clear_hfb(struct BcmEnet_devctrl *pDevCtrl,
		int filter)
{
	if (filter == CLEAR_ALL_HFB) {
		HFB_FILTER_DISABLE_ALL(pDevCtrl);
		GENET_HFB_CTRL(pDevCtrl) &= ~RBUF_HFB_EN;
	} else {
		/* disable this filter */
		HFB_FILTER_DISABLE(pDevCtrl, filter);
		/* clear filter length register */
		SET_HFB_FILTER_LEN(pDevCtrl, filter, 0);
	}

}
/*
 * Utility function to get interface ip address in kernel space.
 */
static inline unsigned int bcmgenet_getip(struct net_device *dev)
{
	struct net_device *pnet_device;
	unsigned int ip = 0;

	read_lock(&dev_base_lock);
	/* read all devices */
	for_each_netdev(&init_net, pnet_device)
	{
		if ((netif_running(pnet_device)) &&
				(pnet_device->ip_ptr != NULL) &&
				(!strcmp(pnet_device->name, dev->name))) {
			struct in_device *pin_dev;
			pin_dev = (struct in_device *)(pnet_device->ip_ptr);
			if (pin_dev && pin_dev->ifa_list)
				ip = htonl(pin_dev->ifa_list->ifa_address);
			break;
		}
	}
	read_unlock(&dev_base_lock);
	return ip;
}

static void bcmgenet_clock_enable(struct BcmEnet_devctrl *pDevCtrl)
{
	unsigned long flags;

	spin_lock_irqsave(&pDevCtrl->lock, flags);
	clk_enable(pDevCtrl->clk);
	pDevCtrl->clock_active = 1;
	spin_unlock_irqrestore(&pDevCtrl->lock, flags);
}

static void bcmgenet_clock_disable(struct BcmEnet_devctrl *pDevCtrl)
{
	unsigned long flags;

	spin_lock_irqsave(&pDevCtrl->lock, flags);
	pDevCtrl->clock_active = 0;
	clk_disable(pDevCtrl->clk);
	spin_unlock_irqrestore(&pDevCtrl->lock, flags);
}

/*
 * ethtool function - get WOL (Wake on LAN) settings,
 * Only Magic Packet Detection is supported through ethtool,
 * the ACPI (Pattern Matching) WOL option is supported in
 * bcmumac_do_ioctl function.
 */
static void bcmgenet_get_wol(struct net_device *dev,
		struct ethtool_wolinfo *wol)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	volatile struct uniMacRegs *umac = pDevCtrl->umac;
	wol->supported = WAKE_MAGIC | WAKE_MAGICSECURE | WAKE_ARP;

	if (!netif_running(dev))
		return;

	wol->wolopts = pDevCtrl->wolopts;
	if (wol->wolopts & WAKE_MAGICSECURE) {
		put_unaligned_be16(umac->mpd_pw_ms, &wol->sopass[0]);
		put_unaligned_be32(umac->mpd_pw_ls, &wol->sopass[2]);
	} else {
		memset(&wol->sopass[0], 0, sizeof(wol->sopass));
	}
}

/*
 * ethtool function - set WOL (Wake on LAN) settings.
 * Only for magic packet detection mode.
 */
static int bcmgenet_set_wol(struct net_device *dev,
		struct ethtool_wolinfo *wol)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	volatile struct uniMacRegs *umac = pDevCtrl->umac;

	if (wol->wolopts & ~(WAKE_MAGIC | WAKE_MAGICSECURE | WAKE_ARP))
		return -EINVAL;

	if (wol->wolopts & WAKE_MAGICSECURE) {
		umac->mpd_pw_ms = get_unaligned_be16(&wol->sopass[0]);
		umac->mpd_pw_ls = get_unaligned_be32(&wol->sopass[2]);
		umac->mpd_ctrl |= MPD_PW_EN;
	}

	device_set_wakeup_enable(&dev->dev, wol->wolopts);
	pDevCtrl->wolopts = wol->wolopts;
	return 0;
}
/*
 * ethtool function - get generic settings.
 */
static int bcmgenet_get_settings(struct net_device *dev,
		struct ethtool_cmd *cmd)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	int rc = 0;

	if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA) {
		/* see comments in bcmgenet_set_settings() */
		cmd->autoneg = netif_carrier_ok(pDevCtrl->dev);
		cmd->speed = SPEED_1000;
		cmd->duplex = DUPLEX_HALF;
		cmd->port = PORT_BNC;
	} else {
		if (!netif_running(dev))
			return -EINVAL;
		rc = mii_ethtool_gset(&pDevCtrl->mii, cmd);
	}

	return rc;
}
/*
 * ethtool function - set settings.
 */
static int bcmgenet_set_settings(struct net_device *dev,
		struct ethtool_cmd *cmd)
{
	int err = 0;
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);

	if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA) {
		/* mocad uses cmd->autoneg to control our RUNNING flag */
		if (cmd->autoneg)
			netif_carrier_on(pDevCtrl->dev);
		else
			netif_carrier_off(pDevCtrl->dev);
	} else {
		if (!netif_running(dev))
			return -EINVAL;

		err = mii_ethtool_sset(&pDevCtrl->mii, cmd);
		if (err < 0)
			return err;
		bcmgenet_mii_setup(dev);

		if (cmd->maxrxpkt != 0)
			DmaDescThres = cmd->maxrxpkt;
	}

	return err;
}
/*
 * ethtool function - get driver info.
 */
static void bcmgenet_get_drvinfo(struct net_device *dev,
		struct ethtool_drvinfo *info)
{
	strncpy(info->driver, CARDNAME, sizeof(info->driver));
	strncpy(info->version, VER_STR, sizeof(info->version));

}
static int bcmgenet_set_rx_csum(struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	spin_lock_bh(&pDevCtrl->bh_lock);
	if (!(dev->flags & NETIF_F_RXCSUM)) {
		/*pDevCtrl->rbuf->rbuf_endian_ctrl &= ~RBUF_ENDIAN_NOSWAP;*/
		pDevCtrl->rbuf->rbuf_ctrl &= ~RBUF_64B_EN;
		pDevCtrl->rbuf->rbuf_chk_ctrl &= ~RBUF_RXCHK_EN;
	} else {
		/*pDevCtrl->rbuf->rbuf_endian_ctrl &= ~RBUF_ENDIAN_NOSWAP;*/
		pDevCtrl->rbuf->rbuf_ctrl |= RBUF_64B_EN;
		pDevCtrl->rbuf->rbuf_chk_ctrl |= RBUF_RXCHK_EN ;
	}
	spin_unlock_bh(&pDevCtrl->bh_lock);
	return 0;
}
static int bcmgenet_set_tx_csum(struct net_device *dev)
{
	unsigned long flags;
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	spin_lock_irqsave(&pDevCtrl->lock, flags);
	if (!(dev->features & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM))) {
		GENET_TBUF_CTRL(pDevCtrl) &= ~RBUF_64B_EN;
		if (dev->needed_headroom > 64)
			dev->needed_headroom -= 64;
	} else {
		GENET_TBUF_CTRL(pDevCtrl) |= RBUF_64B_EN;
		if (dev->needed_headroom < 64)
			dev->needed_headroom += 64;
	}
	spin_unlock_irqrestore(&pDevCtrl->lock, flags);
	return 0;
}
static int bcmgenet_set_sg(struct net_device *dev)
{
	if ((dev->features & NETIF_F_SG) &&
			!(dev->features & NETIF_F_IP_CSUM)) {
		printk(KERN_WARNING "Tx Checksum offloading disabled, not setting SG\n");
		return -EINVAL;
	}
	/* must have 64B tx status enabled */
	return 0;
}
static int bcmgenet_set_features(struct net_device *dev,
		netdev_features_t features)
{
	int ret;
	netdev_features_t changed = features ^ dev->features;
	if (changed & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM))
		ret = bcmgenet_set_tx_csum(dev);
	if (changed & (NETIF_F_RXCSUM))
		ret = bcmgenet_set_rx_csum(dev);
	if (changed & NETIF_F_SG)
		ret = bcmgenet_set_sg(dev);

	return ret;
}

/*
 * standard ethtool support functions.
 */
static struct ethtool_ops bcmgenet_ethtool_ops = {
	.get_settings		= bcmgenet_get_settings,
	.set_settings		= bcmgenet_set_settings,
	.get_drvinfo		= bcmgenet_get_drvinfo,
	.get_wol			= bcmgenet_get_wol,
	.set_wol			= bcmgenet_set_wol,
	.get_link			= ethtool_op_get_link,
};

static int bcmgenet_enable_arp_filter(struct BcmEnet_devctrl *pDevCtrl)
{
	struct net_device *dev = pDevCtrl->dev;
	unsigned int ip;

	ip = bcmgenet_getip(dev);
	if (ip) {
		/* clear the lower halfwords */
		hfb_arp[HFB_ARP_LEN-2] &= ~0xffff;
		hfb_arp[HFB_ARP_LEN-1] &= ~0xffff;
		hfb_arp[HFB_ARP_LEN-2] |= (ip >> 16);
		hfb_arp[HFB_ARP_LEN-1] |= (ip & 0xFFFF);
		/* Enable HFB, to response to ARP request.*/
		if (bcmgenet_update_hfb(dev, hfb_arp, HFB_ARP_LEN, 0) < 0) {
			printk(KERN_ERR "%s: Unable to update HFB\n",
			       __func__);
			return -1;
		}
		GENET_HFB_CTRL(pDevCtrl) |= RBUF_HFB_EN;
		return 0;
	}

	return -1;
}
/*
 * Power down the unimac, based on mode.
 */
static void bcmgenet_power_down(struct BcmEnet_devctrl *pDevCtrl, int mode)
{
	struct net_device *dev;
	int retries = 0;

	dev = pDevCtrl->dev;
	switch (mode) {
	case GENET_POWER_CABLE_SENSE:
#if 0
		/*
		 * EPHY bug, setting ext_pwr_down_dll and ext_pwr_down_phy cause
		 * link IRQ bouncing.
		 */
		pDevCtrl->ext->ext_pwr_mgmt |= (EXT_PWR_DOWN_PHY |
				EXT_PWR_DOWN_DLL | EXT_PWR_DOWN_BIAS);
#else
		/* Workaround for putting EPHY in iddq mode. */
		pDevCtrl->mii.mdio_write(dev, pDevCtrl->phyAddr, 0x1f, 0x008b);
		pDevCtrl->mii.mdio_write(dev, pDevCtrl->phyAddr, 0x10, 0x01c0);
		pDevCtrl->mii.mdio_write(dev, pDevCtrl->phyAddr, 0x14, 0x7000);
		pDevCtrl->mii.mdio_write(dev, pDevCtrl->phyAddr, 0x1f, 0x000f);
		pDevCtrl->mii.mdio_write(dev, pDevCtrl->phyAddr, 0x10, 0x20d0);
		pDevCtrl->mii.mdio_write(dev, pDevCtrl->phyAddr, 0x1f, 0x000b);

#endif
		break;
	case GENET_POWER_WOL_MAGIC:
		/* disable RX while turning on MPD_EN */
		pDevCtrl->umac->cmd &= ~CMD_RX_EN;
		mdelay(10);
		pDevCtrl->umac->mpd_ctrl |= MPD_EN;
		while (!(pDevCtrl->rbuf->rbuf_status & RBUF_STATUS_WOL)) {
			retries++;
			if (retries > 5) {
				printk(KERN_CRIT "%s: polling "
				       "wol mode timeout\n", dev->name);
				pDevCtrl->umac->mpd_ctrl &= ~MPD_EN;
				return;
			}
			mdelay(1);
		}
		printk(KERN_DEBUG "%s: MP WOL-ready status set after "
		       "%d msec\n", dev->name, retries);

		/* Enable CRC forward */
		pDevCtrl->umac->cmd |= CMD_CRC_FWD;
		/* Receiver must be enabled for WOL MP detection */
		pDevCtrl->umac->cmd |= CMD_RX_EN;

		if (pDevCtrl->ext && pDevCtrl->dev_asleep)
			pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_ENERGY_DET_MASK;

		pDevCtrl->intrl2_0->cpu_mask_clear |= UMAC_IRQ_MPD_R;

		set_bit(GENET_POWER_WOL_MAGIC, &pDevCtrl->wol_enabled);
		break;
	case GENET_POWER_WOL_ACPI:
		if (bcmgenet_enable_arp_filter(pDevCtrl)) {
			printk(KERN_CRIT "%s failed to set HFB filter\n",
			       dev->name);
			return;
		}
		pDevCtrl->umac->cmd &= ~CMD_RX_EN;
		mdelay(10);
		GENET_HFB_CTRL(pDevCtrl) |= RBUF_ACPI_EN;
		while (!(pDevCtrl->rbuf->rbuf_status & RBUF_STATUS_WOL)) {
			retries++;
			if (retries > 5) {
				printk(KERN_CRIT "%s polling "
				       "wol mode timeout\n", dev->name);
				GENET_HFB_CTRL(pDevCtrl) &= ~RBUF_ACPI_EN;
				return;
			}
			mdelay(1);
		}
		/* Receiver must be enabled for WOL ACPI detection */
		pDevCtrl->umac->cmd |= CMD_RX_EN;
		if (pDevCtrl->ext && pDevCtrl->dev_asleep)
			pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_ENERGY_DET_MASK;
		printk(KERN_DEBUG "%s: ACPI WOL-ready status set "
		       "after %d msec\n", dev->name, retries);
		/* enable HFB match interrupts */
		pDevCtrl->intrl2_0->cpu_mask_clear |= (UMAC_IRQ_HFB_MM |
				UMAC_IRQ_HFB_SM);
		set_bit(GENET_POWER_WOL_ACPI, &pDevCtrl->wol_enabled);
		break;
	case GENET_POWER_PASSIVE:
		/* Power down LED */
		bcmgenet_mii_reset(pDevCtrl->dev);
		if (pDevCtrl->ext)
			pDevCtrl->ext->ext_pwr_mgmt |= (EXT_PWR_DOWN_PHY |
				EXT_PWR_DOWN_DLL | EXT_PWR_DOWN_BIAS);
		break;
	default:
		break;
	}

}
static void bcmgenet_power_up(struct BcmEnet_devctrl *pDevCtrl, int mode)
{
	switch (mode) {
	case GENET_POWER_CABLE_SENSE:
#if 0
		pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_DLL;
		pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_PHY;
		pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_BIAS;
#endif
		/* enable APD */
		if (pDevCtrl->ext) {
			pDevCtrl->ext->ext_pwr_mgmt |= EXT_PWR_DN_EN_LD;
			bcmgenet_mii_reset(pDevCtrl->dev);
		}
		break;
	case GENET_POWER_WOL_MAGIC:
		pDevCtrl->umac->mpd_ctrl &= ~MPD_EN;
		/* Disable CRC Forward */
		pDevCtrl->umac->cmd &= ~CMD_CRC_FWD;
		/* Stop monitoring magic packet IRQ */
		pDevCtrl->intrl2_0->cpu_mask_set |= UMAC_IRQ_MPD_R;
		clear_bit(GENET_POWER_WOL_MAGIC, &pDevCtrl->wol_enabled);
		break;
	case GENET_POWER_WOL_ACPI:
		GENET_HFB_CTRL(pDevCtrl) &= ~RBUF_ACPI_EN;
		bcmgenet_clear_hfb(pDevCtrl, CLEAR_ALL_HFB);
		/* Stop monitoring ACPI interrupts */
		pDevCtrl->intrl2_0->cpu_mask_set |= (UMAC_IRQ_HFB_SM |
				UMAC_IRQ_HFB_MM);
		clear_bit(GENET_POWER_WOL_ACPI, &pDevCtrl->wol_enabled);
		break;
	case GENET_POWER_PASSIVE:
		if (pDevCtrl->ext) {
			pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_DLL;
			pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_PHY;
			pDevCtrl->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_BIAS;
			/* enable APD */
			pDevCtrl->ext->ext_pwr_mgmt |= EXT_PWR_DN_EN_LD;
			bcmgenet_mii_reset(pDevCtrl->dev);
		}
	default:
		break;
	}
	bcmgenet_ephy_workaround(pDevCtrl->dev);
}
/*
 * ioctl handle special commands that are not present in ethtool.
 */
static int bcmgenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	unsigned long flags;
	struct acpi_data *u_data;
	int val = 0;

	if (!netif_running(dev))
		return -EINVAL;
	/* we can add sub-command in ifr_data if we need to in the future */
	switch (cmd) {
	case SIOCSACPISET:
		spin_lock_irqsave(&pDevCtrl->lock, flags);
		bcmgenet_power_down(pDevCtrl, GENET_POWER_WOL_ACPI);
		spin_unlock_irqrestore(&pDevCtrl->lock, flags);
		break;
	case SIOCSACPICANCEL:
		spin_lock_irqsave(&pDevCtrl->lock, flags);
		bcmgenet_power_up(pDevCtrl, GENET_POWER_WOL_ACPI);
		spin_unlock_irqrestore(&pDevCtrl->lock, flags);
		break;
	case SIOCSPATTERN:
		u_data = (struct acpi_data *)rq->ifr_data;
		val =  bcmgenet_update_hfb(dev, (unsigned int *)u_data->p_data,
				u_data->count, 1);
		if (val >= 0)
			put_user(val, &u_data->fltr_index);
		break;
	case SIOCGPATTERN:
		u_data = (struct acpi_data *)rq->ifr_data;
		val = bcmgenet_read_hfb(dev, u_data);
		break;
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		val = generic_mii_ioctl(&pDevCtrl->mii, if_mii(rq), cmd, NULL);
		break;
	default:
		val = -EINVAL;
		break;
	}

	return val;
}
static const struct net_device_ops bcmgenet_netdev_ops = {
	.ndo_open = bcmgenet_open,
	.ndo_stop = bcmgenet_close,
	.ndo_start_xmit = bcmgenet_xmit,
#ifdef CONFIG_NET_SCH_MULTIQ
	.ndo_select_queue = bcmgenet_select_queue,
#endif
	.ndo_tx_timeout = bcmgenet_timeout,
	.ndo_set_rx_mode = bcmgenet_set_rx_mode,
	.ndo_set_mac_address = bcmgenet_set_mac_addr,
	.ndo_do_ioctl = bcmgenet_ioctl,
	.ndo_set_features = bcmgenet_set_features,
};

static int bcmgenet_drv_probe(struct platform_device *pdev)
{
	void __iomem *base;
	int err = -EIO;
	struct device_node *dn = pdev->dev.of_node;
	struct BcmEnet_devctrl *pDevCtrl;
	struct net_device *dev;
	const void *macaddr;

#ifdef CONFIG_NET_SCH_MULTIQ
	dev = alloc_etherdev_mq(sizeof(*(pDevCtrl)), GENET_MQ_CNT+1);
#else
	dev = alloc_etherdev(sizeof(*pDevCtrl));
#endif
	if (dev == NULL) {
		dev_err(&pdev->dev, "can't allocate net device\n");
		err = -ENOMEM;
		goto err0;
	}
	pDevCtrl = (struct BcmEnet_devctrl *)netdev_priv(dev);
	pDevCtrl->irq0 = irq_of_parse_and_map(dn, 0);
	pDevCtrl->irq1 = irq_of_parse_and_map(dn, 1);

	if (!pDevCtrl->irq0 || !pDevCtrl->irq1) {
		dev_err(&pdev->dev, "can't find IRQs\n");
		return -EINVAL;
	}

	macaddr = of_get_mac_address(dn);
	if (!macaddr) {
		dev_err(&pdev->dev, "can't find MAC address\n");
		return -EINVAL;
	}

	base = of_iomap(dn, 0);
	TRACE(("%s: base=0x%x\n", __func__, (unsigned int)base));

	if (!base) {
		dev_err(&pdev->dev, "can't ioremap\n");
		return -EINVAL;
	}

	dev->base_addr = (unsigned long)base;
	SET_NETDEV_DEV(dev, &pdev->dev);
	dev_set_drvdata(&pdev->dev, pDevCtrl);
	memcpy(dev->dev_addr, macaddr, ETH_ALEN);
	dev->irq = pDevCtrl->irq0;
	dev->watchdog_timeo         = 2*HZ;
	SET_ETHTOOL_OPS(dev, &bcmgenet_ethtool_ops);
	dev->netdev_ops = &bcmgenet_netdev_ops;
	netif_napi_add(dev, &pDevCtrl->napi, bcmgenet_poll, 64);

	netdev_boot_setup_check(dev);

	pDevCtrl->dev = dev;
	/* NOTE: with fast-bridge , must turn this off! */
	pDevCtrl->bIPHdrOptimize = 1;

	spin_lock_init(&pDevCtrl->lock);
	spin_lock_init(&pDevCtrl->bh_lock);
	mutex_init(&pDevCtrl->mdio_mutex);
	/* Mii wait queue */
	init_waitqueue_head(&pDevCtrl->wq);

	/* TODO: get these from the OF properties */
	pDevCtrl->phyType = BRCM_PHY_TYPE_INT;
	pDevCtrl->phySpeed = SPEED_1000;
	pDevCtrl->extPhy = 0;
	pDevCtrl->phyAddr = 10;
	pDevCtrl->devnum = pdev->id;

	pDevCtrl->pdev = pdev;

	/* Init GENET registers, Tx/Rx buffers */
	if (bcmgenet_init_dev(pDevCtrl) < 0)
		goto err1;

	bcmgenet_mii_init(dev);

	INIT_WORK(&pDevCtrl->bcmgenet_irq_work, bcmgenet_irq_task);

	err = register_netdev(dev);
	if (err != 0)
		goto err2;

	if (pDevCtrl->extPhy) {
		/* No Link status IRQ */
		INIT_WORK(&pDevCtrl->bcmgenet_link_work,
				bcmgenet_gphy_link_status);
		init_timer(&pDevCtrl->timer);
		pDevCtrl->timer.data = (unsigned long)pDevCtrl;
		pDevCtrl->timer.function = bcmgenet_gphy_link_timer;
	} else {
		/* check link status */
		bcmgenet_mii_setup(dev);
	}

	netif_carrier_off(pDevCtrl->dev);
	pDevCtrl->next_dev = eth_root_dev;
	eth_root_dev = dev;
	bcmgenet_clock_disable(pDevCtrl);

	return 0;

err2:
	bcmgenet_clock_disable(pDevCtrl);
	bcmgenet_uninit_dev(pDevCtrl);
err1:
	iounmap(base);
	free_netdev(dev);
err0:
	return err;
}

static int bcmgenet_drv_remove(struct platform_device *pdev)
{
	struct BcmEnet_devctrl *pDevCtrl = dev_get_drvdata(&pdev->dev);

	unregister_netdev(pDevCtrl->dev);
	free_irq(pDevCtrl->irq0, pDevCtrl);
	free_irq(pDevCtrl->irq1, pDevCtrl);
	bcmgenet_uninit_dev(pDevCtrl);
	iounmap((void __iomem *)pDevCtrl->base_addr);
	free_netdev(pDevCtrl->dev);
	return 0;
}

static int bcmgenet_drv_suspend(struct device *dev)
{
	int val = 0;
	struct BcmEnet_devctrl *pDevCtrl = dev_get_drvdata(dev);

	cancel_work_sync(&pDevCtrl->bcmgenet_irq_work);

	/*
	 * Save/restore the interface status across PM modes.
	 * FIXME: Don't use open/close for suspend/resume.
	 */
	pDevCtrl->dev_opened = netif_running(pDevCtrl->dev);
	if (pDevCtrl->dev_opened && !pDevCtrl->dev_asleep) {
		pDevCtrl->dev_asleep = 1;
		val = bcmgenet_close(pDevCtrl->dev);
	}

	return val;
}

static int bcmgenet_drv_resume(struct device *dev)
{
	int val = 0;
	struct BcmEnet_devctrl *pDevCtrl = dev_get_drvdata(dev);

	if (pDevCtrl->dev_opened)
		val = bcmgenet_open(pDevCtrl->dev);
	pDevCtrl->dev_asleep = 0;

	return val;
}

static struct dev_pm_ops bcmgenet_pm_ops = {
	.suspend		= bcmgenet_drv_suspend,
	.resume			= bcmgenet_drv_resume,
};

static const struct of_device_id bcmgenet_match[] = {
	{ .compatible = "brcm,genet-v3" },
	{ .compatible = "brcm,genet-v4" },
	{ },
};

static struct platform_driver bcmgenet_plat_drv = {
	.probe =		bcmgenet_drv_probe,
	.remove =		bcmgenet_drv_remove,
	.driver = {
		.name =		"bcmgenet",
		.owner =	THIS_MODULE,
		.pm =		&bcmgenet_pm_ops,
		.of_match_table = bcmgenet_match,
	},
};

static int bcmgenet_module_init(void)
{
	platform_driver_register(&bcmgenet_plat_drv);
	return 0;
}

static void bcmgenet_module_cleanup(void)
{
	platform_driver_unregister(&bcmgenet_plat_drv);
}

module_init(bcmgenet_module_init);
module_exit(bcmgenet_module_cleanup);
MODULE_LICENSE("GPL");
