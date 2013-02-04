/*
 * Copyright (C) 2009 - 2013 Broadcom Corporation
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

#include "usb-brcm.h"

static struct platform_driver	ehci_hcd_brcm_driver;
#define PLATFORM_DRIVER		ehci_hcd_brcm_driver

#include "ehci-hcd.c"

/* catch unwanted redefinitions */
#define CONFIG_PCI		0

/* for global USB settings, see arch/mips/brcmstb/bchip.c */

static int ehci_brcm_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	int ret;

	ehci->big_endian_mmio = 1;

	ehci->caps = (struct ehci_caps *) hcd->regs;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	ehci->regs = (struct ehci_regs *) (hcd->regs +
		HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase)));
#else
	ehci->regs = (struct ehci_regs *) (hcd->regs +
		HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase)));
#endif
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize PCI reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	/* This fixes the lockup during reboot due to prior interrupts */
	ehci_writel(ehci, CMD_RESET, &ehci->regs->command);
	mdelay(10);

	/*
	 * SWLINUX-1705: Avoid OUT packet underflows during high memory
	 *   bus usage
	 * port_status[0x0f] = Broadcom-proprietary USB_EHCI_INSNREG00 @ 0x90
	 */
	ehci_writel(ehci, 0x00800040, &ehci->regs->port_status[0x10]);
	ehci_writel(ehci, 0x00000001, &ehci->regs->port_status[0x12]);

	/* force HC to halt state */
	ehci_halt(ehci);

	ret = ehci_init(hcd);
	if (ret)
		return ret;
	ehci_reset(ehci);
	ehci_port_power(ehci, 1);

	return ret;
}

static void ehci_brcm_shutdown(struct usb_hcd *hcd)
{
	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))
		ehci_shutdown(hcd);
}

/* ehci_brcm_wait_for_sof
 * Wait for start of next microframe, then wait extra delay microseconds
 */
static inline void ehci_brcm_wait_for_sof(struct ehci_hcd *ehci, u32 delay)
{
	int frame_idx = ehci_readl(ehci, &ehci->regs->frame_index);

	while (frame_idx == ehci_readl(ehci, &ehci->regs->frame_index))
		;
	udelay(delay);
}

/* ehci_brcm_hub_control
 * Intercept echi-hcd request to complete RESUME and align it to the start
 * of the next microframe.
 * If RESUME is complete too late in the microframe, host controller
 * detects babble on suspended port and resets the port afterwards.
 * This s/w workaround allows to avoid this problem.
 * See http://jira.broadcom.com/browse/SWLINUX-1909 for more details
 */
static int ehci_brcm_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		ports = HCS_N_PORTS(ehci->hcs_params);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	unsigned long flags;
	int retval, irq_disabled = 0;

	/* RESUME is cleared when GetPortStatus() is called 20ms after start
	  of RESUME */
	if ((typeReq == GetPortStatus) &&
	    (wIndex && wIndex <= ports) &&
	    ehci->reset_done[wIndex-1] &&
	    time_after_eq(jiffies, ehci->reset_done[wIndex-1]) &&
	    (ehci_readl(ehci, status_reg) & PORT_RESUME)) {
		/* to make sure we are not interrupted until RESUME bit
		  is cleared, disable interrupts on current CPU */
		ehci_dbg(ehci, "SOF alignment workaround\n");
		irq_disabled = 1;
		local_irq_save(flags);
		ehci_brcm_wait_for_sof(ehci, 5);
	}

	retval = ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
	if (irq_disabled)
		local_irq_restore(flags);
	return retval;
}

#ifdef CONFIG_PM
static int ehci_brcm_suspend(struct device *dev)
{
	struct usb_hcd		*hcd = dev_get_drvdata(dev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	unsigned long		flags;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible.  The PM and USB cores make sure that
	 * the root hub is either suspended or stopped.
	 */
	ehci_prepare_ports_for_controller_suspend(ehci, device_may_wakeup(dev));
	spin_lock_irqsave(&ehci->lock, flags);
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);
	brcm_usb_suspend(hcd);
	spin_unlock_irqrestore(&ehci->lock, flags);

	return 0;
}

static int ehci_brcm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	brcm_usb_resume(hcd);
	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* If CF is still set, we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int mask = INTR_MASK;

		ehci_prepare_ports_for_controller_resume(ehci);
		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		return 0;
	}

	dev_info(dev, "lost power, restarting\n");
	usb_root_hub_lost_power(hcd->self.root_hub);

	/*
	 * SWLINUX-1705: Avoid OUT packet underflows during high memory
	 *   bus usage
	 * port_status[0x0f] = Broadcom-proprietary USB_EHCI_INSNREG00
	 * @ 0x90
	 */
	ehci_writel(ehci, 0x00800040, &ehci->regs->port_status[0x10]);
	ehci_writel(ehci, 0x00000001, &ehci->regs->port_status[0x12]);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	ehci->rh_state = EHCI_RH_SUSPENDED;
	return 0;
}

static const struct dev_pm_ops brcm_ehci_pmops = {
	.suspend	= ehci_brcm_suspend,
	.resume		= ehci_brcm_resume,
};

#define BRCM_EHCI_PMOPS (&brcm_ehci_pmops)

#else
#define BRCM_EHCI_PMOPS NULL
#endif


/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_brcm_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"Broadcom STB EHCI",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_USB2 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ehci_brcm_reset,
	.start =		ehci_run,
	.stop =			ehci_stop,
	.shutdown =		ehci_brcm_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,
	.endpoint_reset =	ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_brcm_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
#endif
	.relinquish_port =	ehci_relinquish_port,
	.port_handed_over =	ehci_port_handed_over,
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

/*-------------------------------------------------------------------------*/

static int ehci_hcd_brcm_probe(struct platform_device *pdev)
{
	static const u64 usb_dmamask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = (u64 *)&usb_dmamask;
	pdev->dev.coherent_dma_mask = usb_dmamask;

	return brcm_usb_probe(pdev, hcd_name, &ehci_brcm_hc_driver);
}

static int ehci_hcd_brcm_remove(struct platform_device *pdev)
{
	return brcm_usb_remove(pdev);
}

static const struct of_device_id usb_hcd_brcm_match[] = {
	{ .compatible = "brcm,ehci-brcm" },
	{},
};

static struct platform_driver ehci_hcd_brcm_driver = {
	.probe = ehci_hcd_brcm_probe,
	.remove = ehci_hcd_brcm_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "ehci-brcm",
		.bus = &platform_bus_type,
		.of_match_table = of_match_ptr(usb_hcd_brcm_match),
		.pm = BRCM_EHCI_PMOPS,
	}
};

MODULE_ALIAS("ehci-brcm");
