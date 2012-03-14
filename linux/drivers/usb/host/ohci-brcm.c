/*
 * Copyright (C) 2009 - 2010 Broadcom Corporation
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

static struct platform_driver	ohci_hcd_brcm_driver;
#define PLATFORM_DRIVER		ohci_hcd_brcm_driver

#include "ohci-hcd.c"

/* catch unwanted redefinitions */
#define CONFIG_PCI		0

/* for global USB settings, see arch/mips/brcmstb/bchip.c */

static int ohci_brcm_reset(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);

	ohci->flags |= OHCI_QUIRK_BE_MMIO;
	distrust_firmware = 0;
	ohci_hcd_init(ohci);
	return ohci_init(ohci);
}

static int ohci_brcm_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	struct ohci_regs __iomem *regs;

	regs = hcd->regs;

	ohci_writel(ohci, 1, &regs->cmdstatus);
	ohci_readl(ohci, &regs->cmdstatus);
	mdelay(10);

	ohci_hcd_init(ohci);
	ohci_init(ohci);
	ohci_run(ohci);
	hcd->state = HC_STATE_RUNNING;
	return 0;
}

static void ohci_brcm_shutdown(struct usb_hcd *hcd)
{
	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))
		ohci_shutdown(hcd);
}

#ifdef CONFIG_PM
static int ohci_brcm_suspend(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;

	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;
	ohci_bus_suspend(hcd);
	brcm_usb_suspend(hcd);
	return 0;
}

static int ohci_brcm_resume(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	brcm_usb_resume(hcd);
	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;

	ohci_finish_controller_resume(hcd);
	ohci_bus_resume(hcd);
	return 0;
}

#endif

static const struct hc_driver ohci_brcm_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"Broadcom STB OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_brcm_reset,
	.start =		ohci_brcm_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_brcm_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_brcm_suspend,
	.bus_resume =		ohci_brcm_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_brcm_probe(struct platform_device *pdev)
{
	return brcm_usb_probe(pdev, hcd_name, &ohci_brcm_hc_driver);
}

static int ohci_hcd_brcm_remove(struct platform_device *pdev)
{
	return brcm_usb_remove(pdev);
}


static struct platform_driver ohci_hcd_brcm_driver = {
	.probe		= ohci_hcd_brcm_probe,
	.remove		= ohci_hcd_brcm_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "ohci-brcm",
		.owner	= THIS_MODULE,
	},
};

MODULE_ALIAS("ohci-brcm");
