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

#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/brcmstb/brcmstb.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
#include <linux/usb/hcd.h>
#else
#include "../core/hcd.h"
#endif

#define MAX_HCD			8

static struct clk *usb_clk;

int brcm_usb_probe(struct platform_device *pdev, char *hcd_name,
	const struct hc_driver *hc_driver)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	int irq, ret, len;

	if (usb_disabled())
		return -ENODEV;

	if (!usb_clk)
		usb_clk = clk_get(NULL, "usb");
	clk_enable(usb_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err("platform_get_resource error.");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err("platform_get_irq error.");
		return -ENODEV;
	}

	/* initialize hcd */
	hcd = usb_create_hcd(hc_driver, &pdev->dev, (char *)hcd_name);
	if (!hcd) {
		err("Failed to create hcd");
		return -ENOMEM;
	}

	len = res->end - res->start + 1;
	hcd->regs = ioremap(res->start, len);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = len;
	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (ret != 0) {
		err("Failed to add hcd");
		iounmap(hcd->regs);
		usb_put_hcd(hcd);
		clk_disable(usb_clk);
		return ret;
	}

#ifdef CONFIG_PM
	/* disable autosuspend by default to preserve
	 * original behavior
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	usb_disable_autosuspend(hcd->self.root_hub);
#else
	hcd->self.root_hub->autosuspend_disabled = 1;
#endif
#endif

	return ret;
}
EXPORT_SYMBOL(brcm_usb_probe);

int brcm_usb_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	clk_disable(usb_clk);
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);

	return 0;
}
EXPORT_SYMBOL(brcm_usb_remove);

void brcm_usb_suspend(struct usb_hcd *hcd)
{
	/* Since all HCs share clock source, once we enable USB clock, all
	   controllers are capable to generate interrupts if enabled. Since some
	   controllers at this time are still marked as non-accessible, this
	   leads to spurious interrupts.
	   To avoid this, disable controller interrupts.
	*/
	disable_irq(hcd->irq);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	clk_disable(usb_clk);
}
EXPORT_SYMBOL(brcm_usb_suspend);

void brcm_usb_resume(struct usb_hcd *hcd)
{
	clk_enable(usb_clk);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	enable_irq(hcd->irq);
}
EXPORT_SYMBOL(brcm_usb_resume);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom USB common functions");
