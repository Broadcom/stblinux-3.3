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
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/brcmstb/brcmstb.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
#include <linux/usb/hcd.h>
#else
#include "../core/hcd.h"
#endif

#define MAX_HCD			8

static struct clk *usb_clk;

/***********************************************************************
 * Library functions
 ***********************************************************************/

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

#ifdef CONFIG_OF

/***********************************************************************
 * DT support for USB instances
 ***********************************************************************/

struct brcm_usb_instance {
	void __iomem		*ctrl_regs;
	int			ioc;
	int			ipp;
};

#ifdef __LITTLE_ENDIAN
#define ENDIAN_SETTING		0x03 /* !WABO !FNBO FNHW BABO */
#else
#define ENDIAN_SETTING		0x0e /* WABO FNBO FNHW !BABO */
#endif

#define ENDIAN_m		0x0f
#define IOC_m			BIT(4)
#define IPP_m			BIT(5)

#define SEQ_EN_m		BIT(0)
#define SCB_SIZE_s		7
#define SCB_SIZE_m		(0x1f << SCB_SIZE_s)

#define SETUP_REG		0x00
#define EBRIDGE_REG		0x0c
#define OBRIDGE_REG		0x10

static void brcm_usb_instance_hw_init(struct brcm_usb_instance *priv)
{
	u32 reg;

	/* set up byte order for DRAM accesses */
	reg = (readl(priv->ctrl_regs + SETUP_REG) & ~ENDIAN_m) |
	      ENDIAN_SETTING;

	/* set overcurrent and power polarity based on DT properties */
	reg &= ~(IOC_m | IPP_m);
	if (priv->ioc)
		reg |= IOC_m;
	if (priv->ipp)
		reg |= IPP_m;
	writel(reg, priv->ctrl_regs + SETUP_REG);

	/* override lame bridge defaults */
	reg = readl(priv->ctrl_regs + OBRIDGE_REG);
	reg &= ~SEQ_EN_m;
	writel(reg, priv->ctrl_regs + OBRIDGE_REG);

	reg = readl(priv->ctrl_regs + EBRIDGE_REG);
	reg &= ~SEQ_EN_m;
	reg &= ~SCB_SIZE_m;
	reg |= 0x08 << SCB_SIZE_s;
	writel(reg, priv->ctrl_regs + EBRIDGE_REG);
}

static int __devinit brcm_usb_instance_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct resource ctrl_res;
	const u32 *prop;
	struct brcm_usb_instance *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, priv);

	if (of_address_to_resource(dn, 0, &ctrl_res)) {
		dev_err(&pdev->dev, "can't get USB_CTRL base address\n");
		return -EINVAL;
	}

	priv->ctrl_regs = devm_request_and_ioremap(&pdev->dev, &ctrl_res);
	if (!priv->ctrl_regs) {
		dev_err(&pdev->dev, "can't map register space\n");
		return -EINVAL;
	}

	prop = of_get_property(dn, "ipp", NULL);
	if (prop)
		priv->ipp = be32_to_cpup(prop);

	prop = of_get_property(dn, "ioc", NULL);
	if (prop)
		priv->ioc = be32_to_cpup(prop);

	brcm_usb_instance_hw_init(priv);

	return of_platform_populate(dn, NULL, NULL, NULL);
}

static const struct of_device_id brcm_usb_instance_match[] = {
	{ .compatible = "brcm,usb-instance" },
	{},
};

static struct platform_driver brcm_usb_instance_driver = {
	.driver = {
		.name = "usb-brcm",
		.bus = &platform_bus_type,
		.of_match_table = of_match_ptr(brcm_usb_instance_match),
	}
};

/*
 * We really don't want to try to undo of_platform_populate(), so it
 * is not possible to unbind/deregister this driver.
 */
static int __init brcm_usb_instance_init(void)
{
	return platform_driver_probe(&brcm_usb_instance_driver,
				     brcm_usb_instance_probe);
}
module_init(brcm_usb_instance_init);

#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom USB common functions");
