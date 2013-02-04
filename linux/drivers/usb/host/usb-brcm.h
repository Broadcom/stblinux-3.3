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

#ifndef _USB_BRCM_H
#define _USB_BRCM_H

#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/mmdebug.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/brcmstb/brcmstb.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
#include <linux/usb/hcd.h>
#else
#include "../core/hcd.h"
#endif

/* force non-byteswapping reads/writes on LE and BE alike */
#define CONFIG_USB_EHCI_BIG_ENDIAN_MMIO		1
#define CONFIG_USB_OHCI_BIG_ENDIAN_MMIO		1
#undef readl_be
#undef writel_be
#define readl_be(x)				__raw_readl(x)
#define writel_be(x, y)				__raw_writel(x, y)

/* stock Linux drivers assume USB is always PCI-based on platforms with PCI */
#undef CONFIG_PCI

extern int brcm_usb_probe(struct platform_device *, const char *,
	const struct hc_driver *);
extern int brcm_usb_remove(struct platform_device *pdev);
extern void brcm_usb_suspend(struct usb_hcd *hcd);
extern void brcm_usb_resume(struct usb_hcd *hcd);
extern int brcm_usb_is_inactive(void);

#endif /* _USB_BRCM_H */
