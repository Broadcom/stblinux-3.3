/*
 * Copyright (C) 2013 Broadcom Corporation
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

#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/if_ether.h>
#include <linux/platform_device.h>
#include <linux/brcmstb/brcmstb.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>

#include <mach/irqs.h>
#include <mach/hardware.h>

#include "common.h"

static struct map_desc io_map[] __initdata = {
	{
	.virtual = (unsigned long)BRCMSTB_PERIPH_VIRT,
	.pfn     = __phys_to_pfn(BRCMSTB_PERIPH_PHYS),
	.length  = BRCMSTB_PERIPH_LENGTH,
	.type    = MT_DEVICE,
	},
};

static const char *brcmstb_match[] __initdata = {
	"brcm,brcmstb",
	NULL
};

static const struct of_device_id brcmstb_dt_irq_match[] __initconst = {
	{ .compatible = "arm,cortex-a15-gic", .data = gic_of_init },
	{ }
};

static void __init brcmstb_map_io(void)
{
	iotable_init(io_map, ARRAY_SIZE(io_map));
}

static void __init brcmstb_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	/*
	 * FIXME: In 3.8+ the bchip_* calls should go away; DT will tell us
	 * which peripherals are enabled
	 */
	bchip_set_features();

	printk(KERN_INFO "Options: moca=%d sata=%d pcie=%d usb=%d\n",
		brcm_moca_enabled, brcm_sata_enabled,
		brcm_pcie_enabled, brcm_usb_enabled);

	bchip_early_setup();
}

static void brcmstb_restart(char mode, const char *cmd)
{
	brcm_machine_restart(cmd);
}

void __init brcmstb_dt_init_irq(void)
{
	BDEV_WR(BCHP_IRQ0_IRQEN, BCHP_IRQ0_IRQEN_uarta_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartb_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartc_irqen_MASK
		);
	of_irq_init(brcmstb_dt_irq_match);
}

extern asmlinkage void brcmstb_handle_irq(struct pt_regs *regs);

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.map_io		= brcmstb_map_io,
	.init_irq	= brcmstb_dt_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &brcmstb_timer,
	.init_machine	= brcmstb_machine_init,
	.dt_compat	= brcmstb_match,
	.restart	= brcmstb_restart,
MACHINE_END
