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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <linux/brcmstb/brcmstb.h>
#include <linux/compiler.h>

/* FIXME: Read this from device tree */
#define BRCMSTB_UARTA_BASE	BPHYSADDR(BCHP_UARTA_REG_START)

#define __BRCMSTB_PERIPH_VIRT	0xfd000000
#define BRCMSTB_PERIPH_VIRT	((void __iomem *)__BRCMSTB_PERIPH_VIRT)
#define BRCMSTB_PERIPH_PHYS	BPHYSADDR(0)
#define BRCMSTB_PERIPH_LENGTH	SZ_32M

#define PHYS_TO_IO(x)		(((x) & 0x00ffffff) | __BRCMSTB_PERIPH_VIRT)

#ifdef __ASSEMBLY__
#define IO_ADDRESS(x)		PHYS_TO_IO((x))
#else
#define IO_ADDRESS(x)		(void __iomem __force *)(PHYS_TO_IO((x)))
#endif

#endif
