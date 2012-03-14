/*
 * Copyright (C) 2009 Broadcom Corporation
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

#ifndef __ASM_MACH_BRCMSTB_IRQ_H
#define __ASM_MACH_BRCMSTB_IRQ_H

/*
 * This file has IRQ definitions that DO NOT depend on the BCHP headers
 * See also: linux/brcmstb/brcmstb.h
 */

/* IRQs 1-128 are used for HIF L1 */
#define BRCM_VIRTIRQ_BASE	(1 + 128)

/* virtual (non-L1) IRQs */

#define BRCM_CPUIRQ_BASE	(BRCM_VIRTIRQ_BASE + 0)
#define BRCM_OTHERIRQ_BASE	(BRCM_VIRTIRQ_BASE + 8)

/* MIPS interrupts 0-7 */
#define BRCM_IRQ_CPU0		(BRCM_CPUIRQ_BASE + 0)
#define BRCM_IRQ_CPU1		(BRCM_CPUIRQ_BASE + 1)
#define BRCM_IRQ_CPU2		(BRCM_CPUIRQ_BASE + 2)
#define BRCM_IRQ_CPU3		(BRCM_CPUIRQ_BASE + 3)
#define BRCM_IRQ_CPU4		(BRCM_CPUIRQ_BASE + 4)
#define BRCM_IRQ_CPU5		(BRCM_CPUIRQ_BASE + 5)
#define BRCM_IRQ_CPU6		(BRCM_CPUIRQ_BASE + 6)
#define BRCM_IRQ_CPU7		(BRCM_CPUIRQ_BASE + 7)

#define BRCM_IRQ_IPI0		BRCM_IRQ_CPU0
#define BRCM_IRQ_IPI1		BRCM_IRQ_CPU1
#define BRCM_IRQ_HW0		BRCM_IRQ_CPU2
#define BRCM_IRQ_HW1		BRCM_IRQ_CPU3

/* performance counter */

#define BRCM_IRQ_PERF		(BRCM_OTHERIRQ_BASE + 0)

#define NR_IRQS			160
#define MIPS_CPU_IRQ_BASE	BRCM_CPUIRQ_BASE

#endif /* __ASM_MACH_BRCMSTB_IRQ_H */
