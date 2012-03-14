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

#include <linux/init.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/compiler.h>
#include <linux/brcmstb/brcmstb.h>

#include "op_impl.h"

/*
 * Events coming from opcontrol look like: 0xXYZZ
 *   X = ModuleID
 *   Y = SetID
 *   ZZ = EventID
 *
 * ModuleID and SetID are set globally so they must match for all events.
 * Exception: events with ModuleID 0 can be used regardless of the
 * ModuleID / SetID settings.
 */

#define BMIPS_MOD_SET(event)		(((event) >> 8) & 0xff)
#define BMIPS_MOD(event)		(((event) >> 12) & 0x0f)
#define BMIPS_SET(event)		(((event) >> 8) & 0x03)
#define BMIPS_EVENT(event)		((event) & 0x7f)

/* Default to using TP0 (HW can only profile one thread at a time) */
#define PMU_TP				0

#define NUM_COUNTERS			4

#define GLOB_ENABLE			0x80000000
#define GLOB_SET_SHIFT			0
#define GLOB_MOD_SHIFT			2

#define CTRL_ENABLE			0x8001
#define CTRL_EVENT_SHIFT		2

#define CNTR_OVERFLOW			0x80000000

#if defined(CONFIG_CPU_BMIPS3300)

/* These registers do not conform to any known MIPS standard */
#define r_perfcntr0()			__read_32bit_c0_register($25, 0)
#define r_perfcntr1()			__read_32bit_c0_register($25, 1)
#define r_perfcntr2()			__read_32bit_c0_register($25, 2)
#define r_perfcntr3()			__read_32bit_c0_register($25, 3)

#define w_perfcntr0(x)			__write_32bit_c0_register($25, 0, x)
#define w_perfcntr1(x)			__write_32bit_c0_register($25, 1, x)
#define w_perfcntr2(x)			__write_32bit_c0_register($25, 2, x)
#define w_perfcntr3(x)			__write_32bit_c0_register($25, 3, x)

#define w_perfctrl0(x)			__write_32bit_c0_register($25, 4, x)
#define w_perfctrl1(x)			__write_32bit_c0_register($25, 5, x)

#define r_glob()			__read_32bit_c0_register($25, 6)
#define w_glob(x)			__write_32bit_c0_register($25, 6, x)

#elif defined(CONFIG_CPU_BMIPS4380)

static unsigned long bmips_cbr;

#define PERF_RD(x)			DEV_RD(bmips_cbr + BMIPS_PERF_ ## x)
#define PERF_WR(x, y)			DEV_WR_RB(bmips_cbr + \
						BMIPS_PERF_ ## x, (y))

#define r_perfcntr0()			PERF_RD(COUNTER_0)
#define r_perfcntr1()			PERF_RD(COUNTER_1)
#define r_perfcntr2()			PERF_RD(COUNTER_2)
#define r_perfcntr3()			PERF_RD(COUNTER_3)

#define w_perfcntr0(x)			PERF_WR(COUNTER_0, (x))
#define w_perfcntr1(x)			PERF_WR(COUNTER_1, (x))
#define w_perfcntr2(x)			PERF_WR(COUNTER_2, (x))
#define w_perfcntr3(x)			PERF_WR(COUNTER_3, (x))

#define w_perfctrl0(x)			PERF_WR(CONTROL_0, (x))
#define w_perfctrl1(x)			PERF_WR(CONTROL_1, (x))

#define r_glob()			PERF_RD(GLOBAL_CONTROL)
#define w_glob(x)			PERF_WR(GLOBAL_CONTROL, (x))

#endif

static int (*save_perf_irq)(void);

struct op_mips_model op_model_bmips_ops;

static struct bmips_register_config {
	unsigned int globctrl;
	unsigned int control[4];
	unsigned int counter[4];
} reg;

/* Compute all of the registers in preparation for enabling profiling.  */

static void bmips_reg_setup(struct op_counter_config *ctr)
{
	int i;
	unsigned int mod_set = 0;
	unsigned int ev;

	memset(&reg, 0, sizeof(reg));
	reg.globctrl = GLOB_ENABLE;

	for (i = 0; i < NUM_COUNTERS; i++) {
		if (!ctr[i].enabled)
			continue;
		ev = ctr[i].event;

		if (mod_set && BMIPS_MOD_SET(ev) &&
				mod_set != BMIPS_MOD_SET(ev)) {
			printk(KERN_WARNING "%s: profiling event 0x%x "
				"conflicts with another event, disabling\n",
				__FUNCTION__, ev);
			continue;
		}
		reg.counter[i] = ctr[i].count;
		reg.control[i] = CTRL_ENABLE |
			(BMIPS_EVENT(ev) << CTRL_EVENT_SHIFT);
		if (BMIPS_MOD_SET(ev))
			mod_set = BMIPS_MOD_SET(ev);
		reg.globctrl |= (BMIPS_MOD(ev) << GLOB_MOD_SHIFT) |
			(BMIPS_SET(ev) << GLOB_SET_SHIFT);
	}
}

/* Program all of the registers in preparation for enabling profiling.  */

static void bmips_cpu_setup(void *args)
{
	w_perfcntr0(reg.counter[0]);
	w_perfcntr1(reg.counter[1]);
	w_perfcntr2(reg.counter[2]);
	w_perfcntr3(reg.counter[3]);
}

static void bmips_cpu_start(void *args)
{
	w_perfctrl0(reg.control[0] | (reg.control[1] << 16));
	w_perfctrl1(reg.control[2] | (reg.control[3] << 16));
	w_glob(reg.globctrl);
}

static void bmips_cpu_stop(void *args)
{
	w_perfctrl0(0);
	w_perfctrl1(0);
}

static int bmips_perfcount_handler(void)
{
	int handled = IRQ_NONE;

	if (!(r_glob() & GLOB_ENABLE))
		return handled;

#define HANDLE_COUNTER(n) \
	if (r_perfcntr ## n() & CNTR_OVERFLOW) { \
		oprofile_add_sample(get_irq_regs(), n); \
		w_perfcntr ## n(reg.counter[n]); \
		handled = IRQ_HANDLED; \
	}

	HANDLE_COUNTER(0)
	HANDLE_COUNTER(1)
	HANDLE_COUNTER(2)
	HANDLE_COUNTER(3)

	if (handled == IRQ_HANDLED)
		bmips_cpu_start(NULL);

	return handled;
}

static void bmips_perf_reset(void)
{
#ifdef CONFIG_CPU_BMIPS4380
	bmips_cbr = __BMIPS_GET_CBR();
	change_c0_brcm_cmt_ctrl(0x3 << 30, PMU_TP << 30);
#endif

	w_glob(GLOB_ENABLE);
	w_perfctrl0(0);
	w_perfctrl1(0);
	w_perfcntr0(0);
	w_perfcntr1(0);
	w_perfcntr2(0);
	w_perfcntr3(0);
}

static int __init bmips_init(void)
{
	bmips_perf_reset();

	switch (current_cpu_type()) {
	case CPU_BMIPS3300:
		op_model_bmips_ops.cpu_type = "mips/bmips3300";
		break;
	case CPU_BMIPS4380:
		op_model_bmips_ops.cpu_type = "mips/bmips4380";
		break;
	default:
		BUG();
	}
	save_perf_irq = perf_irq;
	perf_irq = bmips_perfcount_handler;

	return 0;
}

static void bmips_exit(void)
{
	bmips_perf_reset();
	w_glob(0);
	perf_irq = save_perf_irq;
}

struct op_mips_model op_model_bmips_ops = {
	.reg_setup	= bmips_reg_setup,
	.cpu_setup	= bmips_cpu_setup,
	.init		= bmips_init,
	.exit		= bmips_exit,
	.cpu_start	= bmips_cpu_start,
	.cpu_stop	= bmips_cpu_stop,
	.num_counters	= 4
};
