/*
 * Copyright (C) 2012 Broadcom Corporation
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
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/compiler.h>

#include <asm/cpu-info.h>
#include <asm/mipsregs.h>
#include <asm/barrier.h>
#include <asm/cacheflush.h>
#include <asm/r4kcache.h>
#include <asm/asm-offsets.h>
#include <asm/inst.h>
#include <asm/fpu.h>
#include <asm/hazards.h>
#include <asm/cpu-features.h>
#include <linux/brcmstb/brcmstb.h>
#include <dma-coherence.h>

/***********************************************************************
 * MIPS features, caches, and bus interface
 ***********************************************************************/

void brcmstb_cpu_setup(void)
{
#if   defined(CONFIG_CPU_BMIPS3300)

	unsigned long cbr = __BMIPS_GET_CBR();

	/* Set BIU to async mode */
	set_c0_brcm_bus_pll(BIT(22));
	__sync();

#ifdef BCHP_MISB_BRIDGE_WG_MODE_N_TIMEOUT
	/* Enable write gathering */
	BDEV_WR_RB(BCHP_MISB_BRIDGE_WG_MODE_N_TIMEOUT, 0x264);

	/* Enable split mode */
	BDEV_WR_RB(BCHP_MISB_BRIDGE_MISB_SPLIT_MODE, 0x1);
	__sync();
#endif

	/* put the BIU back in sync mode */
	clear_c0_brcm_bus_pll(BIT(22));

	/* clear BHTD to enable branch history table */
	clear_c0_brcm_reset(BIT(16));

	/* Flush and enable RAC */
	DEV_WR_RB(cbr + BMIPS_RAC_CONFIG, 0x100);
	DEV_WR_RB(cbr + BMIPS_RAC_CONFIG, 0xf);
	DEV_WR_RB(cbr + BMIPS_RAC_ADDRESS_RANGE, 0x0fff0000);

#elif defined(CONFIG_CPU_BMIPS4380)

	unsigned long cbr = __BMIPS_GET_CBR();

	/* CRBMIPS438X-164: CBG workaround */
	switch (read_c0_prid()) {
	case 0x2a040:
	case 0x2a042:
	case 0x2a044:
	case 0x2a060:
		DEV_UNSET(cbr + BMIPS_L2_CONFIG, 0x07000000);
	}

	/* clear BHTD to enable branch history table */
	clear_c0_brcm_config_0(BIT(21));

	/* XI/ROTR enable */
	if (kernel_uses_smartmips_rixi) {
		set_c0_brcm_config_0(BIT(23));
		set_c0_brcm_cmt_ctrl(BIT(15));
	}

#elif defined(CONFIG_CPU_BMIPS5000)

	/* enable RDHWR, BRDHWR */
	set_c0_brcm_config(BIT(17) | BIT(21));

	if (kernel_uses_smartmips_rixi) {
		/* XI enable */
		set_c0_brcm_config(BIT(27));

		/* enable MIPS32R2 ROR instruction for XI TLB handlers */
		__asm__ __volatile__(
		"	li	$8, 0x5a455048\n"
		"	.word	0x4088b00f\n"	/* mtc0 $8, $22, 15 */
		"	nop; nop; nop\n"
		"	.word	0x4008b008\n"	/* mfc0 $8, $22, 8 */
		"	lui	$9, 0x0100\n"
		"	or	$8, $9\n"
		"	.word	0x4088b008\n"	/* mtc0 $8, $22, 8 */
		: : : "$8", "$9");
	}

#endif
}

/***********************************************************************
 * Simulate privileged instructions (RDHWR, MFC0) and unaligned accesses
 ***********************************************************************/

#define OPCODE 0xfc000000
#define BASE   0x03e00000
#define RT     0x001f0000
#define OFFSET 0x0000ffff
#define LL     0xc0000000
#define SC     0xe0000000
#define SPEC0  0x00000000
#define SPEC3  0x7c000000
#define RD     0x0000f800
#define FUNC   0x0000003f
#define SYNC   0x0000000f
#define RDHWR  0x0000003b

#define BRDHWR 0xec000000
#define OP_MFC0 0x40000000

int brcm_simulate_opcode(struct pt_regs *regs, unsigned int opcode)
{
	struct thread_info *ti = task_thread_info(current);
	int rd = (opcode & RD) >> 11;
	int rt = (opcode & RT) >> 16;

	/* PR34054: use alternate RDHWR instruction encoding */
	if (((opcode & OPCODE) == BRDHWR && (opcode & FUNC) == RDHWR)
	    || ((opcode & OPCODE) == SPEC3 && (opcode & FUNC) == RDHWR)) {

		if (rd == 29) {
			regs->regs[rt] = ti->tp_value;
			atomic_inc(&brcm_rdhwr_count);
			return 0;
		}
	}

	/* emulate MFC0 $15 for optimized memcpy() CPU detection */
	if ((opcode & OPCODE) == OP_MFC0 &&
	    (opcode & OFFSET) == (15 << 11)) {
		regs->regs[rt] = read_c0_prid();
		return 0;
	}

	return -1;	/* unhandled */
}

int brcm_unaligned_fp(void __user *addr, union mips_instruction *insn,
	struct pt_regs *regs)
{
	unsigned int op = insn->i_format.opcode;
	unsigned int rt = insn->i_format.rt;
	unsigned int res;
	int wordlen = 8;

	/* on r4k, only the even slots ($f0, $f2, ...) are used */
	u8 *fprptr = (u8 *)current + THREAD_FPR0 + (rt >> 1) *
		(THREAD_FPR2 - THREAD_FPR0);

	if (op == lwc1_op || op == swc1_op) {
		wordlen = 4;
#ifdef __LITTLE_ENDIAN
		/* LE: LSW ($f0) precedes MSW ($f1) */
		fprptr += (rt & 1) ? 4 : 0;
#else
		/* BE: MSW ($f1) precedes LSW ($f0) */
		fprptr += (rt & 1) ? 0 : 4;
#endif
	}

	preempt_disable();
	if (is_fpu_owner())
		save_fp(current);
	else
		own_fpu(1);

	if (op == lwc1_op || op == ldc1_op) {
		if (!access_ok(VERIFY_READ, addr, wordlen))
			goto sigbus;
		/*
		 * FPR load: copy from user struct to kernel saved
		 * register struct, then restore all FPRs
		 */
		__asm__ __volatile__ (
		"1:	lb	%0, 0(%3)\n"
		"	sb	%0, 0(%2)\n"
		"	addiu	%2, 1\n"
		"	addiu	%3, 1\n"
		"	addiu	%1, -1\n"
		"	bnez	%1, 1b\n"
		"	li	%0, 0\n"
		"3:\n"
		"	.section .fixup,\"ax\"\n"
		"4:	li	%0, %4\n"
		"	j	3b\n"
		"	.previous\n"
		"	.section __ex_table,\"a\"\n"
		STR(PTR)" 1b,4b\n"
		"	.previous\n"
			: "=&r" (res), "+r" (wordlen),
			  "+r" (fprptr), "+r" (addr)
			: "i" (-EFAULT));
		if (res)
			goto fault;

		restore_fp(current);
	} else {
		if (!access_ok(VERIFY_WRITE, addr, wordlen))
			goto sigbus;
		/*
		 * FPR store: copy from kernel saved register struct
		 * to user struct
		 */
		__asm__ __volatile__ (
		"2:	lb	%0, 0(%2)\n"
		"1:	sb	%0, 0(%3)\n"
		"	addiu	%2, 1\n"
		"	addiu	%3, 1\n"
		"	addiu	%1, -1\n"
		"	bnez	%1, 2b\n"
		"	li	%0, 0\n"
		"3:\n"
		"	.section .fixup,\"ax\"\n"
		"4:	li	%0, %4\n"
		"	j	3b\n"
		"	.previous\n"
		"	.section __ex_table,\"a\"\n"
		STR(PTR)" 1b,4b\n"
		"	.previous\n"
			: "=&r" (res), "+r" (wordlen),
			  "+r" (fprptr), "+r" (addr)
			: "i" (-EFAULT));
		if (res)
			goto fault;
	}
	preempt_enable();

	atomic_inc(&brcm_unaligned_fp_count);
	return 0;

sigbus:
	preempt_enable();
	return -EINVAL;

fault:
	preempt_enable();
	return -EFAULT;
}

/***********************************************************************
 * CPU divisor / PLL manipulation
 ***********************************************************************/
/*
 * 0: CP0 COUNT/COMPARE frequency depends on divisor
 * 1: CP0 COUNT/COMPARE frequency does not depend on divisor
 */
static int fixed_counter_freq;

#if defined(CONFIG_BCM7425B0) || defined(CONFIG_BCM7344B0) || \
	defined(CONFIG_BCM7346B0)
/* SWLINUX-2063: MIPS cannot enter divide-by-N mode */
#define	BROKEN_MIPS_DIVIDER
#endif

/* MIPS active standby on 7550 */
#define CPU_PLL_MODE1		216000

/*
 * current ADJUSTED base frequency (reflects the current PLL settings)
 * brcm_cpu_khz (in time.c) always has the ORIGINAL clock frequency and
 *   is never changed after bootup
 */
unsigned long brcm_adj_cpu_khz;

/* multiplier used in brcm_fixup_ticks to scale the # of ticks
 * 0               - no fixup needed
 * any other value - factor * 2^16 */
static unsigned long fixup_ticks_ratio;

/* current CPU divisor, as set by the user */
static __maybe_unused int cpu_div = 1;

/*
 * MIPS clockevent code always assumes the original boot-time CP0 clock rate.
 * This function scales the number of ticks according to the current HW
 * settings.
 */
unsigned long brcm_fixup_ticks(unsigned long delta)
{
	unsigned long long tmp = delta;

	if (unlikely(!brcm_adj_cpu_khz))
		brcm_adj_cpu_khz = brcm_cpu_khz;

	if (likely(!fixup_ticks_ratio))
		return delta;

	tmp *= fixup_ticks_ratio;
	tmp >>= 16;

	return (unsigned long)tmp;
}

static unsigned int orig_udelay_val[NR_CPUS];

struct spd_change {
	int			old_div;
	int			new_div;
	int			old_base;
	int			new_base;
};

void brcm_set_cpu_speed(void *arg)
{
	struct spd_change *c = arg;
	uint32_t new_div = (uint32_t)c->new_div;
	unsigned long __maybe_unused count, compare, delta;
	signed long sdelta;
	int cpu = smp_processor_id();
	uint32_t __maybe_unused tmp0, tmp1, tmp2, tmp3;

	/* scale udelay_val */
	if (!orig_udelay_val[cpu])
		orig_udelay_val[cpu] = current_cpu_data.udelay_val;

	if (c->new_base == brcm_cpu_khz)
		current_cpu_data.udelay_val = orig_udelay_val[cpu] / new_div;
	else
		current_cpu_data.udelay_val =
			(unsigned long long)orig_udelay_val[cpu] *
			c->new_base / (new_div * c->old_base);

	/* scale any pending timer events */
	compare = read_c0_compare();
	count = read_c0_count();

	sdelta = (long)compare - (long)count;
	if (sdelta > 0) {
		if (!fixed_counter_freq)
			delta = ((unsigned long long)sdelta *
				c->old_div * c->new_base) /
				(new_div * c->old_base);
		else
			delta = ((unsigned long long)sdelta *
				c->new_base) / c->old_base;
	write_c0_compare(read_c0_count() + delta);
	}

	if (cpu != 0)
		return;

#if defined(CONFIG_BRCM_CPU_PLL)
	brcm_adj_cpu_khz = c->new_base;
#if defined(CONFIG_BCM7550)
	if (brcm_adj_cpu_khz == CPU_PLL_MODE1) {
		/* 216Mhz */
		BDEV_WR_RB(BCHP_VCXO_CTL_CONFIG_FSM_PLL_NEXT_CFG_3A,
			0x801b2806);
		BDEV_WR_RB(BCHP_VCXO_CTL_CONFIG_FSM_PLL_NEXT_CFG_3B,
			0x00300618);
	} else {
		/* 324Mhz */
		BDEV_WR_RB(BCHP_VCXO_CTL_CONFIG_FSM_PLL_NEXT_CFG_3A,
			0x801b2806);
		BDEV_WR_RB(BCHP_VCXO_CTL_CONFIG_FSM_PLL_NEXT_CFG_3B,
			0x00300418);
	}
	BDEV_WR_RB(BCHP_VCXO_CTL_CONFIG_FSM_PLL_UPDATE, 1);
#else
#error CPU PLL adjustment not supported on this chip
#endif
#endif

	if ((brcm_adj_cpu_khz == brcm_cpu_khz) &&
	    (fixed_counter_freq || new_div == 1)) {
		fixup_ticks_ratio = 0;
	} else {
		fixup_ticks_ratio =
			((unsigned long long)brcm_adj_cpu_khz << 16) /
			 (unsigned long long)brcm_cpu_khz;
		if (!fixed_counter_freq)
			fixup_ticks_ratio /= new_div;
	}

	printk(KERN_DEBUG "ratio=%lu adj=%lu freq=%lu new_div=%d\n",
	       fixup_ticks_ratio, brcm_adj_cpu_khz, brcm_cpu_khz, new_div);
	new_div = ffs(new_div) - 1;

	/* see BMIPS datasheet, CP0 register $22 */

#if defined(CONFIG_CPU_BMIPS3300)
	change_c0_brcm_bus_pll(0x07 << 22, (new_div << 23) | (0 << 22));
#elif defined(CONFIG_CPU_BMIPS5000)
	change_c0_brcm_mode(0x0f << 4, (1 << 7) | (new_div << 4));
#elif defined(CONFIG_CPU_BMIPS4380)
	__asm__ __volatile__(
	"	.set	push\n"
	"	.set	noreorder\n"
	"	.set	nomacro\n"
	"	.set	mips32\n"
	/* get kseg1 address for CBA into %3 */
	"	mfc0	%3, $22, 6\n"
	"	li	%2, 0xfffc0000\n"
	"	and	%3, %2\n"
	"	li	%2, 0xa0000000\n"
	"	add	%3, %2\n"
	/* %1 = async bit, %2 = mask out everything but 30:28 */
	"	lui	%1, 0x1000\n"
	"	lui	%2, 0x8fff\n"
	"	beqz	%0, 1f\n"
	"	ori	%2, 0xffff\n"
	/* handle SYNC to ASYNC */
	"	sync\n"
	"	mfc0	%4, $22, 5\n"
	"	and	%4, %2\n"
	"	or	%4, %1\n"
	"	mtc0	%4, $22, 5\n"
	"	nop\n"
	"	nop\n"
	"	lw	%2, 4(%3)\n"
	"	sw	%2, 4(%3)\n"
	"	sync\n"
	"	sll	%0, 29\n"
	"	or	%4, %0\n"
	"	mtc0	%4, $22, 5\n"
	"	nop; nop; nop; nop\n"
	"	nop; nop; nop; nop\n"
	"	nop; nop; nop; nop\n"
	"	nop; nop; nop; nop\n"
	"	b	2f\n"
	"	nop\n"
	/* handle ASYNC to SYNC */
	"1:\n"
	"	mfc0	%4, $22, 5\n"
	"	and	%4, %2\n"
	"	or	%4, %1\n"
	"	mtc0	%4, $22, 5\n"
	"	nop; nop; nop; nop\n"
	"	nop; nop; nop; nop\n"
	"	sync\n"
	"	and	%4, %2\n"
	"	mtc0	%4, $22, 5\n"
	"	nop\n"
	"	nop\n"
	"	lw	%2, 4(%3)\n"
	"	sw	%2, 4(%3)\n"
	"	sync\n"
	"2:\n"
	"	.set	pop\n"
	: "+r" (new_div),
	  "=&r" (tmp0), "=&r" (tmp1), "=&r" (tmp2), "=&r" (tmp3));
#endif
}

#ifdef CONFIG_BRCM_CPU_DIV

ssize_t brcm_pm_show_cpu_div(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cpu_div);
}

ssize_t brcm_pm_store_cpu_div(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct spd_change chg;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (val != 1 && val != 2 && val != 4 && val != 8
#if defined(CONFIG_CPU_BMIPS5000)
		&& val != 16
#endif
			)
		return -EINVAL;

#if defined(BROKEN_MIPS_DIVIDER)
	return val == 1 ? count : -EINVAL;
#endif

	chg.old_div = cpu_div;
	chg.new_div = val;
	chg.old_base = brcm_adj_cpu_khz;
	chg.new_base = brcm_adj_cpu_khz;

	on_each_cpu(brcm_set_cpu_speed, &chg, 1);
	cpu_div = val;
	return count;
}

#endif /* CONFIG_BRCM_CPU_DIV */

#ifdef CONFIG_BRCM_CPU_PLL

static int cpu_pll_mode;

ssize_t brcm_pm_show_cpu_pll(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cpu_pll_mode);
}

ssize_t brcm_pm_store_cpu_pll(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct spd_change chg;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (cpu_pll_mode == val)
		return count;

	switch (val) {
	case 0:
		chg.new_base = brcm_cpu_khz;
		break;
	case 1:
		chg.new_base = CPU_PLL_MODE1;
		break;
	default:
		return -EINVAL;
	}

	chg.old_div = cpu_div;
	chg.new_div = cpu_div;
	chg.old_base = brcm_adj_cpu_khz;
	on_each_cpu(brcm_set_cpu_speed, &chg, 1);

	cpu_pll_mode = val;
	return count;
}

#endif /* CONFIG_BRCM_CPU_PLL */

static int bmips_check_caps(void)
{
	unsigned long __maybe_unused config;
#ifdef CONFIG_CPU_BMIPS5000
	fixed_counter_freq = 1;
#elif defined(CONFIG_CPU_BMIPS4380)
	config = read_c0_brcm_config();
	fixed_counter_freq = !!(config & 0x40);
#else
	fixed_counter_freq = 0;
#endif
	printk(KERN_INFO "PM: CP0 COUNT/COMPARE frequency %s on divisor\n",
	       fixed_counter_freq ? "does not depend" : "depends");
	return 0;
}
late_initcall(bmips_check_caps);
