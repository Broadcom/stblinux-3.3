
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/bitops.h>

#include <asm/exception.h>

#include <mach/irqs.h>

static const void __iomem *base = (void *)0xee41a400;

#define IRQBIT(irq)		BIT((irq) & 0x1f)
#define IRQWD(irq)		(base + (((irq) >> 5) << 2))

void brcmstb_mask_irq(struct irq_data *d)
{
	__raw_writel(IRQBIT(d->irq), IRQWD(d->irq) + 0x18);
}

void brcmstb_unmask_irq(struct irq_data *d)
{
	__raw_writel(IRQBIT(d->irq), IRQWD(d->irq) + 0x24);
}

static struct irq_chip brcmstb_internal_irq_chip = {
	.name		= "HIF_L1",
	.irq_ack	= brcmstb_mask_irq,
	.irq_mask	= brcmstb_mask_irq,
	.irq_unmask	= brcmstb_unmask_irq,
};

#define do_IRQ(x)		handle_IRQ((x) - 1, regs)
#define flip_tp(x)		do { } while (0)

asmlinkage void __exception_irq_entry brcmstb_handle_irq(struct pt_regs *regs)
{
	u32 pend, shift;

	pend = __raw_readl(IRQWD(0) + 0x00) & ~__raw_readl(IRQWD(0) + 0x0c);
	while ((shift = ffs(pend)) != 0) {
		pend ^= (1 << (shift - 1));
		do_IRQ(shift);
		flip_tp(shift);
	}

	pend = __raw_readl(IRQWD(32) + 0x00) & ~__raw_readl(IRQWD(32) + 0x0c);
	while ((shift = ffs(pend)) != 0) {
		pend ^= (1 << (shift - 1));
		shift += 32;
		do_IRQ(shift);
		flip_tp(shift);
	}

	pend = __raw_readl(IRQWD(64) + 0x00) & ~__raw_readl(IRQWD(64) + 0x0c);
	while ((shift = ffs(pend)) != 0) {
		pend ^= (1 << (shift - 1));
		shift += 64;
		do_IRQ(shift);
		flip_tp(shift);
	}
}

#define BPHYSADDR(x)		((x) + 0xe0000000)

static struct resource irq_resource = {
	.name	= "HIF_L1",
	.start	= BPHYSADDR(0x0041a400),
	.end	= BPHYSADDR(0x0041a42f),
};

void __init brcmstb_init_irq(void)
{
	int i, n;

	request_resource(&iomem_resource, &irq_resource);

	for (n = 0; n < NR_IRQS; n += 32) {
		__raw_writel(0xffffffff, IRQWD(n) + 0x18);

		for (i = n; (i < (n + 32)) && (i < NR_IRQS); i++) {
			irq_set_chip_and_handler(i, &brcmstb_internal_irq_chip,
						 handle_level_irq);
			//irq_set_chip_data(i, base);
			set_irq_flags(i, IRQF_VALID);
		}
	}
}

