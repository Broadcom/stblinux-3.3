/*
 * Copyright (C) 2000-2004 Russell King
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

#include <linux/bootmem.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/compiler.h>
#include <linux/atomic.h>
#include <linux/printk.h>
#include <linux/module.h>

#include <asm/page.h>
#include <asm/pgtable-32.h>
#include <asm/pgtable-bits.h>
#include <asm/addrspace.h>
#include <asm/tlbflush.h>
#include <asm/r4kcache.h>
#include <linux/brcmstb/brcmstb.h>

#include <spaces.h>

/*
 * Override default behavior to allow cached access to all valid DRAM ranges
 */
int __uncached_access(struct file *file, unsigned long addr)
{
	if (file->f_flags & O_SYNC)
		return 1;
	if (addr >= BCHP_PHYSICAL_OFFSET && addr < UPPERMEM_START)
		return 1;
	if (addr >= PCIE_MEM_START)
		return 1;
	return 0;
}

/***********************************************************************
 * Wired TLB mappings for upper memory support
 ***********************************************************************/

#define UNIQUE_ENTRYHI(idx) (CKSEG0 + ((idx) << (PAGE_SHIFT + 1)))

/* (PFN << 6) | GLOBAL | VALID | DIRTY | cacheability */
#define ENTRYLO_CACHED(paddr)	(((paddr) >> 6) | (0x07) | (0x03 << 3))
#define ENTRYLO_UNCACHED(paddr)	(((paddr) >> 6) | (0x07) | (0x02 << 3))

/* GLOBAL | !VALID */
#define ENTRYLO_INVALID()	(0x01)

struct tlb_entry {
	unsigned long entrylo0;
	unsigned long entrylo1;
	unsigned long entryhi;
	unsigned long pagemask;
};

static struct tlb_entry __maybe_unused uppermem_mappings[] = {
{
#if defined(CONFIG_BRCM_UPPER_768MB)
	.entrylo0		= ENTRYLO_CACHED(TLB_UPPERMEM_PA),
	.entrylo1		= ENTRYLO_INVALID(),
	.entryhi		= TLB_UPPERMEM_VA,
	.pagemask		= PM_256M,
#endif
},
};

static inline void __cpuinit brcm_write_tlb_entry(int idx,
	unsigned long entrylo0, unsigned long entrylo1, unsigned long entryhi,
	unsigned long pagemask)
{
	write_c0_entrylo0(entrylo0);
	write_c0_entrylo1(entrylo1);
	write_c0_entryhi(entryhi);
	write_c0_pagemask(pagemask);
	write_c0_index(idx);
	mtc0_tlbw_hazard();
	tlb_write_indexed();
	tlbw_use_hazard();
}

/*
 * This function is used instead of add_wired_entry(), because it does not
 * have any external dependencies and is not marked __init
 */
static inline void __cpuinit brcm_add_wired_entry(unsigned long entrylo0,
	unsigned long entrylo1, unsigned long entryhi, unsigned long pagemask)
{
	int i = read_c0_wired();
	write_c0_wired(i + 1);
	brcm_write_tlb_entry(i, entrylo0, entrylo1, entryhi, pagemask);
}

extern void tlb_init(void);
extern void build_tlb_refill_handler(void);

void __cpuinit brcm_tlb_init(void)
{
#ifdef CONFIG_BRCM_UPPER_MEMORY
	if (smp_processor_id() == 0) {
		int i;
		struct tlb_entry *e = uppermem_mappings;

		tlb_init();
		for (i = 0; i < ARRAY_SIZE(uppermem_mappings); i++, e++)
			brcm_add_wired_entry(e->entrylo0, e->entrylo1,
				e->entryhi, e->pagemask);
		write_c0_pagemask(PM_DEFAULT_MASK);
	} else {
		/* bypass tlb_init() / probe_tlb() for secondary CPU */
		cpu_data[smp_processor_id()].tlbsize = cpu_data[0].tlbsize;
		build_tlb_refill_handler();
	}
#else
	tlb_init();
#endif
}

/*
 * Initialize upper memory TLB entries
 *
 * On TP1 this must happen before we set up $sp/$gp .  It is always
 * possible for stacks, task_structs, thread_info's, and other
 * important structures to be allocated out of upper memory so
 * this happens early on.
 */
asmlinkage void plat_wired_tlb_setup(void)
{
#ifdef CONFIG_BRCM_UPPER_MEMORY
	int i, tlbsz;

	/* Flush TLB.  local_flush_tlb_all() is not available yet. */
	write_c0_entrylo0(0);
	write_c0_entrylo1(0);
	write_c0_pagemask(PM_DEFAULT_MASK);
	write_c0_wired(0);

	tlbsz = (read_c0_config1() >> 25) & 0x3f;
	for (i = 0; i <= tlbsz; i++) {
		write_c0_entryhi(UNIQUE_ENTRYHI(i));
		write_c0_index(i);
		mtc0_tlbw_hazard();
		tlb_write_indexed();
		tlbw_use_hazard();
	}

	write_c0_wired(0);
	mtc0_tlbw_hazard();

	for (i = 0; i < ARRAY_SIZE(uppermem_mappings); i++) {
		struct tlb_entry *e = &uppermem_mappings[i];
		brcm_add_wired_entry(e->entrylo0, e->entrylo1, e->entryhi,
			e->pagemask);
	}

	write_c0_pagemask(PM_DEFAULT_MASK);
#endif
}

#ifdef CONFIG_BRCM_UPPER_768MB

/***********************************************************************
 * Special allocator for coherent (uncached) memory
 * (Required for >256MB upper memory)
 ***********************************************************************/

#define CONSISTENT_DMA_SIZE	(CONSISTENT_END - CONSISTENT_BASE)
#define CONSISTENT_OFFSET(x)	(((unsigned long)(x) - CONSISTENT_BASE) >> \
	PAGE_SHIFT)
#define CONSISTENT_PTE_INDEX(x) (((unsigned long)(x) - CONSISTENT_BASE) >> \
	PGDIR_SHIFT)
#define NUM_CONSISTENT_PTES	(CONSISTENT_DMA_SIZE >> PGDIR_SHIFT)

/*
 * These are the page tables (4MB each) covering uncached, DMA consistent
 * allocations
 */
static pte_t *consistent_pte[NUM_CONSISTENT_PTES];
static DEFINE_SPINLOCK(consistent_lock);

struct arm_vm_region {
	struct list_head	vm_list;
	unsigned long		vm_start;
	unsigned long		vm_end;
	void			*vm_cac_va;
	int			vm_active;
};

static struct arm_vm_region consistent_head = {
	.vm_list	= LIST_HEAD_INIT(consistent_head.vm_list),
	.vm_start	= CONSISTENT_BASE,
	.vm_end		= CONSISTENT_END,
};

static struct arm_vm_region *
arm_vm_region_alloc(struct arm_vm_region *head, size_t size, gfp_t gfp)
{
	unsigned long addr = head->vm_start, end = head->vm_end - size;
	unsigned long flags;
	struct arm_vm_region *c, *new;

	new = kmalloc(sizeof(struct arm_vm_region), gfp);
	if (!new)
		goto out;

	spin_lock_irqsave(&consistent_lock, flags);

	list_for_each_entry(c, &head->vm_list, vm_list) {
		if ((addr + size) < addr)
			goto nospc;
		if ((addr + size) <= c->vm_start)
			goto found;
		addr = c->vm_end;
		if (addr > end)
			goto nospc;
	}

found:
	/*
	 * Insert this entry _before_ the one we found.
	 */
	list_add_tail(&new->vm_list, &c->vm_list);
	new->vm_start = addr;
	new->vm_end = addr + size;
	new->vm_active = 1;

	spin_unlock_irqrestore(&consistent_lock, flags);
	return new;

nospc:
	spin_unlock_irqrestore(&consistent_lock, flags);
	kfree(new);
out:
	return NULL;
}

static struct arm_vm_region *arm_vm_region_find(struct arm_vm_region *head,
	unsigned long addr)
{
	struct arm_vm_region *c;

	list_for_each_entry(c, &head->vm_list, vm_list) {
		if (c->vm_active && c->vm_start == addr)
			goto out;
	}
	c = NULL;
out:
	return c;
}

static int __init consistent_init(void)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	int ret = 0, i = 0;
	u32 base = CONSISTENT_BASE;

	do {
		pgd = pgd_offset(&init_mm, base);
		pud = pud_alloc(&init_mm, pgd, base);
		if (!pud) {
			pr_err("%s: no pud tables\n", __func__);
			ret = -ENOMEM;
			break;
		}
		pmd = pmd_alloc(&init_mm, pud, base);
		if (!pmd) {
			pr_err("%s: no pmd tables\n", __func__);
			ret = -ENOMEM;
			break;
		}

		pte = pte_alloc_kernel(pmd, base);
		if (!pte) {
			pr_err("%s: no pte tables\n", __func__);
			ret = -ENOMEM;
			break;
		}

		consistent_pte[i++] = pte;
		base += (1 << PGDIR_SHIFT);
	} while (base < CONSISTENT_END);

	return ret;
}

core_initcall(consistent_init);

int brcm_map_coherent(dma_addr_t dma_handle, void *cac_va, size_t size,
	void **uncac_va, gfp_t gfp)
{
	struct arm_vm_region *c;
	struct page *page;
	pte_t *pte;
	int idx;
	u32 off;

	c = arm_vm_region_alloc(&consistent_head, size, gfp);
	if (!c)
		return -EINVAL;

	c->vm_cac_va = cac_va;

	page = virt_to_page(cac_va);
	idx = CONSISTENT_PTE_INDEX(c->vm_start);
	off = CONSISTENT_OFFSET(c->vm_start) & (PTRS_PER_PTE-1);
	pte = consistent_pte[idx] + off;

	pr_debug("map addr %08lx idx %x off %x pte %p\n",
		c->vm_start, idx, off, pte);

	do {
		if (off >= PTRS_PER_PTE) {
			off = 0;
			BUG_ON(idx >= NUM_CONSISTENT_PTES - 1);
			pte = consistent_pte[++idx];
		}

		BUG_ON(!pte_none(*pte));
		set_pte(pte, mk_pte(page, PAGE_KERNEL_UNCACHED));
		page++;
		pte++;
		off++;
	} while (size -= PAGE_SIZE);

	*uncac_va = (void *)c->vm_start;
	return 0;
}

void *brcm_unmap_coherent(void *vaddr)
{
	struct arm_vm_region *c;
	unsigned long flags, addr;
	void *ret = NULL;
	pte_t *pte;
	int idx;
	u32 off;

	spin_lock_irqsave(&consistent_lock, flags);
	c = arm_vm_region_find(&consistent_head, (unsigned long)vaddr);
	if (!c) {
		spin_unlock_irqrestore(&consistent_lock, flags);
		pr_err("%s: invalid VA %p\n", __func__, vaddr);
		return NULL;
	}
	c->vm_active = 0;
	spin_unlock_irqrestore(&consistent_lock, flags);

	ret = c->vm_cac_va;
	addr = c->vm_start;

	idx = CONSISTENT_PTE_INDEX(addr);
	off = CONSISTENT_OFFSET(addr) & (PTRS_PER_PTE-1);
	pte = consistent_pte[idx] + off;

	pr_debug("unmap addr %08lx idx %x off %x pte %p\n",
		addr, idx, off, pte);

	do {
		if (off >= PTRS_PER_PTE) {
			off = 0;
			BUG_ON(idx >= NUM_CONSISTENT_PTES - 1);
			pte = consistent_pte[++idx];
		}

		pte_clear(&init_mm, addr, pte);
		pte++;
		off++;
		addr += PAGE_SIZE;
	} while (addr < c->vm_end);
	flush_tlb_kernel_range(c->vm_start, c->vm_end);

	spin_lock_irqsave(&consistent_lock, flags);
	list_del(&c->vm_list);
	spin_unlock_irqrestore(&consistent_lock, flags);

	kfree(c);

	return ret;
}

#endif /* CONFIG_BRCM_UPPER_768MB */

void __iomem *plat_ioremap(phys_t offset, unsigned long size,
	unsigned long flags)
{
	/* sanity check */
	if ((offset + size - 1) < offset ||
	    !size ||
	    offset > max(KSEG0_SIZE, KSEG1_SIZE))
		return NULL;

	/* !XKS01, XKS01: uncached access to EBI/registers @ PA 1000_0000 */
	if (offset >= 0x10000000 &&
	    (offset + size) <= 0x20000000 &&
	    flags == _CACHE_UNCACHED)
		return (void *)(KSEG1 + offset);

	/* !XKS01, XKS01: easy cached access to some DRAM */
	if ((offset + size) <= KSEG0_SIZE &&
	    flags == _CACHE_CACHABLE_NONCOHERENT)
		return (void *)(KSEG0 + offset);

	/* !XKS01 only: easy uncached access to some DRAM */
	if ((offset + size) <= KSEG1_SIZE &&
	    flags == _CACHE_UNCACHED)
		return (void *)(KSEG1 + offset);

	/* anything else gets mapped using page tables */
	return NULL;
}
EXPORT_SYMBOL(plat_ioremap);

int plat_iounmap(const volatile void __iomem *addr)
{
	phys_t va = (unsigned long)addr;

	if (va >= KSEG0 && va < (KSEG0 + KSEG0_SIZE))
		return 1;
	if (va >= KSEG1 && va < (KSEG1 + KSEG1_SIZE))
		return 1;
	return 0;
}
EXPORT_SYMBOL(plat_iounmap);

/***********************************************************************
 * RAM configuration
 ***********************************************************************/

#define MAGIC0		0xdeadbeef
#define MAGIC1		0xfeedcafe

static inline unsigned int __init probe_ram_size(void)
{
	unsigned long addr = KSEG1, taddr;
	uint32_t olddata;
	unsigned long flags;
	unsigned int i, memsize = 256;

	pr_info("Probing system memory size... ");

	local_irq_save(flags);
	cache_op(Hit_Writeback_Inv_D, KSEG0);
	olddata = DEV_RD(addr);

	/*
	 * Try to figure out where memory wraps around.  If it does not
	 * wrap, assume 256MB
	*/
	for (i = 4; i <= 128; i <<= 1) {
		taddr = KSEG1 + i * 1048576;
		DEV_WR(addr, MAGIC0);
		if (DEV_RD(taddr) == MAGIC0) {
			DEV_WR(addr, MAGIC1);
			if (DEV_RD(taddr) == MAGIC1) {
				memsize = i;
				break;
			}
		}
	}

	DEV_WR(addr, olddata);
	cache_op(Hit_Writeback_Inv_D, KSEG0);
	local_irq_restore(flags);

	pr_cont("found %u MB\n", memsize);

	return memsize;
}

void __init board_get_ram_size(unsigned long *dram0_mb, unsigned long *dram1_mb)
{
#if defined(CONFIG_BRCM_OVERRIDE_RAM_SIZE)
	*dram0_mb = CONFIG_BRCM_FORCED_DRAM0_SIZE;
#if defined(CONFIG_BRCM_FORCED_DRAM1_SIZE)
	*dram1_mb = CONFIG_BRCM_FORCED_DRAM1_SIZE;
#endif
	pr_info("Using %lu MB + %lu MB RAM (from kernel configuration)\n",
		*dram0_mb, *dram1_mb);
#else
	/* DRAM0_SIZE variable from CFE */
	if (*dram0_mb) {
		pr_info("Using %lu MB + %lu MB RAM (from CFE)\n",
			*dram0_mb, *dram1_mb);
		return;
	}
	*dram0_mb = probe_ram_size();
#endif
}

static void __init __brcm_wraparound_check(unsigned long start,
	unsigned long midpoint, int memc_no)
{
	int found = 0, idx = read_c0_wired();
	unsigned long *a, *b, old_a, va = FIXADDR_TOP - 0x2000;

	brcm_write_tlb_entry(idx, ENTRYLO_UNCACHED(start),
		ENTRYLO_UNCACHED(midpoint), va, PM_4K);

	a = (unsigned long *)(va);
	b = (unsigned long *)(va + 0x1000);

	old_a = *a;
	mb();

	*b ^= 0x55555555;
	mb();

	if (*a != old_a)
		found = 1;
	mb();

	*b ^= 0x55555555;
	mb();

	/* this should always match, but check anyway */
	if (*a != old_a)
		found = 1;

	brcm_write_tlb_entry(idx, ENTRYLO_INVALID(), ENTRYLO_INVALID(),
		UNIQUE_ENTRYHI(idx), PM_4K);

	if (found)
		panic("DRAM%d wraparound detected at 0x%lx\n",
			memc_no, midpoint);
}

void __init brcm_wraparound_check(void)
{
	/*
	 * Find the middle of the DRAM region, compensating for the memory
	 * hole at the 256MB mark if necessary.  If there is no wraparound
	 * at the middle address, everything else should be OK.
	 */
	if (brcm_dram0_size_mb >= 512)
		__brcm_wraparound_check(0,
			(brcm_dram0_size_mb << 19) + 0x10000000, 0);
	else
		__brcm_wraparound_check(0, brcm_dram0_size_mb << 19, 0);

	if (brcm_dram1_size_mb)
		__brcm_wraparound_check(MEMC1_START,
			brcm_dram1_size_mb << 19, 1);
}
