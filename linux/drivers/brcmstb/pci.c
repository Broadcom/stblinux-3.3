/*
 * Copyright (C) 2009 - 2012 Broadcom Corporation
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

#define pr_fmt(fmt)		"PCI: " fmt

#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/printk.h>
#include <linux/syscore_ops.h>
#include <linux/brcmstb/brcmstb.h>

/* NOTE: all PHYSICAL addresses */

/* Use assigned bus numbers so "ops" can tell the controllers apart */
#define BRCM_BUSNO_PCIE		0x00

#define PCIE_OUTBOUND_WIN(win, start, len) do { \
	BDEV_WR(BCHP_PCIE_MISC_CPU_2_PCIE_MEM_WIN##win##_LO, \
		(start) + MMIO_ENDIAN); \
	BDEV_WR(BCHP_PCIE_MISC_CPU_2_PCIE_MEM_WIN##win##_HI, 0); \
	BDEV_WR(BCHP_PCIE_MISC_CPU_2_PCIE_MEM_WIN##win##_BASE_LIMIT, \
		(((start) >> 20) << 4) | \
		 ((((start) + (len) - 1) >> 20) << 20)); \
	} while (0)

static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 *data);
static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 data);

static inline int get_busno_pcie(void)  { return BRCM_BUSNO_PCIE;  }

static struct pci_ops brcmstb_pci_ops = {
	.read = brcm_pci_read_config,
	.write = brcm_pci_write_config,
};

/*
 * Note: our PCIe core does not support IO BARs at all.  MEM only.
 * The Linux PCI code insists on having IO resources and io_map_base for
 * all PCI controllers, so those values are totally bogus.
 */
#define IO_ADDR_PCIE		0x400
#define PCIE_IO_SIZE		0x400
#define BOGUS_IO_MAP_BASE	1

/* pci_controller resources */

static struct resource pcie_mem_resource = {
	.name			= "External PCIe MEM",
	.start			= PCIE_MEM_START,
	.end			= PCIE_MEM_START + PCIE_MEM_SIZE - 1,
	.flags			= IORESOURCE_MEM,
};

static struct resource pcie_io_resource = {
	.name			= "External PCIe IO (unavailable)",
	.start			= IO_ADDR_PCIE,
	.end			= IO_ADDR_PCIE + PCIE_IO_SIZE - 1,
	.flags			= IORESOURCE_MEM,
};

static struct pci_controller brcmstb_pcie_controller = {
	.pci_ops		= &brcmstb_pci_ops,
	.io_resource		= &pcie_io_resource,
	.mem_resource		= &pcie_mem_resource,
	.get_busno		= &get_busno_pcie,
	.io_map_base		= BOGUS_IO_MAP_BASE,
};

struct brcm_pci_bus {
	struct pci_controller	*controller;
	char			*name;
	unsigned int		hw_busnum;
	unsigned int		busnum_shift;
	unsigned int		slot_shift;
	unsigned int		func_shift;
	int			memory_hole;
	unsigned long		idx_reg;
	unsigned long		data_reg;
	struct clk		*clk;
};

static struct brcm_pci_bus brcm_buses[] = {
	[BRCM_BUSNO_PCIE] = {
		&brcmstb_pcie_controller, "PCIe",    1, 20, 15, 12, 0 },
};


/***********************************************************************
 * PCIe Bridge setup
 ***********************************************************************/

#if defined(__BIG_ENDIAN)
#define	DATA_ENDIAN		2	/* PCI->DDR inbound accesses */
#define MMIO_ENDIAN		2	/* MIPS->PCI outbound accesses */
#else
#define	DATA_ENDIAN		0
#define MMIO_ENDIAN		0
#endif

static struct wktmr_time pcie_reset_started;

#define PCIE_LINK_UP() \
	(((BDEV_RD(BCHP_PCIE_MISC_PCIE_STATUS) & 0x30) == 0x30) ? 1 : 0)

void brcm_early_pcie_setup(void)
{
	/*
	 * Called from bchip_early_setup() in order to start PCIe link
	 * negotiation immediately at kernel boot time.  The RC is supposed
	 * to give the endpoint device 100ms to settle down before
	 * attempting configuration accesses.  So we let the link negotiation
	 * happen in the background instead of busy-waiting.
	 */

	struct wktmr_time tmp;

	/* reset the bridge and the endpoint device */
	BDEV_WR_F_RB(HIF_RGR1_SW_INIT_1, PCIE_BRIDGE_SW_INIT, 1);
	BDEV_WR_F_RB(HIF_RGR1_SW_INIT_1, PCIE_SW_PERST, 1);

	/* delay 100us */
	wktmr_read(&tmp);
	while (wktmr_elapsed(&tmp) < (100 * WKTMR_1US))
		;

	/* take the bridge out of reset */
	BDEV_WR_F_RB(HIF_RGR1_SW_INIT_1, PCIE_BRIDGE_SW_INIT, 0);

	/* enable SCB_MAX_BURST_SIZE | CSR_READ_UR_MODE | SCB_ACCESS_EN */
	BDEV_WR(BCHP_PCIE_MISC_MISC_CTRL, 0x00103000);

	/* set up MIPS->PCIE memory windows (4x 128MB) */
	PCIE_OUTBOUND_WIN(0, PCIE_MEM_START + 0x00000000, 0x08000000);
	PCIE_OUTBOUND_WIN(1, PCIE_MEM_START + 0x08000000, 0x08000000);
	PCIE_OUTBOUND_WIN(2, PCIE_MEM_START + 0x10000000, 0x08000000);
	PCIE_OUTBOUND_WIN(3, PCIE_MEM_START + 0x18000000, 0x08000000);

#if defined(CONFIG_BRCM_HAS_2GB_MEMC0)
	/* set up 4GB PCIE->SCB memory window on BAR2 */
	BDEV_WR(BCHP_PCIE_MISC_RC_BAR2_CONFIG_LO, 0x00000011);
	BDEV_WR(BCHP_PCIE_MISC_RC_BAR2_CONFIG_HI, 0x00000000);
	BDEV_WR_F(PCIE_MISC_MISC_CTRL, SCB0_SIZE, 0x10);
	BDEV_WR_F(PCIE_MISC_MISC_CTRL, SCB1_SIZE, 0x0f);
#else
	/* set up 1GB PCIE->SCB memory window on BAR2 */
	BDEV_WR(BCHP_PCIE_MISC_RC_BAR2_CONFIG_LO, 0x0000000f);
	BDEV_WR(BCHP_PCIE_MISC_RC_BAR2_CONFIG_HI, 0x00000000);
#endif

	/* disable PCIE->GISB window */
	BDEV_WR(BCHP_PCIE_MISC_RC_BAR1_CONFIG_LO, 0x00000000);
	/* disable the other PCIE->SCB memory window */
	BDEV_WR(BCHP_PCIE_MISC_RC_BAR3_CONFIG_LO, 0x00000000);

	/* disable MSI (for now...) */
	BDEV_WR(BCHP_PCIE_MISC_MSI_BAR_CONFIG_LO, 0);

	/* set up L2 interrupt masks */
	BDEV_WR_RB(BCHP_PCIE_INTR2_CPU_CLEAR, 0);
	BDEV_WR_RB(BCHP_PCIE_INTR2_CPU_MASK_CLEAR, 0);
	BDEV_WR_RB(BCHP_PCIE_INTR2_CPU_MASK_SET, 0xffffffff);

	/* take the EP device out of reset */
	BDEV_WR_F_RB(HIF_RGR1_SW_INIT_1, PCIE_SW_PERST, 0);

	/* record the current time */
	wktmr_read(&pcie_reset_started);
}

void brcm_setup_pcie_bridge(void)
{
	/* give the RC/EP time to wake up, before trying to configure RC */
	while (wktmr_elapsed(&pcie_reset_started) < (100 * WKTMR_1MS))
		;

	if (!PCIE_LINK_UP()) {
		struct clk *clk;

		brcm_pcie_enabled = 0;
		clk = brcm_buses[BRCM_BUSNO_PCIE].clk;
		if (clk) {
			clk_disable(clk);
			clk_put(clk);
		}
		pr_info("PCIe link down\n");
		return;
	}
	pr_info("PCIe link up, %sGbps x%lu\n",
		BDEV_RD_F(PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL,
			  NEG_LINK_SPEED) == 0x2 ? "5.0" : "2.5",
		BDEV_RD_F(PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL,
			  NEG_LINK_WIDTH));

	/* enable MEM_SPACE and BUS_MASTER for RC */
	BDEV_WR(BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND, 0x6);

	/* set base/limit for outbound transactions */
#if defined(CONFIG_BRCM_HAS_2GB_MEMC0)
	BDEV_WR(BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT, 0xeff0d000);
#else
	BDEV_WR(BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT, 0xbff0a000);
#endif

	/* disable the prefetch range */
	BDEV_WR(BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT, 0x0000fff0);

	/* set pri/sec bus numbers */
	BDEV_WR(BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO, 0x00010100);

	/* enable configuration request retry (see pci_scan_device()) */
	BDEV_WR_F(PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL, RC_CRS_EN, 1);

	/* PCIE->SCB endian mode for BAR2 */
	BDEV_WR_F_RB(PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, ENDIAN_MODE_BAR2,
		DATA_ENDIAN);
}

#if defined(CONFIG_PM)
/*
 * syscore device to handle PCIe bus suspend and resume
 */
static inline void pcie_enable(int enable)
{
	struct clk *clk;

	if (!brcm_pcie_enabled)
		return;

	clk = brcm_buses[BRCM_BUSNO_PCIE].clk;
	if (clk)
		enable ? clk_enable(clk) : clk_disable(clk);
}

static int pcie_suspend(void)
{
	pcie_enable(0);
	return 0;
}

static void pcie_resume(void)
{
	pcie_enable(1);
}

static struct syscore_ops pcie_pm_ops = {
	.suspend        = pcie_suspend,
	.resume         = pcie_resume,
};
#endif

/***********************************************************************
 * PCI controller registration
 ***********************************************************************/

static int __init brcmstb_pci_init(void)
{
	if (brcm_pcie_enabled) {
		brcm_buses[BRCM_BUSNO_PCIE].clk = clk_get(NULL, "pcie");
		brcm_setup_pcie_bridge();
		brcm_buses[BRCM_BUSNO_PCIE].idx_reg =
			BVIRTADDR(BCHP_PCIE_EXT_CFG_PCIE_EXT_CFG_INDEX);
		brcm_buses[BRCM_BUSNO_PCIE].data_reg =
			BVIRTADDR(BCHP_PCIE_EXT_CFG_PCIE_EXT_CFG_DATA);

		register_pci_controller(&brcmstb_pcie_controller);
#if defined(CONFIG_PM)
		register_syscore_ops(&pcie_pm_ops);
#endif
	}
	return 0;
}

arch_initcall(brcmstb_pci_init);

/***********************************************************************
 * Read/write PCI configuration registers
 ***********************************************************************/

#define CFG_INDEX(bus, devfn, reg) \
	(((PCI_SLOT(devfn) & 0x1f) << (brcm_buses[bus->number].slot_shift)) | \
	 ((PCI_FUNC(devfn) & 0x07) << (brcm_buses[bus->number].func_shift)) | \
	 (brcm_buses[bus->number].hw_busnum << \
	  brcm_buses[bus->number].busnum_shift) | \
	 (reg))

static int devfn_ok(struct pci_bus *bus, unsigned int devfn)
{
	/* PCIe: check for link down or invalid slot number */
	if (bus->number == BRCM_BUSNO_PCIE &&
	    (!PCIE_LINK_UP() || PCI_SLOT(devfn) != 0))
		return 0;

	return 1;	/* OK */
}

static int brcm_pci_write_config(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 data)
{
	u32 val = 0, mask, shift;

	if (!devfn_ok(bus, devfn))
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	BUG_ON(((where & 3) + size) > 4);

	if (size < 4) {
		/* partial word - read, modify, write */
		DEV_WR_RB(brcm_buses[bus->number].idx_reg,
			CFG_INDEX(bus, devfn, where & ~3));
		val = DEV_RD(brcm_buses[bus->number].data_reg);
	}

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;

	DEV_WR_RB(brcm_buses[bus->number].idx_reg,
		CFG_INDEX(bus, devfn, where & ~3));
	val = (val & ~mask) | ((data << shift) & mask);
	DEV_WR_RB(brcm_buses[bus->number].data_reg, val);

	return PCIBIOS_SUCCESSFUL;
}

static int brcm_pci_read_config(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 *data)
{
	u32 val, mask, shift;

	if (!devfn_ok(bus, devfn))
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	BUG_ON(((where & 3) + size) > 4);

	DEV_WR_RB(brcm_buses[bus->number].idx_reg,
		CFG_INDEX(bus, devfn, where & ~3));
	val = DEV_RD(brcm_buses[bus->number].data_reg);

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;

	*data = (val & mask) >> shift;
	return PCIBIOS_SUCCESSFUL;
}

/***********************************************************************
 * PCI slot to IRQ mappings (aka "fixup")
 ***********************************************************************/

int __devinit pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	if (dev->bus->number == BRCM_BUSNO_PCIE) {
		const int pcie_irq[] = {
			BRCM_IRQ_PCIE_INTA, BRCM_IRQ_PCIE_INTB,
			BRCM_IRQ_PCIE_INTC, BRCM_IRQ_PCIE_INTD,
		};
		if ((pin - 1) > 3)
			return 0;
		return pcie_irq[pin - 1];
	}
	return 0;
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}


/***********************************************************************
 * Per-device initialization
 ***********************************************************************/

static void __devinit brcm_pcibios_fixup(struct pci_dev *dev)
{
	int slot = PCI_SLOT(dev->devfn);

	pr_info("found device %04x:%04x on %s bus, slot %d (irq %d)\n",
		dev->vendor, dev->device, brcm_buses[dev->bus->number].name,
		slot, pcibios_map_irq(dev, slot, 1));

	/* zero out the BARs and let Linux assign an address */
	pci_write_config_dword(dev, PCI_COMMAND, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_1, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_2, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_3, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_4, 0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_5, 0);
	pci_write_config_dword(dev, PCI_INTERRUPT_LINE, 0);
}

DECLARE_PCI_FIXUP_EARLY(PCI_ANY_ID, PCI_ANY_ID, brcm_pcibios_fixup);

/***********************************************************************
 * DMA address remapping
 ***********************************************************************/

/*
 * PCIe inbound BARs collapse the "holes" in the chip's SCB address space.
 * Therefore the PCI addresses need to be adjusted as they will not match
 * the SCB addresses (MIPS physical addresses).
 *
 * The address maps can be found in <asm/mach-brcmstb/spaces.h> .
 */

static int dev_collapses_memory_hole(struct device *dev)
{
#if defined(CONFIG_BRCM_UPPER_MEMORY) || defined(CONFIG_HIGHMEM)

	struct pci_dev *pdev;

	if (unlikely(dev == NULL) ||
	    likely(dev->bus != &pci_bus_type))
		return 0;

	pdev = to_pci_dev(dev);
	if (unlikely(pdev == NULL) ||
	    brcm_buses[pdev->bus->number].memory_hole)
		return 0;

	return 1;
#else
	return 0;
#endif
}

static dma_addr_t brcm_phys_to_pci(struct device *dev, unsigned long phys)
{
	if (!dev_collapses_memory_hole(dev))
		return phys;
	if (phys >= MEMC1_START)
		return phys - MEMC1_PCI_OFFSET;
	if (phys >= (BRCM_PCI_HOLE_START + BRCM_PCI_HOLE_SIZE))
		return phys - BRCM_PCI_HOLE_SIZE;
	return phys;
}

dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size)
{
	return brcm_phys_to_pci(dev, virt_to_phys(addr));
}

dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page)
{
	return brcm_phys_to_pci(dev, page_to_phys(page));
}

unsigned long plat_dma_addr_to_phys(struct device *dev, dma_addr_t dma_addr)
{
	if (!dev_collapses_memory_hole(dev))
		return dma_addr;
	if (dma_addr >= (MEMC1_START - MEMC1_PCI_OFFSET))
		return dma_addr + MEMC1_PCI_OFFSET;
	if (dma_addr >= BRCM_PCI_HOLE_START)
		return dma_addr + BRCM_PCI_HOLE_SIZE;
	return dma_addr;
}
