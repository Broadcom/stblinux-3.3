/*
 * sdhci-brcmstb.c Support for SDHCI on Broadcom SoC's
 *
 * Author: Al Cooper <acooper@broadcom.com>
 * Based on sdhci-dove.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/brcmstb/brcmstb.h>

#include "sdhci-pltfm.h"
#include "sdhci-brcmstb.h"

#define SDIO_CFG_REG(x, y)	(x + BCHP_SDIO_0_CFG_##y - \
				 BCHP_SDIO_0_CFG_REG_START)
#define SDIO_CFG_SET(base, reg, mask) do {				\
		BDEV_SET(SDIO_CFG_REG(base, reg),			\
			 BCHP_SDIO_0_CFG_##reg##_##mask##_MASK);	\
	} while (0)
#define SDIO_CFG_UNSET(base, reg, mask) do {				\
		BDEV_UNSET(SDIO_CFG_REG(base, reg),			\
			   BCHP_SDIO_0_CFG_##reg##_##mask##_MASK);	\
	} while (0)
#define SDIO_CFG_FIELD(base, reg, field, val) do {			\
		BDEV_UNSET(SDIO_CFG_REG(base, reg),			\
			   BCHP_SDIO_0_CFG_##reg##_##field##_MASK);	\
		BDEV_SET(SDIO_CFG_REG(base, reg),			\
		 val << BCHP_SDIO_0_CFG_##reg##_##field##_SHIFT);	\
	} while (0)

#define SDHCI_OVERRIDE_OPTIONS_NONE		0x00000000
#define SDHCI_OVERRIDE_OPTIONS_UHS_SDR50	0x00000001
#define SDHCI_OVERRIDE_OPTIONS_TUNING		0x00000002

#define CAP0_SHIFT(field) BCHP_SDIO_0_CFG_CAP_REG0_##field##_SHIFT
#define CAP1_SHIFT(field) BCHP_SDIO_0_CFG_CAP_REG1_##field##_SHIFT

static inline void sdhci_override_caps(uintptr_t cfg_base, int base_clock,
				       int timeout_clock, int options)
{
	uint32_t val;

	/* Set default for every field with all options off */
	val = (0 << CAP0_SHIFT(DDR50_SUPPORT) |			\
	       0 << CAP0_SHIFT(SD104_SUPPORT) |			\
	       0 << CAP0_SHIFT(SDR50) |				\
	       0 << CAP0_SHIFT(SLOT_TYPE) |			\
	       0 << CAP0_SHIFT(ASYNCH_INT_SUPPORT) |		\
	       0 << CAP0_SHIFT(64B_SYS_BUS_SUPPORT) |		\
	       0 << CAP0_SHIFT(1_8V_SUPPORT) |			\
	       0 << CAP0_SHIFT(3_0V_SUPPORT) |			\
	       1 << CAP0_SHIFT(3_3V_SUPPORT) |			\
	       1 << CAP0_SHIFT(SUSP_RES_SUPPORT) |		\
	       1 << CAP0_SHIFT(SDMA_SUPPORT) |			\
	       1 << CAP0_SHIFT(HIGH_SPEED_SUPPORT) |		\
	       1 << CAP0_SHIFT(ADMA2_SUPPORT) |			\
	       1 << CAP0_SHIFT(EXTENDED_MEDIA_SUPPORT) |	\
	       1 << CAP0_SHIFT(MAX_BL) |			\
	       0 << CAP0_SHIFT(BASE_FREQ) |			\
	       1 << CAP0_SHIFT(TIMEOUT_CLK_UNIT) |		\
	       0 << CAP0_SHIFT(TIMEOUT_FREQ));

	val |= (base_clock << CAP0_SHIFT(BASE_FREQ));
	val |= (timeout_clock << CAP0_SHIFT(TIMEOUT_FREQ));
	if (options & SDHCI_OVERRIDE_OPTIONS_UHS_SDR50)
		val |= (1 << CAP0_SHIFT(SDR50)) |
			(1 << CAP0_SHIFT(1_8V_SUPPORT));
	BDEV_WR(SDIO_CFG_REG(cfg_base, CAP_REG0), val);

	val = (1 << CAP1_SHIFT(CAP_REG_OVERRIDE) |	\
	       0 << CAP1_SHIFT(SPI_BLK_MODE) |		\
	       0 << CAP1_SHIFT(SPI_MODE) |		\
	       0 << CAP1_SHIFT(CLK_MULT) |		\
	       0 << CAP1_SHIFT(RETUNING_MODES) |	\
	       0 << CAP1_SHIFT(USE_TUNING) |		\
	       0 << CAP1_SHIFT(RETUNING_TIMER) |	\
	       0 << CAP1_SHIFT(Driver_D_SUPPORT) |	\
	       0 << CAP1_SHIFT(Driver_C_SUPPORT) |	\
	       0 << CAP1_SHIFT(Driver_A_SUPPORT));
	BDEV_WR(SDIO_CFG_REG(cfg_base, CAP_REG1), val);
}

static int sdhci_brcmstb_config(struct platform_device *pdev)
{
	struct resource *iomem;
	uintptr_t cfg_base;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!iomem)
		return -ENOMEM;
	cfg_base = iomem->start;
	if (BDEV_RD(SDIO_CFG_REG(cfg_base, SCRATCH)) & 0x01) {
		dev_info(&pdev->dev, "Disabled by bootloader\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "Enabling controller\n");
	BDEV_UNSET(SDIO_CFG_REG(cfg_base, SDIO_EMMC_CTRL1), 0xf000);
	BDEV_UNSET(SDIO_CFG_REG(cfg_base, SDIO_EMMC_CTRL2), 0x00ff);

	/*
	 * This is broken on all chips and defaults to enabled on
	 * some chips so disable it.
	 */
	SDIO_CFG_UNSET(cfg_base, SDIO_EMMC_CTRL1, SCB_SEQ_EN);

#ifdef CONFIG_CPU_LITTLE_ENDIAN
	/* FRAME_NHW | BUFFER_ABO */
	BDEV_SET(SDIO_CFG_REG(cfg_base, SDIO_EMMC_CTRL1), 0x3000);
#else
	/* WORD_ABO | FRAME_NBO | FRAME_NHW */
	BDEV_SET(SDIO_CFG_REG(cfg_base, SDIO_EMMC_CTRL1), 0xe000);
	/* address swap only */
	BDEV_SET(SDIO_CFG_REG(cfg_base, SDIO_EMMC_CTRL2), 0x0050);
#endif

#if defined(CONFIG_BCM7231B0) || defined(CONFIG_BCM7346B0)
	SDIO_CFG_SET(cfg_base, CAP_REG1, CAP_REG_OVERRIDE);
#elif defined(CONFIG_BCM7344B0)
	SDIO_CFG_SET(cfg_base, CAP_REG0, HIGH_SPEED_SUPPORT);
	SDIO_CFG_SET(cfg_base, CAP_REG1, CAP_REG_OVERRIDE);
#elif defined(CONFIG_BCM7425)
	/*
	 * HW7425-1352: Disable TUNING because it's broken.
	 * Use manual input and output clock delays to work around
	 * 7425B2 timing issues.
	 */
	if (BRCM_CHIP_REV() == 0x12) {
		/* disable tuning */
		sdhci_override_caps(cfg_base, 100, 50,
				    SDHCI_OVERRIDE_OPTIONS_UHS_SDR50);

		/* enable input delay, resolution = 1, value = 8 */
		SDIO_CFG_FIELD(cfg_base, IP_DLY, IP_TAP_DELAY, 8);
		SDIO_CFG_FIELD(cfg_base, IP_DLY, IP_DELAY_CTRL, 1);
		SDIO_CFG_SET(cfg_base, IP_DLY, IP_TAP_EN);

		/* enable output delay */
		SDIO_CFG_FIELD(cfg_base, OP_DLY, OP_TAP_DELAY, 4);
		SDIO_CFG_FIELD(cfg_base, OP_DLY, OP_DELAY_CTRL, 3);
		SDIO_CFG_SET(cfg_base, OP_DLY, OP_TAP_EN);

		/* Use the manual clock delay */
		SDIO_CFG_FIELD(cfg_base, SD_CLOCK_DELAY, INPUT_CLOCK_DELAY, 8);
	}
#elif defined(CONFIG_BCM7563A0)
	sdhci_override_caps(cfg_base, 50, 50, SDHCI_OVERRIDE_OPTIONS_NONE);
#elif defined(CONFIG_BCM7584A0)
	/* enable output delay */
	SDIO_CFG_FIELD(cfg_base, OP_DLY, OP_TAP_DELAY, 4);
	SDIO_CFG_FIELD(cfg_base, OP_DLY, OP_DELAY_CTRL, 3);
	SDIO_CFG_SET(cfg_base, OP_DLY, OP_TAP_EN);
#endif
	return 0;
}

static int sdhci_brcmstb_supported(void)
{
	/* Chips with broken SDIO - 7429A0, 7435A0, 7425B0 and 7425B1 */
	if ((BRCM_CHIP_ID() == 0x7425) &&
	    ((BRCM_CHIP_REV() == 0x10) || (BRCM_CHIP_REV() == 0x11)))
		return 0;
	if ((BRCM_CHIP_ID() == 0x7429) && (BRCM_CHIP_REV() == 0x00))
		return 0;
	if ((BRCM_CHIP_ID() == 0x7435) && (BRCM_CHIP_REV() == 0x00))
		return 0;
	return 1;
}


static u32 sdhci_brcmstb_readl(struct sdhci_host *host, int reg)
{
	return __raw_readl(host->ioaddr + reg);
}

static void sdhci_brcmstb_writel(struct sdhci_host *host, u32 val, int reg)
{
	__raw_writel(val, host->ioaddr + reg);
}

static u16 sdhci_brcmstb_readw(struct sdhci_host *host, int reg)
{
	return __raw_readw(host->ioaddr + reg);
}

static void sdhci_brcmstb_writew(struct sdhci_host *host, u16 val, int reg)
{
#if defined(CONFIG_BCM7425B0)
	if (reg == SDHCI_HOST_CONTROL2 && BRCM_CHIP_REV() == 0x12) {
		/* HW7425-1414: I/O voltage uses Power Control Register (29h) */
		int pow_reg = __raw_readb(host->ioaddr + SDHCI_POWER_CONTROL);

		pow_reg &= ~SDHCI_POWER_330;
		if (val & SDHCI_CTRL_VDD_180)
			pow_reg |= SDHCI_POWER_180;
		else
			pow_reg |= SDHCI_POWER_330;
		__raw_writeb(pow_reg, host->ioaddr + SDHCI_POWER_CONTROL);
	}
#endif
	__raw_writew(val, host->ioaddr + reg);
}

static struct sdhci_ops sdhci_brcmstb_ops = {
	.read_w		= sdhci_brcmstb_readw,
	.write_w	= sdhci_brcmstb_writew,
	.read_l		= sdhci_brcmstb_readl,
	.write_l	= sdhci_brcmstb_writel,
};

static struct sdhci_pltfm_data sdhci_brcmstb_pltfm_data = {
	.ops	= &sdhci_brcmstb_ops,
	.quirks2 = SDHCI_QUIRK2_NO_1_8_V
};


static int __devinit sdhci_brcmstb_probe(struct platform_device *pdev)
{
	int res;
	struct sdhci_brcmstb_pdata *pdata = pdev->dev.platform_data;

	if (!sdhci_brcmstb_supported()) {
		dev_info(&pdev->dev, "Disabled, unsupported chip revision\n");
		return -ENODEV;
	}
	if (sdhci_brcmstb_config(pdev) != 0)
		return -ENODEV;
	res = sdhci_pltfm_register(pdev, &sdhci_brcmstb_pltfm_data);
	if (!res) {
		struct sdhci_host *host = platform_get_drvdata(pdev);
		host->mmc->caps |= pdata->caps;
		host->mmc->caps2 |= pdata->caps2;
	}
	return res;
}

static int __devexit sdhci_brcmstb_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

#ifdef CONFIG_PM
static int sdhci_brcmstb_suspend(struct device *dev)
{
	int ret = 0;

	if (sdhci_pltfm_pmops.suspend)
		ret = sdhci_pltfm_pmops.suspend(dev);
	return ret;
}

static int sdhci_brcmstb_resume(struct device *dev)
{
	int ret = 0;

	sdhci_brcmstb_config(to_platform_device(dev));
	if (sdhci_pltfm_pmops.resume)
		ret = sdhci_pltfm_pmops.resume(dev);
	return ret;
}

const struct dev_pm_ops sdhci_brcmstb_pmops = {
	.suspend	= sdhci_brcmstb_suspend,
	.resume		= sdhci_brcmstb_resume,
};
#endif

static struct platform_driver sdhci_brcmstb_driver = {
	.driver		= {
		.name	= "sdhci-brcmstb",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm		= &sdhci_brcmstb_pmops,
#endif
	},
	.probe		= sdhci_brcmstb_probe,
	.remove		= __devexit_p(sdhci_brcmstb_remove),
};

module_platform_driver(sdhci_brcmstb_driver);

MODULE_DESCRIPTION("SDHCI driver for Broadcom");
MODULE_AUTHOR("Al Cooper <acooper@broadcom.com>");
MODULE_LICENSE("GPL v2");
