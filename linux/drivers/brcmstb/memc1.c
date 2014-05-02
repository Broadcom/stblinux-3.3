/*
 * Copyright (C) 2011 Broadcom Corporation
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
#include <stdarg.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include <linux/brcmstb/brcmstb.h>

#if 1
#define DBG	printk
#else
#define DBG(...)		do { } while (0)
#endif

#if defined(BCHP_MEMC_DDR_1_SSPD_CMD)

#define MAX_CLIENT_INFO_NUM		NUM_MEMC_CLIENTS
#define MAX_DDR_PARAMS_NUM		16
#define MAX_DDR_APHY_PARAMS_NUM		16
#define MAX_ARB_PARAMS_NUM		2

#define MEMC_STATE_UNKNOWN		0
#define MEMC_STATE_ON			1
#define MEMC_STATE_OFF			2

static int  __brcm_pm_memc1_initialized(void);
static void __brcm_pm_memc1_clock_start(void);
static void __brcm_pm_memc1_clock_stop(void);
static int  __brcm_pm_memc1_clock_running(void);
static void brcm_pm_memc1_enable_idle_pad(void);
static void brcm_pm_memc1_disable_idle_pad(void);
static void brcm_pm_memc1_enable_plls(void);
static void brcm_pm_memc1_disable_plls(void);

struct memc_config {
	u32	client_info[MAX_CLIENT_INFO_NUM];
	u32	ddr23_aphy_params[MAX_DDR_APHY_PARAMS_NUM];
	u32	ddr_params[MAX_DDR_PARAMS_NUM];
	u32	arb_params[MAX_ARB_PARAMS_NUM];

	u32	vcdl[4];
	int	shmoo_value[8];

	int	shmoo_valid;
	int	valid;
	/* if CFE did not initialize non-primary MEMC we cannot touch it
	  because we do not have calibration capabilities.
	  _initalized_ and _clock_active_ are initially set to
	  0: unknown
	  The first time any method is called it will set _initialized_ to
	  one of the following values:
	  1: on
	  2: off - unusable
	 */
	int	initialized;
	int	clock_active;
};

static struct memc_config __maybe_unused memc1_config;

static void brcm_pm_memc1_sspd_control(int enable)
{
	if (enable) {
		BDEV_WR_F_RB(MEMC_DDR_1_SSPD_CMD, SSPD, 1);
		while (!BDEV_RD_F(MEMC_DDR_1_POWER_DOWN_STATUS, SSPD))
			udelay(1);
	} else {
		BDEV_WR_F_RB(MEMC_DDR_1_SSPD_CMD, SSPD, 0);
		while (BDEV_RD_F(MEMC_DDR_1_POWER_DOWN_STATUS, SSPD))
			udelay(1);
	}
}

static void brcm_pm_memc1_suspend_s3(void)
{
	/* clear standby field */
	BDEV_UNSET(BCHP_DDR40_PHY_CONTROL_REGS_1_STANDBY_CONTROL, 0x0f);
	/* inhibit DDR_RSTb pulse, Arm for standby */
	BDEV_SET(BCHP_DDR40_PHY_CONTROL_REGS_1_STANDBY_CONTROL, BIT(5) | 0x05);
}

static void brcm_pm_memc1_ddr_params(int restore)
{
	int ii = 0;

	if (restore) {
		/* program ddr iobuf registers */
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_2,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_3,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_5,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_0,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_1,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_0,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_1,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_2,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_3,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_4,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_CNTRLR_CONFIG,
			memc1_config.ddr_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_INIT_CNTRL,
			memc1_config.ddr_params[ii++]);
	} else {
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_2);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_3);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_5);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_0);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_1);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_0);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_1);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_2);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_3);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_4);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_CNTRLR_CONFIG);
		memc1_config.ddr_params[ii++] =
			BDEV_RD(BCHP_MEMC_DDR_1_DRAM_INIT_CNTRL);
	}
	BUG_ON(ii > MAX_DDR_PARAMS_NUM);
}

static void brcm_pm_memc1_arb_params(int restore)
{
	int ii = 0;

	if (restore) {
		BDEV_WR_RB(BCHP_MEMC_ARB_1_FULLNESS_THRESHOLD,
			memc1_config.arb_params[ii++]);
		BDEV_WR_RB(BCHP_MEMC_ARB_1_MINIMUM_COMMAND_SIZE,
			memc1_config.arb_params[ii++]);
	} else {
		memc1_config.arb_params[ii++] =
			BDEV_RD(BCHP_MEMC_ARB_1_FULLNESS_THRESHOLD);
		memc1_config.arb_params[ii++] =
			BDEV_RD(BCHP_MEMC_ARB_1_MINIMUM_COMMAND_SIZE);
	}

	BUG_ON(ii > MAX_ARB_PARAMS_NUM);
}

static int brcm_pm_memc1_clock_running(void)
{
	if (memc1_config.clock_active == MEMC_STATE_UNKNOWN) {
		/* do this only once */
		memc1_config.clock_active = __brcm_pm_memc1_clock_running() ?
			MEMC_STATE_ON : MEMC_STATE_OFF;
	}
	return memc1_config.clock_active == MEMC_STATE_ON;
}

static void brcm_pm_memc1_clock_start(void)
{
	if (!brcm_pm_memc1_clock_running())
		__brcm_pm_memc1_clock_start();
	memc1_config.clock_active = MEMC_STATE_ON;
}

static void brcm_pm_memc1_clock_stop(void)
{
	if (brcm_pm_memc1_clock_running())
		__brcm_pm_memc1_clock_stop();
	memc1_config.clock_active = MEMC_STATE_OFF;
}

static int brcm_pm_memc1_initialized(void)
{
	if (memc1_config.initialized == MEMC_STATE_UNKNOWN) {
		brcm_pm_memc1_clock_start();
		/* do this only once */
		memc1_config.initialized = __brcm_pm_memc1_initialized() ?
			MEMC_STATE_ON : MEMC_STATE_OFF;
	}
	return memc1_config.initialized == MEMC_STATE_ON;
}

#define CHECK_MEMC1_INIT() \
	if (!brcm_pm_memc1_initialized()) { \
		printk(KERN_ERR "%s: not initialized\n", __func__); \
		return -1; \
	}

int brcm_pm_memc1_suspend(int is_s3)
{
	CHECK_MEMC1_INIT();

	/*
	 * Force all dirty cache lines to be written
	 * If a dirty cache line for memc1 were to be written later,
	 * it might hang the cache unit and/or cpu.
	 */
	_dma_cache_wback_inv(0, ~0);
	if (is_s3)
		brcm_pm_memc1_suspend_s3();
	else
		brcm_pm_memc1_sspd_control(1);

	return 0;
}

int brcm_pm_memc1_resume(int is_s3)
{
	CHECK_MEMC1_INIT();

	if (is_s3 == 0)
		brcm_pm_memc1_sspd_control(0);

	return 0;
}


int brcm_pm_memc1_powerdown(void)
{
	CHECK_MEMC1_INIT();

	DBG(KERN_DEBUG "%s\n", __func__);

	brcm_pm_save_restore_rts(BCHP_MEMC_ARB_1_REG_START,
		memc1_config.client_info, 0);
	brcm_pm_memc1_ddr_params(0);
	brcm_pm_memc1_arb_params(0);
	brcm_pm_memc1_enable_idle_pad();

	memc1_config.shmoo_valid = 1;
	memc1_config.valid = 1;

	/*
	 * Force all dirty cache lines to be written
	 * If a dirty cache line for memc1 were to be written later,
	 * it might hang the cache unit and/or cpu.
	 */
	_dma_cache_wback_inv(0, ~0);
	brcm_pm_memc1_sspd_control(1);

	brcm_pm_memc1_disable_plls();

	DBG(KERN_DEBUG "%s reset\n", __func__);
	BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET, MEMC_DRAM_INIT, 1);
	BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET, MEMC_CORE, 1);
	BDEV_WR_F_RB(MEMC_DDR_1_DRAM_INIT_CNTRL, DDR3_INIT_MODE, 1);
	mdelay(1);

	/* Stop the clocks */
	brcm_pm_memc1_clock_stop();
	memc1_config.valid = 1;

	return 0;
}

int brcm_pm_memc1_powerup(void)
{
	CHECK_MEMC1_INIT();

	if (!memc1_config.valid || !memc1_config.shmoo_valid) {
		printk(KERN_ERR "%s: no valid saved configuration %d %d\n",
		       __func__, memc1_config.valid, memc1_config.shmoo_valid);
		return -1;
	}

	DBG(KERN_DEBUG "%s\n", __func__);

	/* Restart the clocks */
	brcm_pm_memc1_clock_start();
	brcm_pm_memc1_enable_plls();
	brcm_pm_memc1_disable_idle_pad();

	brcm_pm_memc1_sspd_control(0);
	BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET, MEMC_DRAM_INIT, 0);
	BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET, MEMC_CORE, 0);
	mdelay(1);
	printk(KERN_DEBUG "memc1: powered up\n");

	brcm_pm_memc1_ddr_params(1);
	brcm_pm_memc1_arb_params(1);
	brcm_pm_save_restore_rts(BCHP_MEMC_ARB_1_REG_START,
		memc1_config.client_info, 1);
	brcm_pm_memc1_sspd_control(0);

	return 0;
}

#if defined(CONFIG_BCM7425) || defined(CONFIG_BCM7435)
static int __brcm_pm_memc1_initialized(void)
{
	return BDEV_RD_F(MEMC_DDR_1_DRAM_INIT_STATUS, INIT_DONE);
}

static void __brcm_pm_memc1_clock_start(void)
{
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_108_CLOCK_ENABLE, 1);
}

static void __brcm_pm_memc1_clock_stop(void)
{
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_108_CLOCK_ENABLE, 0);
}

static int __brcm_pm_memc1_clock_running(void)
{
	return BDEV_RD_F(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
			DDR1_SCB_CLOCK_ENABLE) &&
		BDEV_RD_F(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
			DDR1_108_CLOCK_ENABLE);
}

static void brcm_pm_memc1_enable_idle_pad(void)
{
	/* enable idle pad powerdown */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		PHY_IDLE_ENABLE, 1);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 1);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_IDLE_PAD_CONTROL,
		0x132);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL) |
			0xFFFFF);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL) |
			0xFFFFF);
	DBG(KERN_DEBUG "%s\n", __func__);
}

static void brcm_pm_memc1_disable_idle_pad(void)
{
	/* disable idle pad powerdown */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 0);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_IDLE_PAD_CONTROL,
		0);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL) |
			0);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL) |
			0);

}

static void brcm_pm_memc1_enable_plls(void)
{
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_SYS_PLL_PWRDN_ref_clk_sel,
		PWRDN, 0);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_PLL_CONFIG, 0);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_POWER_SWITCH_MEMORY,
		DDR1_POWER_SWITCH_MEMORY, 2);
	mdelay(1);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_POWER_SWITCH_MEMORY,
		DDR1_POWER_SWITCH_MEMORY, 0);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_MEMORY_STANDBY_ENABLE,
		DDR1_MEMORY_STANDBY_ENABLE, 0);
#if defined(BCHP_CLKGEN_MEMSYS_1_32_POWER_MANAGEMENT)
	BDEV_WR_F_RB(CLKGEN_MEMSYS_1_32_POWER_MANAGEMENT,
		MEMSYS_PLL_PWRDN_POWER_MANAGEMENT, 0);
#endif

}
static void brcm_pm_memc1_disable_plls(void)
{
	/* power down the PLLs */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_SYS_PLL_PWRDN_ref_clk_sel,
		PWRDN, 1);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_PLL_CONFIG, 3);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_POWER_SWITCH_MEMORY,
		DDR1_POWER_SWITCH_MEMORY, 3);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_MEMORY_STANDBY_ENABLE,
		DDR1_MEMORY_STANDBY_ENABLE, 1);

#if defined(BCHP_CLKGEN_MEMSYS_1_32_POWER_MANAGEMENT)
	BDEV_WR_F_RB(CLKGEN_MEMSYS_1_32_POWER_MANAGEMENT,
		MEMSYS_PLL_PWRDN_POWER_MANAGEMENT, 1);
#endif
}



#endif

#endif
