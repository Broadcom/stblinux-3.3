/*
 * Copyright (C) 20014 Broadcom Corporation
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

#ifndef I2C_BRCMSTB_H
#define I2C_BRCMSTB_H

#define BRCMSTB_I2C_MAX_BUS_ID          5
#define BRCMSTB_I2C_BUS_DISABLE         0x0
#define BRCMSTB_I2C_BUS_ENABLE          0x1
#define BRCMSTB_I2C_BUS_ENABLE_CMDLINE  0x02

struct brcmstb_i2c_platform_data {
	u32 reg_start;
	u32 reg_end;
	u32 clk_rate;
	char *bus_name;
	int  bus_id;
	u32 flags;
	int irq;
};

#endif /* I2C_BRCMSTB_H */
