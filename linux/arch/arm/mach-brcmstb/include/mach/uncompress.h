/*
 *  arch/arm/mach-brcmstb/include/mach/uncompress.h
 *  Derived from: arch/arm/mach-ebsa110/include/mach/uncompress.h
 *
 *  Copyright (C) 1996,1997,1998 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/serial_reg.h>
#include <linux/types.h>
#include <linux/brcmstb/brcmstb.h>

#define SERIAL_BASE	((void __iomem *)BPHYSADDR(BCHP_UARTA_REG_START))

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	u32 v;
	u32 *base = SERIAL_BASE;

	do {
		v = base[UART_LSR];
		barrier();
	} while (!(v & UART_LSR_THRE));

	base[UART_TX] = c;
}

static inline void flush(void)
{
	unsigned char v, *base = SERIAL_BASE;

	do {
		v = base[UART_LSR << 2];
		barrier();
	} while ((v & (UART_LSR_TEMT|UART_LSR_THRE)) !=
		 (UART_LSR_TEMT|UART_LSR_THRE));
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
