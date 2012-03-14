/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003 Ralf Baechle
 * Copyright (C) 2009 Broadcom Corporation
 */
#ifndef __ASM_MACH_BRCMSTB_CPU_FEATURE_OVERRIDES_H
#define __ASM_MACH_BRCMSTB_CPU_FEATURE_OVERRIDES_H

#if defined(CONFIG_CPU_BMIPS4380)

#define cpu_has_dc_aliases		1
#define cpu_has_ic_fills_f_dc		0
#define cpu_has_vtag_icache		0
#define cpu_has_inclusive_pcaches	0
#define cpu_icache_snoops_remote_store	1
#define cpu_has_mips32r1		1
#define cpu_has_mips32r2		0
#define cpu_has_mips64r1		0
#define cpu_has_mips64r2		0
#define cpu_dcache_line_size()		64
#define cpu_icache_line_size()		64
#define cpu_scache_line_size()		0

#elif defined(CONFIG_CPU_BMIPS3300)

#define cpu_has_dc_aliases		1
#define cpu_has_ic_fills_f_dc		0
#define cpu_has_vtag_icache		0
#define cpu_has_inclusive_pcaches	0
#define cpu_icache_snoops_remote_store	1
#define cpu_has_mips32r1		1
#define cpu_has_mips32r2		0
#define cpu_has_mips64r1		0
#define cpu_has_mips64r2		0
#define cpu_dcache_line_size()		16
#define cpu_icache_line_size()		16
#define cpu_scache_line_size()		0

#elif defined(CONFIG_CPU_BMIPS5000)

#define cpu_has_dc_aliases		0
#define cpu_has_ic_fills_f_dc		1
#define cpu_has_vtag_icache		0
#define cpu_has_inclusive_pcaches	1
#define cpu_icache_snoops_remote_store	1
#define cpu_has_mips32r1		1
#define cpu_has_mips32r2		0
#define cpu_has_mips64r1		0
#define cpu_has_mips64r2		0
#define cpu_dcache_line_size()		32
#define cpu_icache_line_size()		64
#define cpu_scache_line_size()		128

#endif

#if defined(CONFIG_BRCM_HAS_XI)
#define kernel_uses_smartmips_rixi	1
#endif

#endif /* __ASM_MACH_BRCMSTB_CPU_FEATURE_OVERRIDES_H */
