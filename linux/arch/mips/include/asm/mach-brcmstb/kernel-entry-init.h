/***************************************************************************
 *     Copyright (c) 2008, Broadcom Corporation
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
 *
 ***************************************************************************/

#ifndef __ASM_MACH_BRCMSTB_KERNEL_ENTRY_H
#define __ASM_MACH_BRCMSTB_KERNEL_ENTRY_H

	.macro kernel_entry_setup

	# save arguments for CFE callback
	sw	a0, cfe_handle
	sw	a2, cfe_entry
	sw	a3, cfe_seal

	jal	bmips_enable_xks01

	.endm

        .macro  smp_slave_setup
        .endm

#endif /* __ASM_MACH_BRCMSTB_KERNEL_ENTRY_H */
