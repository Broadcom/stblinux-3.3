/*---------------------------------------------------------------------------

    Copyright (c) 2001-2007 Broadcom Corporation                 /\
                                                          _     /  \     _
    _____________________________________________________/ \   /    \   / \_
                                                            \_/      \_/  

 Copyright (c) 2007 Broadcom Corporation
 All rights reserved.
 
 Redistribution and use of this software in source and binary forms, with or
 without modification, are permitted provided that the following conditions
 are met:
 
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 
 * Neither the name of Broadcom Corporation nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission of Broadcom Corporation.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 File: pmlib.h

 Description:
 Power management library definitions

    when        who         what
    -----       ---         ----
    20071030    cernekee    initial version
    20080303    cernekee    add TP1 shutdown
 ------------------------------------------------------------------------- */

#ifndef _H_PMLIB_
#define _H_PMLIB_

/*
 * For all read/write fields:
 *
 * BRCM_PM_UNDEF from brcm_pm_get_status() means Not Available
 * BRCM_PM_UNDEF to brcm_pm_set_status() means Don't Touch
 */

#define BRCM_PM_UNDEF		-1

struct brcm_pm_state
{
	int usb_status;		/* 1=on, 0=off */
	int sata_status;	/* 1=on, 0=off */
	int tp1_status;		/* 1=on, 0=off */
	int tp2_status;		/* 1=on, 0=off */
	int tp3_status;		/* 1=on, 0=off */
	int memc1_status;	/* 1=on, 0=off, 2=testmode */

	int cpu_base;		/* current base frequency, in Hz */
	int cpu_divisor;	/* 1, 2, 4, or 8 */
	int cpu_pll;		/* 0=normal, >0 = power save modes */

	int ddr_timeout;	/* 0=no PM, >0 = timeout for self-refresh */

	int standby_flags;
};

struct brcm_pm_cfg
{
	int use_dhcp;		/* stop/start dhcpcd */
};

void *brcm_pm_init(void);
void brcm_pm_close(void *);

int brcm_pm_get_cfg(void *ctx, struct brcm_pm_cfg *);
int brcm_pm_set_cfg(void *ctx, struct brcm_pm_cfg *);

int brcm_pm_get_status(void *ctx, struct brcm_pm_state *);
int brcm_pm_set_status(void *ctx, struct brcm_pm_state *);

#define BRCM_PM_STANDBY		1
#define BRCM_PM_SUSPEND		3
#define BRCM_PM_HIBERNATE	4
#define BRCM_PM_IRW_HALT	5

int brcm_pm_suspend(void *ctx, int suspend_mode);

#endif /* _H_PMLIB_ */
