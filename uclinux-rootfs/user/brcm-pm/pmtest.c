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

 File: pmtest.c

 Description:
 Power management sample application

    when        who         what
    -----       ---         ----
    20071030    cernekee    initial version
 ------------------------------------------------------------------------- */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>


#include <pmlib.h>

void usage(void)
{
	printf("usage: pmtest <cmd>\n");
	printf("\n");
	printf("examples:\n");
	printf("  status       show current power status\n");
	printf("  usb 0        power down USB controllers\n");
	printf("  sata 1       power up SATA controller\n");
	printf("  tp1 0        power down TP1 (second CPU thread)\n");
	printf("  memc1 0      power down MEMC1 (if available)\n");
	printf("  cpu 4        set CPU clock to BASE/4\n");
	printf("  pll 1        set alternate CPU PLL mode #1\n");
	printf("  ddr 64       enable DDR self-refresh after 64 idle cycles\n");
	printf("  ddr 0        disable DDR self-refresh\n");
	printf("  standby 0x1  enter passive standby (flags = 0x1)\n");
	printf("  irw_halt     enter irw_halt mode\n");
	exit(1);
}

void fatal(char *str)
{
	printf("error: %s\n", str);
	exit(1);
}

int main(int argc, char **argv)
{
	struct brcm_pm_state state;
	struct brcm_pm_cfg cfg;
	void *brcm_pm_ctx;
	int val = 0, has_val = 0, ret;
	char *cmd, *arg;

	while((ret = getopt(argc, argv, "h")) != -1) {
		switch(ret) {
			case 'h':
			default:
				usage();
				break;
		}
	}

	if(optind >= argc)
		usage();
	cmd = argv[optind];
	optind++;

	brcm_pm_ctx = brcm_pm_init();
	if(! brcm_pm_ctx)
		fatal("can't open PM context");
	if(brcm_pm_get_status(brcm_pm_ctx, &state) != 0)
		fatal("can't get PM state");
	
	if(brcm_pm_get_cfg(brcm_pm_ctx, &cfg) != 0)
		fatal("can't get PM config");
	if(brcm_pm_set_cfg(brcm_pm_ctx, &cfg) != 0)
		fatal("can't set PM config");

	if(! strcmp(cmd, "status"))
	{
		printf("usb:          %d\n", state.usb_status);
		printf("sata:         %d\n", state.sata_status);
		printf("tp1:          %d\n", state.tp1_status);
		printf("memc1:        %d\n", state.memc1_status);
		printf("cpu_base:     %d\n", state.cpu_base);
		printf("cpu_divisor:  %d\n", state.cpu_divisor);
		printf("cpu_pll:      %d\n", state.cpu_pll);
		if (state.cpu_divisor != BRCM_PM_UNDEF)
			printf("cpu_speed:    %d\n",
				state.cpu_base / state.cpu_divisor);
		printf("ddr:          %d\n", state.ddr_timeout);
		return(0);
	}

	if(optind < argc)
	{
		char *endptr;
		arg = argv[optind];
		optind++;
		val = strtol(arg, &endptr, 0);
		has_val = 1;
	}

	/* second parameter is optional */

	if(! strcmp(cmd, "standby"))
	{
		if(has_val)
		{
			state.standby_flags = val;
			if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
				fatal("can't set PM state");
		}
		if(brcm_pm_suspend(brcm_pm_ctx, BRCM_PM_STANDBY) != 0)
			fatal("can't suspend");
		return(0);
	}

	if(! strcmp(cmd, "irw_halt"))
	{
		if(brcm_pm_suspend(brcm_pm_ctx, BRCM_PM_IRW_HALT) != 0)
			fatal("can't suspend");
		return(0);
	}

	/* second parameter is mandatory */

	if(!has_val)
		usage();

	if(! strcmp(cmd, "usb"))
	{
		state.usb_status = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (USB)");
		return(0);
	}

	if(! strcmp(cmd, "sata"))
	{
		state.sata_status = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (SATA)");
		return(0);
	}

	if(! strcmp(cmd, "tp1"))
	{
		state.tp1_status = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (TP1)");
		return(0);
	}

	if(! strcmp(cmd, "memc1"))
	{
		state.memc1_status = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (MEMC1)");
		return(0);
	}

	if(! strcmp(cmd, "cpu"))
	{
		state.cpu_divisor = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (CPU)");
		return(0);
	}

	if(! strcmp(cmd, "ddr"))
	{
		state.ddr_timeout = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (DDR)");
		return(0);
	}

	if(! strcmp(cmd, "pll"))
	{
		state.cpu_pll = val;
		if(brcm_pm_set_status(brcm_pm_ctx, &state) != 0)
			fatal("can't set PM state (CPU PLL)");
		return(0);
	}

	usage();
	return(1);	/* never reached */
}
