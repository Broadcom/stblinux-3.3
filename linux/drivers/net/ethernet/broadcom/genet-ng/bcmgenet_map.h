/*
 * Copyright (c) 2002-2008 Broadcom Corporation
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
 *
*/
/* uniMAC register definations.*/

#ifndef __BCMGENET_MAP_H__
#define __BCMGENET_MAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/types.h>
#include "bcmgenet_defs.h"

#ifndef __ASSEMBLY__

/* 64B status Block */
struct status_64 {
	u32	length_status;		/* length and peripheral status */
	u32	ext_status;		/* Extended status*/
	u32	rx_csum;		/* partial rx checksum */
#if CONFIG_BRCM_GENET_VERSION < 3
	u32	filter_index;		/* Filter index */
	u32	extracted_bytes[4];	/* Extracted byte 0 - 16 */
	u32	reserved[4];
#else /* GENET_V3+ */
	u32	filter_index[2];	/* Filter index */
	u32	extracted_bytes[4];	/* Extracted byte 0 - 16 */
	u32	reserved[3];
#endif
	u32	tx_csum_info;		/* Tx checksum info. */
	u32	unused[3];		/* unused */
} ;
/* Rx status bits */
#define STATUS_RX_EXT_MASK		0x1FFFFF
#define STATUS_RX_CSUM_MASK		0xFFFF
#define STATUS_RX_CSUM_OK		0x10000
#define STATUS_RX_CSUM_FR		0x20000
#define STATUS_RX_PROTO_TCP		0
#define STATUS_RX_PROTO_UDP		1
#define STATUS_RX_PROTO_ICMP	2
#define STATUS_RX_PROTO_OTHER	3
#define STATUS_RX_PROTO_MASK	3
#define STATUS_RX_PROTO_SHIFT	18
#define STATUS_FILTER_INDEX_MASK	0xFFFF
/* Tx status bits */
#define STATUS_TX_CSUM_START_MASK	0X7FFF
#define STATUS_TX_CSUM_START_SHIFT	16
#define STATUS_TX_CSUM_PROTO_UDP	0x8000
#define STATUS_TX_CSUM_OFFSET_MASK	0x7FFF
#define STATUS_TX_CSUM_LV			0x80000000

/*
** DMA Descriptor
*/
struct DmaDesc {
	u32	length_status;	/* in bytes of data in buffer */
	u32	address_lo;	/* lower 32 bits of PA */
#if CONFIG_BRCM_GENET_VERSION >= 4
	u32	address_hi;	/* upper 32 bits of PA */
#endif
};

#define WORDS_PER_BD			(sizeof(struct DmaDesc) / sizeof(u32))

static inline void dmadesc_set_addr(volatile struct DmaDesc *d, dma_addr_t addr)
{
	d->address_lo = addr & 0xffffffff;
#if CONFIG_BRCM_GENET_VERSION >= 4
	d->address_hi = (u64)addr >> 32;
#endif
}

static inline dma_addr_t dmadesc_get_addr(const volatile struct DmaDesc *d)
{
	dma_addr_t addr;

	addr = d->address_lo;
#if CONFIG_BRCM_GENET_VERSION >= 4
	addr |= (u64)d->address_hi << 32;
#endif
	return addr;
}

/*
** UniMAC TSV or RSV (Transmit Status Vector or Receive Status Vector
*/
/* Rx/Tx common counter group.*/
struct PktCounterSize {
	u32	cnt_64;		/* RO Recvied/Transmited 64 bytes packet */
	u32	cnt_127;	/* RO Rx/Tx 127 bytes packet */
	u32	cnt_255;	/* RO Rx/Tx 65-255 bytes packet */
	u32	cnt_511;	/* RO Rx/Tx 256-511 bytes packet */
	u32	cnt_1023;	/* RO Rx/Tx 512-1023 bytes packet */
	u32	cnt_1518;	/* RO Rx/Tx 1024-1518 bytes packet */
	u32	cnt_mgv;	/* RO Rx/Tx 1519-1522 good VLAN packet */
	u32	cnt_2047;	/* RO Rx/Tx 1522-2047 bytes packet*/
	u32	cnt_4095;	/* RO Rx/Tx 2048-4095 bytes packet*/
	u32	cnt_9216;	/* RO Rx/Tx 4096-9216 bytes packet*/
};
/* RSV, Receive Status Vector */
struct UniMacRSV {
	struct PktCounterSize stat_sz;	/* (0x400 - 0x424), stats of received
			packets classfied by size */
	u32	rx_pkt;		/* RO (0x428) Receive pkt count*/
	u32	rx_bytes;	/* RO Receive byte count */
	u32	rx_mca;		/* RO # of Received multicast pkt */
	u32	rx_bca;		/* RO # of Receive broadcast pkt */
	u32	rx_fcs;		/* RO # of Received FCS error  */
	u32	rx_cf;		/* RO # of Received control frame pkt*/
	u32	rx_pf;		/* RO # of Received pause frame pkt */
	u32	rx_uo;		/* RO # of unknown op code pkt */
	u32	rx_aln;		/* RO # of alignment error count */
	u32	rx_flr;		/* RO # of frame length out of range count */
	u32	rx_cde;		/* RO # of code error pkt */
	u32	rx_fcr;		/* RO # of carrier sense error pkt */
	u32	rx_ovr;		/* RO # of oversize pkt*/
	u32	rx_jbr;		/* RO # of jabber count */
	u32	rx_mtue;	/* RO # of MTU error pkt*/
	u32	rx_pok;		/* RO # of Received good pkt */
	u32	rx_uc;		/* RO # of unicast pkt */
	u32	rx_ppp;		/* RO # of PPP pkt */
	u32	rcrc;		/* RO (0x470),# of CRC match pkt */
};

/* TSV, Transmit Status Vector */
struct UniMacTSV {
	struct PktCounterSize stat_sz;	/* (0x480 - 0x0x4a4), statistics of
			xmited packets classified by size */
	u32	tx_pkt;		/* RO (0x4a8) Transmited pkt */
	u32	tx_mca;		/* RO # of xmited multicast pkt */
	u32	tx_bca;		/* RO # of xmited broadcast pkt */
	u32	tx_pf;		/* RO # of xmited pause frame count */
	u32	tx_cf;		/* RO # of xmited control frame count */
	u32	tx_fcs;		/* RO # of xmited FCS error count */
	u32	tx_ovr;		/* RO # of xmited oversize pkt */
	u32	tx_drf;		/* RO # of xmited deferral pkt */
	u32	tx_edf;		/* RO # of xmited Excessive deferral pkt*/
	u32	tx_scl;		/* RO # of xmited single collision pkt */
	u32	tx_mcl;		/* RO # of xmited multiple collision pkt*/
	u32	tx_lcl;		/* RO # of xmited late collision pkt */
	u32	tx_ecl;		/* RO # of xmited excessive collision pkt*/
	u32	tx_frg;		/* RO # of xmited fragments pkt*/
	u32	tx_ncl;		/* RO # of xmited total collision count */
	u32	tx_jbr;		/* RO # of xmited jabber count*/
	u32	tx_bytes;	/* RO # of xmited byte count */
	u32	tx_pok;		/* RO # of xmited good pkt */
	u32	tx_uc;		/* RO (0x0x4f0)# of xmited unitcast pkt */
};

struct uniMacRegs {
	u32	unused;			/* (00) RO */
	u32	hdBkpCtrl;		/* (04) RW */
	u32	cmd;			/* (08) RW */
	u32	mac_0;			/* (0x0c) RW */
	u32	mac_1;			/* (0x10) RW */
	u32	max_frame_len;		/* (0x14) RW */
	u32	pause_quant;		/* (0x18) RW */
	u32	unused0[9];		/* (0x1c - 0x3c) */
	u32	sdf_offset;		/* (0x40) RW */
	u32	mode;			/* (0x44) RO */
	u32	frm_tag0;		/* (0x48) RW */
	u32	frm_tag1;		/* (0x4c) RW */
	u32	unused1[3];		/* (0x50 - 0x58) */
	u32	tx_ipg_len;		/* (0x5c) RW */
	u32	unused2;		/* (0x60) */
	u32	eee_ctrl;		/* (0x64) RW */
	u32	eee_lpi_timer;		/* (0x68) RW */
	u32	eee_wake_timer;		/* (0x6c) RW */
	u32	eee_ref_count;		/* (0x70) RWS */
	u32	unused3;		/* (0x74) */
	u32	rx_pkt_drop_status;	/* (0x78) RW */
	u32	symmetric_idle_thld;	/* (0x7c) RW */
	u32	unused4[164];		/* (0x80 - 0x30c) */
	u32	macsec_tx_crc;		/* (0x310) RW */
	u32	macsec_ctrl;		/* (0x314) RW */
	u32	ts_status;		/* (0x318) RO */
	u32	ts_data;		/* (0x31c) RO */
	u32	unused5[4];		/* (0x320 - 0x32c) */
	u32	pause_ctrl;		/* (0x330) RW */
	u32	tx_flush;		/* (0x334) RW */
	u32	rxfifo_status;		/* (0x338) RO */
	u32	txfifo_status;		/* (0x33c) RO */
	u32	ppp_ctrl;		/* (0x340) RW */
	u32	ppp_refresh_ctrl;	/* (0x344) RW */
	u32	unused6[4];		/* (0x348 - 0x354) */
	u32	unused7[4];		/* (0x358 - 0x364) */
	u32	unused8[38];		/* (0x368 - 0x3fc) */
	struct UniMacRSV rsv;		/* (0x400 - 0x470) */
	u32	unused9[3];		/* (0x474 - 0x47c) */
	struct UniMacTSV tsv;		/* (0x480 - 0x4f0) */
	u32	unused10[7];		/* (0x4f4 - 0x50c) */
	u32	unused11[28];		/* (0x510 - 0x57c) */
	u32	mib_ctrl;		/* (0x580) RW */
	u32	unused12[31];		/* (0x584 - 0x5fc) */
	u32	bkpu_ctrl;		/* (0x600) RW */
	u32	mac_rxerr_mask;		/* (0x604) RW  */
	u32	max_pkt_size;		/* (0x608) RW */
	u32	vlan_tag;		/* (0x60c) RW */
	u32	unused13;		/* (0x610) */
	u32	mdio_cmd;		/* (0x614  RO */
	u32	mdio_cfg;		/* (0x618) RW */
#if CONFIG_BRCM_GENET_VERSION > 1
	u32	unused14;		/* (0x61c) */
#else
	u32	rbuf_ovfl_pkt_cnt;	/* (0x61c) RO */
#endif
	u32	mpd_ctrl;		/* (0x620) RW */
	u32	mpd_pw_ms;		/* (0x624) RW */
	u32	mpd_pw_ls;		/* (0x628) RW */
	u32	unused15[3];		/* (0x62c - 0x634) */
	u32	mdf_cnt;		/* (0x638) RO */
	u32	unused16[4];		/* (0x63c - 0x648) */
	u32	diag_sel;		/* (0x64c) RW */
	u32	mdf_ctrl;		/* (0x650) RW */
	u32	mdf_addr[34];		/* (0x654 - 0x6d8) */
};

#if CONFIG_BRCM_GENET_VERSION < 3
#define HFB_NUM_FLTRS		16
#else
#define HFB_NUM_FLTRS		48
#endif

#if CONFIG_BRCM_GENET_VERSION > 1
struct tbufRegs {
	u32	tbuf_ctrl;		/* (0x00) */
	u32	unused0;		/* (0x04) */
	u32	tbuf_endian_ctrl;	/* (0x08) */
	u32	tbuf_bp_mc;		/* (0x0c) */
	u32	tbuf_pkt_rdy_thld;	/* (0x10) */
	u32	tbuf_energy_ctrl;	/* (0x14) */
	u32	tbuf_ext_bp_stats;	/* (0x18) */
	u32	tbuf_tsv_mask0;		/* (0x1c) */
	u32	tbuf_tsv_mask1;		/* (0x20) */
	u32	tbuf_tsv_status0;	/* (0x24) */
	u32	tbuf_tsv_status1;	/* (0x28) */
};

struct rbufRegs {
	u32	rbuf_ctrl;		/* (0x00) */
	u32	unused0;		/* (0x04) */
	u32	rbuf_pkt_rdy_thld;	/* (0x08) */
	u32	rbuf_status;		/* (0x0c) */
	u32	rbuf_endian_ctrl;	/* (0x10) */
	u32	rbuf_chk_ctrl;		/* (0x14) */
#if CONFIG_BRCM_GENET_VERSION == 2
	u32	rbuf_rxc_offset[8];	/* (0x18 - 0x34) */
	u32	unused1[18];
	u32	rbuf_ovfl_pkt_cnt;	/* (0x80) */
	u32	rbuf_err_cnt;		/* (0x84) */
	u32	rbuf_energy_ctrl;	/* (0x88) */

	u32	unused2[7];
	u32	rbuf_pd_sram_ctrl;	/* (0xa8) */
	u32	unused3[12];
	u32	rbuf_test_mux_ctrl;	/* (0xdc) */
#else /* GENET_V3+ */
	u32	unused1[7];		/* (0x18 - 0x34) */
	u32	rbuf_rxc_offset[24];	/* (0x34 - 0x90) */
	u32	rbuf_ovfl_pkt_cnt;	/* (0x94) */
	u32	rbuf_err_cnt;		/* (0x98) */
	u32	rbuf_energy_ctrl;	/* (0x9c) */
	u32	rbuf_pd_sram_ctrl;	/* (0xa0) */
	u32	rbuf_test_mux_ctrl;	/* (0xa4) */
	u32	rbuf_spare_reg0;	/* (0xa8) */
	u32	rbuf_spare_reg1;	/* (0xac) */
	u32	rbuf_spare_reg2;	/* (0xb0) */
	u32	rbuf_tbuf_size_ctrl;	/* (0xb4) */
#endif
};

struct hfbRegs {
	u32	hfb_ctrl;
#if CONFIG_BRCM_GENET_VERSION > 2
	u32	hfb_flt_enable[2];
	u32	unused[4];
#endif
	u32	hfb_fltr_len[HFB_NUM_FLTRS / 4];
};

#else /* CONFIG_BRCM_GENET_VERSION > 1 */
struct rbufRegs {
	u32	rbuf_ctrl;		/* (0x00) */
	u32	rbuf_flush_ctrl;	/* (0x04) */
	u32	rbuf_pkt_rdy_thld;	/* (0x08) */
	u32	rbuf_status;		/* (0x0c) */
	u32	rbuf_endian_ctrl;	/* (0x10) */
	u32	rbuf_chk_ctrl;		/* (0x14) */
	u32	rbuf_rxc_offset[8];	/* (0x18 - 0x34) */
	u32	rbuf_hfb_ctrl;		/* (0x38) */
	u32	rbuf_fltr_len[HFB_NUM_FLTRS / 4]; /* (0x3c - 0x48) */
	u32	unused0[13];		/* (0x4c - 0x7c) */
	u32	tbuf_ctrl;		/* (0x80) */
	u32	tbuf_flush_ctrl;	/* (0x84) */
	u32	unused1[5];		/* (0x88 - 0x98) */
	u32	tbuf_endian_ctrl;	/* (0x9c) */
	u32	tbuf_bp_mc;		/* (0xa0) */
	u32	tbuf_pkt_rdy_thld;	/* (0xa4) */
	u32	unused2[2];		/* (0xa8 - 0xac) */
	u32	rgmii_oob_ctrl;		/* (0xb0) */
	u32	rgmii_ib_status;	/* (0xb4) */
	u32	rgmii_led_ctrl;		/* (0xb8) */
	u32	unused3;		/* (0xbc) */
	u32	moca_status;		/* (0xc0) */
	u32	unused4[6];		/* (0xc4 - 0xd8) */
	u32	test_mux_ctrl;		/* (0xdc) */
};
#endif
/* uniMac intrl2 registers */
struct intrl2Regs {
	u32	cpu_stat;		/* (0x00) */
	u32	cpu_set;		/* (0x04) */
	u32	cpu_clear;		/* (0x08) */
	u32	cpu_mask_status;	/* (0x0c) */
	u32	cpu_mask_set;		/* (0x10) */
	u32	cpu_mask_clear;		/* (0x14) */
	u32	pci_stat;		/* (0x00) */
	u32	pci_set;		/* (0x04) */
	u32	pci_clear;		/* (0x08) */
	u32	pci_mask_status;	/* (0x0c) */
	u32	pci_mask_set;		/* (0x10) */
	u32	pci_mask_clear;		/* (0x14) */
};

/* Register block offset */
#define GENET_GR_BRIDGE_OFF			0x0040
#define GENET_EXT_OFF				0x0080
#define GENET_INTRL2_0_OFF			0x0200
#define GENET_INTRL2_1_OFF			0x0240
#define GENET_RBUF_OFF				0X0300
#define GENET_UMAC_OFF				0x0800

#if CONFIG_BRCM_GENET_VERSION == 1
#define GENET_HFB_OFF				0x1000
#define GENET_RDMA_OFF				0x2000
#define GENET_TDMA_OFF				0x3000
#elif CONFIG_BRCM_GENET_VERSION == 2
#define GENET_TBUF_OFF				0x0600
#define GENET_HFB_OFF				0x1000
#define GENET_HFB_REG_OFF			0x2000
#define GENET_RDMA_OFF				0x3000
#define GENET_TDMA_OFF				0x4000
#elif CONFIG_BRCM_GENET_VERSION == 3
#define GENET_TBUF_OFF				0x0600
#define GENET_HFB_OFF				0x8000
#define GENET_HFB_REG_OFF			0xfc00
#define GENET_RDMA_OFF				0x10000
#define GENET_TDMA_OFF				0x11000
#elif CONFIG_BRCM_GENET_VERSION == 4
#define GENET_TBUF_OFF				0x0600
#define GENET_HFB_OFF				0x8000
#define GENET_HFB_REG_OFF			0xfc00
#define GENET_RDMA_OFF				0x2000
#define GENET_TDMA_OFF				0x4000
#endif

struct SysRegs {
	u32	sys_rev_ctrl;
	u32	sys_port_ctrl;
#if CONFIG_BRCM_GENET_VERSION > 1
	u32	rbuf_flush_ctrl;
	u32	tbuf_flush_ctrl;
#endif
};

struct GrBridgeRegs {
	u32	gr_bridge_rev;
	u32	gr_bridge_ctrl;
	u32	gr_bridge_sw_reset_0;
	u32	gr_bridge_sw_reset_1;
};

struct ExtRegs {
	u32	ext_pwr_mgmt;
	u32	ext_emcg_ctrl;
	u32	ext_test_ctrl;
#if CONFIG_BRCM_GENET_VERSION > 1
	u32	rgmii_oob_ctrl;
	u32	rgmii_ib_status;
	u32	rgmii_led_ctrl;
	u32	ext_genet_pwr_mgmt;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	ext_gphy_ctrl;
	u32	ext_gphy_status;
#endif
#else
	u32	ext_in_ctrl;
	u32	ext_fblp_ctrl;
	u32	ext_stat0;
	u32	ext_stat1;
	u32	ext_ch_ctrl[6];
#endif
};

struct rDmaRingRegs {
	u32	rdma_write_pointer;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	rdma_write_pointer_hi;
#endif
	u32	rdma_producer_index;
	u32	rdma_consumer_index;
	u32	rdma_ring_buf_size;
	u32	rdma_start_addr;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	rdma_start_addr_hi;
#endif
	u32	rdma_end_addr;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	rdma_end_addr_hi;
#endif
	u32	rdma_mbuf_done_threshold;
	u32	rdma_xon_xoff_threshold;
	u32	rdma_read_pointer;	/* NOTE: not used by hardware */
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	rdma_read_pointer_hi;
	u32	unused[3];
#else
	u32	unused[7];
#endif
};

struct tDmaRingRegs {
	u32	tdma_read_pointer;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	tdma_read_pointer_hi;
#endif
	u32	tdma_consumer_index;
	u32	tdma_producer_index;
	u32	tdma_ring_buf_size;
	u32	tdma_start_addr;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	tdma_start_addr_hi;
#endif
	u32	tdma_end_addr;
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	tdma_end_addr_hi;
#endif
	u32	tdma_mbuf_done_threshold;
	u32	tdma_flow_period;
	u32	tdma_write_pointer;	/* NOTE: not used by hardware */
#if CONFIG_BRCM_GENET_VERSION > 3
	u32	tdma_write_pointer_hi;
	u32	unused[3];
#else
	u32	unused[7];
#endif
};

struct rDmaRegs {
	struct rDmaRingRegs rDmaRings[17];
#if CONFIG_BRCM_GENET_VERSION > 1
	u32	rdma_ring_cfg;
#endif
	u32	rdma_ctrl;
	u32	rdma_status;
#if CONFIG_BRCM_GENET_VERSION < 2
	u32	unused;
#endif
	u32	rdma_scb_burst_size;
	u32	rdma_activity;
	u32	rdma_mask;
	u32	rdma_map[3];
	u32	rdma_back_status;
	u32	rdma_override;
	u32	rdma_timeout[17];
#if CONFIG_BRCM_GENET_VERSION > 2
	u32	rdma_index2ring[8];
#endif
	u32	rdma_test;
	u32	rdma_debug;
};

struct tDmaRegs {
	struct tDmaRingRegs tDmaRings[17];
#if CONFIG_BRCM_GENET_VERSION > 1
	u32	tdma_ring_cfg;
#endif
	u32	tdma_ctrl;
	u32	tdma_status;
#if CONFIG_BRCM_GENET_VERSION == 1
	u32	unused;
#endif
	u32	tdma_scb_burst_size;
	u32	tdma_activity;
	u32	tdma_mask;
#if CONFIG_BRCM_GENET_VERSION > 2
	u32	tdma_map[2];
#else
	u32	tdma_map[3];
#endif
	u32	tdma_back_status;
	u32	tdma_override;
	u32	tdma_rate_limit_ctrl;
	u32	tdma_arb_ctrl;
	u32	tdma_priority[3];
#if CONFIG_BRCM_GENET_VERSION > 2
	u32	tdma_rate_adj;
	u32	tdma_test;
	u32	tdma_debug;
#else
	u32	tdma_test;
	u32	tdma_debug;
	u32	tdma_rate_adj;
#endif
};


#endif /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif

#endif
