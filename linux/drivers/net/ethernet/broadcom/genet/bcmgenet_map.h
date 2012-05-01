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

#include "bcmgenet_defs.h"

#ifndef __ASSEMBLY__

/* 64B status Block */
struct status_64 {
	unsigned long length_status;	/* length and peripheral status */
	unsigned long ext_status;		/* Extended status*/
	unsigned long rx_csum;			/* partial rx checksum */
#if CONFIG_BRCM_GENET_VERSION < 3
	unsigned long filter_index;		/* Filter index */
	unsigned long extracted_bytes[4];	/* Extracted byte 0 - 16 */
	unsigned long reserved[4];
#else /* GENET_V3+ */
	unsigned long filter_index[2];		/* Filter index */
	unsigned long extracted_bytes[4];	/* Extracted byte 0 - 16 */
	unsigned long reserved[3];
#endif
	unsigned long tx_csum_info;		/* Tx checksum info. */
	unsigned long unused[3];		/* unused */
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
	unsigned long length_status;	/* in bytes of data in buffer */
	unsigned long address;	/* address of data */
};
/*
** UniMAC TSV or RSV (Transmit Status Vector or Receive Status Vector
*/
/* Rx/Tx common counter group.*/
struct PktCounterSize {
	unsigned long cnt_64;	 /* RO Recvied/Transmited 64 bytes packet */
	unsigned long cnt_127;	 /* RO Rx/Tx 127 bytes packet */
	unsigned long cnt_255;	 /* RO Rx/Tx 65-255 bytes packet */
	unsigned long cnt_511;	 /* RO Rx/Tx 256-511 bytes packet */
	unsigned long cnt_1023;	 /* RO Rx/Tx 512-1023 bytes packet */
	unsigned long cnt_1518;	 /* RO Rx/Tx 1024-1518 bytes packet */
	unsigned long cnt_mgv;	 /* RO Rx/Tx 1519-1522 good VLAN packet */
	unsigned long cnt_2047;	 /* RO Rx/Tx 1522-2047 bytes packet*/
	unsigned long cnt_4095;	 /* RO Rx/Tx 2048-4095 bytes packet*/
	unsigned long cnt_9216;	 /* RO Rx/Tx 4096-9216 bytes packet*/
};
/* RSV, Receive Status Vector */
struct UniMacRSV {
	struct PktCounterSize stat_sz;	/* (0x400 - 0x424), stats of received
			packets classfied by size */
	unsigned long rx_pkt;	 /* RO (0x428) Receive pkt count*/
	unsigned long rx_bytes;	 /* RO Receive byte count */
	unsigned long rx_mca;	 /* RO # of Received multicast pkt */
	unsigned long rx_bca;	 /* RO # of Receive broadcast pkt */
	unsigned long rx_fcs;	 /* RO # of Received FCS error  */
	unsigned long rx_cf;	 /* RO # of Received control frame pkt*/
	unsigned long rx_pf;	 /* RO # of Received pause frame pkt */
	unsigned long rx_uo;	 /* RO # of unknown op code pkt */
	unsigned long rx_aln;	 /* RO # of alignment error count */
	unsigned long rx_flr;	 /* RO # of frame length out of range count */
	unsigned long rx_cde;	 /* RO # of code error pkt */
	unsigned long rx_fcr;	 /* RO # of carrier sense error pkt */
	unsigned long rx_ovr;	 /* RO # of oversize pkt*/
	unsigned long rx_jbr;	 /* RO # of jabber count */
	unsigned long rx_mtue;	 /* RO # of MTU error pkt*/
	unsigned long rx_pok;	 /* RO # of Received good pkt */
	unsigned long rx_uc;	 /* RO # of unicast pkt */
	unsigned long rx_ppp;	 /* RO # of PPP pkt */
	unsigned long rcrc;		 /* RO (0x470),# of CRC match pkt */
};

/* TSV, Transmit Status Vector */
struct UniMacTSV {
	struct PktCounterSize stat_sz;	/* (0x480 - 0x0x4a4), statistics of
			xmited packets classified by size */
	unsigned long tx_pkt;	/* RO (0x4a8) Transmited pkt */
	unsigned long tx_mca;	/* RO # of xmited multicast pkt */
	unsigned long tx_bca;	/* RO # of xmited broadcast pkt */
	unsigned long tx_pf;	/* RO # of xmited pause frame count */
	unsigned long tx_cf;	/* RO # of xmited control frame count */
	unsigned long tx_fcs;	/* RO # of xmited FCS error count */
	unsigned long tx_ovr;	/* RO # of xmited oversize pkt */
	unsigned long tx_drf;	/* RO # of xmited deferral pkt */
	unsigned long tx_edf;	/* RO # of xmited Excessive deferral pkt*/
	unsigned long tx_scl;	/* RO # of xmited single collision pkt */
	unsigned long tx_mcl;	/* RO # of xmited multiple collision pkt*/
	unsigned long tx_lcl;	/* RO # of xmited late collision pkt */
	unsigned long tx_ecl;	/* RO # of xmited excessive collision pkt*/
	unsigned long tx_frg;	/* RO # of xmited fragments pkt*/
	unsigned long tx_ncl;	/* RO # of xmited total collision count */
	unsigned long tx_jbr;	/* RO # of xmited jabber count*/
	unsigned long tx_bytes;	/* RO # of xmited byte count */
	unsigned long tx_pok;	/* RO # of xmited good pkt */
	unsigned long tx_uc;	/* RO (0x0x4f0)# of xmited unitcast pkt */
};

struct uniMacRegs {
	unsigned long unused;	/* (00) UMAC register start from offset 0x04 */
	unsigned long hdBkpCtrl;	/* (04) RW */
	unsigned long cmd;		/* (08) RW */
	unsigned long mac_0;	/* (0x0c) RW */
	unsigned long mac_1;	/* (0x10) RW */
	unsigned long max_frame_len;	/* (0x14) RW */
	unsigned long pause_quant;	/* (0x18) RW */
	unsigned long unused0[9];
	unsigned long sdf_offset;	/* (0x40) RW */
	unsigned long mode;			/* (0x44) RO */
	unsigned long frm_tag0;		/* (0x48) RW */
	unsigned long frm_tag1;		/* (0x4c) RW */
	unsigned long unused10[3];
	unsigned long tx_ipg_len;	/* (0x5c) RW */
	unsigned long unused1[172];
	unsigned long macsec_tx_crc;	/* (0x310) RW */
	unsigned long macsec_ctrl;	/* (0x314) RW */
	unsigned long ts_status;	/* (0x318) RO */
	unsigned long ts_data;		/* (0x31c) RO */
	unsigned long unused2[4];
	unsigned long pause_ctrl;	/* (0x330) RW */
	unsigned long tx_flush;		/* (0x334) RW */
	unsigned long rxfifo_status;	/* (0x338) RO */
	unsigned long txfifo_status;	/* (0x33c) RO */
	unsigned long ppp_ctrl;			/* (0x340) RW */
	unsigned long ppp_refresh_ctrl;	/* (0x344) RW */
	unsigned long unused11[4];		/* (0x348 - 0x354)*/
	unsigned long unused12[4];		/* (0x358 - 0x364) */
	unsigned long unused13[38];
	struct UniMacRSV rsv;			/* (0x400 - 0x470) */
	unsigned long unused3[3];
	struct UniMacTSV tsv;			/* (0x480 - 0x4f0) */
	unsigned long unused4[7];		/* Ignore RUNT sutff for now! */
	unsigned long unused5[28];
	unsigned long mib_ctrl;			/* (0x580) RW */
	unsigned long unused6[31];
	unsigned long bkpu_ctrl;		/* (0x600) RW */
	unsigned long mac_rxerr_mask;	/* (0x604) RW  */
	unsigned long max_pkt_size;		/* (0x608) RW */
	unsigned long unused7[2];
	unsigned long mdio_cmd;			/* (0x614  RO */
	unsigned long mdio_cfg;			/* (0x618) RW */
#if CONFIG_BRCM_GENET_VERSION > 1
	unsigned long unused9;
#else
	unsigned long rbuf_ovfl_pkt_cnt;	/* (0x61c) RO */
#endif
	unsigned long mpd_ctrl;		/* (0x620) RW */
	unsigned long mpd_pw_ms;	/* (0x624) RW */
	unsigned long mpd_pw_ls;	/* (0x628) RW */
	unsigned long unused8[9];
	unsigned long mdf_ctrl;		/* (0x650) RW */
	unsigned long mdf_addr[34];	/* (0x654 - 0x6d8) */

};

#if CONFIG_BRCM_GENET_VERSION < 3
#define HFB_NUM_FLTRS		16
#else
#define HFB_NUM_FLTRS		48
#endif

#if CONFIG_BRCM_GENET_VERSION > 1
struct tbufRegs {
	unsigned long tbuf_ctrl;	/* (00) tx buffer control */
	unsigned long unused0;
	unsigned long tbuf_endian_ctrl;		/* (08) */
	unsigned long tbuf_bp_mc;			/* (0c) */
	unsigned long tbuf_pkt_rdy_thld;	/* (10) */
	unsigned long tbuf_energy_ctrl;		/* (14) */
	unsigned long tbuf_ext_bp_stats;	/* (18) */
	unsigned long tbuf_tsv_mask0;
	unsigned long tbuf_tsv_mask1;
	unsigned long tbuf_tsv_status0;
	unsigned long tbuf_tsv_status1;
};

struct rbufRegs {
	unsigned long rbuf_ctrl;			/* (00)*/
	unsigned long unused0;
	unsigned long rbuf_pkt_rdy_thld;	/* (08)*/
	unsigned long rbuf_status;			/* (0c)*/
	unsigned long rbuf_endian_ctrl;		/* (10)*/
	unsigned long rbuf_chk_ctrl;		/* (14)*/
#if CONFIG_BRCM_GENET_VERSION == 2
	unsigned long rbuf_rxc_offset[8];	/* (18 - 34) */
	unsigned long unused1[18];
	unsigned long rbuf_ovfl_pkt_cnt;	/* (80) */
	unsigned long rbuf_err_cnt;		/* (84) */
	unsigned long rbuf_energy_ctrl;		/* (88) */

	unsigned long unused2[7];
	unsigned long rbuf_pd_sram_ctrl;	/* (a8) */
	unsigned long unused3[12];
	unsigned long rbuf_test_mux_ctrl;	/* (dc) */
#else /* GENET_V3+ */
	unsigned long unused1[7];		/* (18 - 34) */
	unsigned long rbuf_rxc_offset[24];	/* (34 - 90) */
	unsigned long rbuf_ovfl_pkt_cnt;	/* (94) */
	unsigned long rbuf_err_cnt;		/* (98) */
	unsigned long rbuf_energy_ctrl;		/* (9c) */
	unsigned long rbuf_pd_sram_ctrl;	/* (a0) */
	unsigned long rbuf_test_mux_ctrl;	/* (a4) */
	unsigned long rbuf_spare_reg0;		/* (a8) */
	unsigned long rbuf_spare_reg1;		/* (ac) */
	unsigned long rbuf_spare_reg2;		/* (b0) */
	unsigned long rbuf_tbuf_size_ctrl;	/* (b4) */
#endif
};

struct hfbRegs {
	unsigned long hfb_ctrl;
#if CONFIG_BRCM_GENET_VERSION > 2
	unsigned long hfb_flt_enable[2];
	unsigned long unused[4];
#endif
	unsigned long hfb_fltr_len[HFB_NUM_FLTRS / 4];
};

#else /* CONFIG_BRCM_GENET_VERSION > 1 */
struct rbufRegs {
	unsigned long rbuf_ctrl;			/* (00) */
	unsigned long rbuf_flush_ctrl;		/* (04) */
	unsigned long rbuf_pkt_rdy_thld;	/* (08) */
	unsigned long rbuf_status;			/* (0c) */
	unsigned long rbuf_endian_ctrl;		/* (10) */
	unsigned long rbuf_chk_ctrl;		/* (14) */
	unsigned long rbuf_rxc_offset[8];	/* (18 - 34) */
	unsigned long rbuf_hfb_ctrl;		/* (38) */
	unsigned long rbuf_fltr_len[HFB_NUM_FLTRS / 4];	/* (3c - 48) */
	unsigned long unused0[13];
	unsigned long tbuf_ctrl;			/* (80) */
	unsigned long tbuf_flush_ctrl;		/* (84) */
	unsigned long unused1[5];
	unsigned long tbuf_endian_ctrl;		/* (9c) */
	unsigned long tbuf_bp_mc;			/* (a0) */
	unsigned long tbuf_pkt_rdy_thld;	/* (a4) */
	unsigned long unused2[2];
	unsigned long rgmii_oob_ctrl;		/* (b0) */
	unsigned long rgmii_ib_status;		/* (b4) */
	unsigned long rgmii_led_ctrl;		/* (b8) */
	unsigned long unused3;
	unsigned long moca_status;			/* (c0) */
	unsigned long unused4[6];
	unsigned long test_mux_ctrl;		/* (dc) */
};
#endif
/* uniMac intrl2 registers */
struct intrl2Regs {
	unsigned long cpu_stat;	/*(00) CPU interrupt status */
	unsigned long cpu_set;	/*(04) set the corresponding irq*/
	unsigned long cpu_clear;	/*(08) clear the corresponding irq*/
	unsigned long cpu_mask_status;	/*(0c) Show current masking of irq*/
	unsigned long cpu_mask_set;	/*(10) Disable corresponding irq*/
	unsigned long cpu_mask_clear;	/*(14) Enable corresponding irq*/
	unsigned long pci_stat;		/*(00) PCI interrupt status */
	unsigned long pci_set;		/*(04) set the corresponding irq*/
	unsigned long pci_clear;	/*(08) clear the corresponding irq*/
	unsigned long pci_mask_status;	/*(0c) Show current masking of irq*/
	unsigned long pci_mask_set;	/*(10) Disable corresponding irq*/
	unsigned long pci_mask_clear;	/*(14) Enable corresponding irq*/
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
#else /* GENET_V3+ */
#define GENET_TBUF_OFF				0x0600
#define GENET_HFB_OFF				0x8000
#define GENET_HFB_REG_OFF			0xfc00
#define GENET_RDMA_OFF				0x10000
#define GENET_TDMA_OFF				0x11000
#endif

struct SysRegs {
	unsigned long sys_rev_ctrl;
	unsigned long sys_port_ctrl;
#if CONFIG_BRCM_GENET_VERSION > 1
	unsigned long rbuf_flush_ctrl;
	unsigned long tbuf_flush_ctrl;
#endif
};

struct GrBridgeRegs {
	unsigned long gr_bridge_rev;
	unsigned long gr_bridge_ctrl;
	unsigned long gr_bridge_sw_reset_0;
	unsigned long gr_bridge_sw_reset_1;
};

struct ExtRegs {
	unsigned long ext_pwr_mgmt;
	unsigned long ext_emcg_ctrl;
	unsigned long ext_test_ctrl;
#if CONFIG_BRCM_GENET_VERSION > 1
	unsigned long rgmii_oob_ctrl;
	unsigned long rgmii_ib_status;
	unsigned long rgmii_led_ctrl;
	unsigned long ext_genet_pwr_mgmt;
#else
	unsigned long ext_in_ctrl;
	unsigned long ext_fblp_ctrl;
	unsigned long ext_stat0;
	unsigned long ext_stat1;
	unsigned long ext_ch_ctrl[6];
#endif
};

struct rDmaRingRegs {
	unsigned long rdma_write_pointer;
	unsigned long rdma_producer_index;
	unsigned long rdma_consumer_index;
	unsigned long rdma_ring_buf_size;
	unsigned long rdma_start_addr;
	unsigned long rdma_end_addr;
	unsigned long rdma_mbuf_done_threshold;
	unsigned long rdma_xon_xoff_threshold;
	unsigned long rdma_read_pointer;
	unsigned long unused[7];
};

struct tDmaRingRegs {
	unsigned long tdma_read_pointer;
	unsigned long tdma_consumer_index;
	unsigned long tdma_producer_index;
	unsigned long tdma_ring_buf_size;
	unsigned long tdma_start_addr;
	unsigned long tdma_end_addr;
	unsigned long tdma_mbuf_done_threshold;
	unsigned long tdma_flow_period;
	unsigned long tdma_write_pointer;
	unsigned long unused[7];
};

struct rDmaRegs {
	struct rDmaRingRegs rDmaRings[17];
#if CONFIG_BRCM_GENET_VERSION > 1
	unsigned long rdma_ring_cfg;
#endif
	unsigned long rdma_ctrl;
	unsigned long rdma_status;
#if CONFIG_BRCM_GENET_VERSION < 2
	unsigned long unused;
#endif
	unsigned long rdma_scb_burst_size;
	unsigned long rdma_activity;
	unsigned long rdma_mask;
	unsigned long rdma_map[3];
	unsigned long rdma_back_status;
	unsigned long rdma_override;
	unsigned long rdma_timeout[17];
#if CONFIG_BRCM_GENET_VERSION > 2
	unsigned long rdma_index2ring[8];
#endif
	unsigned long rdma_test;
	unsigned long rdma_debug;
};

struct tDmaRegs {
	struct tDmaRingRegs tDmaRings[17];
#if CONFIG_BRCM_GENET_VERSION > 1
	unsigned long tdma_ring_cfg;
#endif
	unsigned long tdma_ctrl;
	unsigned long tdma_status;
#if CONFIG_BRCM_GENET_VERSION == 1
	unsigned long unused;
#endif
	unsigned long tdma_scb_burst_size;
	unsigned long tdma_activity;
	unsigned long tdma_mask;
#if CONFIG_BRCM_GENET_VERSION > 2
	unsigned long tdma_map[2];
#else
	unsigned long tdma_map[3];
#endif
	unsigned long tdma_back_status;
	unsigned long tdma_override;
	unsigned long tdma_rate_limit_ctrl;
	unsigned long tdma_arb_ctrl;
	unsigned long tdma_priority[3];
	unsigned long tdma_test;
	unsigned long tdma_debug;
	unsigned long tdma_rate_adj;
};


#endif /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif

#endif
