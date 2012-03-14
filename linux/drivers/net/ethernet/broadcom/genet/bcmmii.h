/*
 *
 * Copyright (c) 2002-2005 Broadcom Corporation
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
#ifndef _BCMMII_H_
#define _BCMMII_H_

/* BRCM EPHY Aux status summery Register */
#define MII_BRCM_AUX_STAT_SUM				0x19
#define MII_BRCM_AUX_FD						0x01
#define MII_BRCM_AUX_AN						0x10
#define MII_BRCM_AUX_LINK_UP				0x20
#define MII_BRCM_AUX_SPEED_100				0x40
#define MII_BRCM_AUX_LP_AN					0x80
#define MII_BRCM_AUX_LP_PR					0x100
/* highest common denominator*/
#define MII_BRCM_AUX_AN_HCD_MASK			0x03
#define MII_BRCM_AUX_AN_HCD_SHIFT			0x08
#define MII_BRCM_AUX_AN_HCD_10T				0X01
#define MII_BRCM_AUX_AN_HCD_10T_FULL		0x02
#define MII_BRCM_AUX_AN_HCD_100TX			0x03
#define MII_BRCM_AUX_AN_HCD_100T4			0x04
#define MII_BRCM_AUX_AN_HCD_100TX_FULL		0x05
#define MII_BRCM_AUX_AN_PAUSE				0x800
#define MII_BRCM_AUX_AN_COMPLETE			0x8000
/* BRCM GPHY Aux status summery register bits ?? */
#define MII_BRCM_AUX_GPHY_TX_PAUSE			0
#define MII_BRCM_AUX_GPHY_RX_PAUSE			1
/****************************************************************************
Prototypes
****************************************************************************/

extern void mii_setup(struct net_device *dev);
extern int mii_init(struct net_device *dev);
extern int mii_probe(struct net_device *dev, void * p);

#endif /* _BCMMII_H_ */
