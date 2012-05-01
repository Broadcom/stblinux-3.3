/***************************************************************************
 *     Copyright (c) 1999-2012, Broadcom Corporation
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
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Wed Mar  7 03:11:44 2012
 *                 MD5 Checksum         d41d8cd98f00b204e9800998ecf8427e
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008005
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: $
 *
 ***************************************************************************/

#ifndef BCHP_GIO_AON_H__
#define BCHP_GIO_AON_H__

/***************************************************************************
 *GIO_AON - GPIO
 ***************************************************************************/
#define BCHP_GIO_AON_ODEN_LO                     0x004094c0 /* GENERAL PURPOSE I/O OPEN DRAIN ENABLE [17:0] */
#define BCHP_GIO_AON_DATA_LO                     0x004094c4 /* GENERAL PURPOSE I/O DATA [17:0] */
#define BCHP_GIO_AON_IODIR_LO                    0x004094c8 /* GENERAL PURPOSE I/O DIRECTION [17:0] */
#define BCHP_GIO_AON_EC_LO                       0x004094cc /* GENERAL PURPOSE I/O EDGE CONFIGURATION [17:0] */
#define BCHP_GIO_AON_EI_LO                       0x004094d0 /* GENERAL PURPOSE I/O EDGE INSENSITIVE [17:0] */
#define BCHP_GIO_AON_MASK_LO                     0x004094d4 /* GENERAL PURPOSE I/O INTERRUPT MASK [17:0] */
#define BCHP_GIO_AON_LEVEL_LO                    0x004094d8 /* GENERAL PURPOSE I/O INTERRUPT TYPE [17:0] */
#define BCHP_GIO_AON_STAT_LO                     0x004094dc /* GENERAL PURPOSE I/O INTERRUPT STATUS [17:0] */
#define BCHP_GIO_AON_ODEN_EXT                    0x004094e0 /* GENERAL PURPOSE I/O OPEN DRAIN ENABLE [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_DATA_EXT                    0x004094e4 /* GENERAL PURPOSE I/O DATA [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_IODIR_EXT                   0x004094e8 /* GENERAL PURPOSE I/O DIRECTION [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_EC_EXT                      0x004094ec /* GENERAL PURPOSE I/O EDGE CONFIGURATION [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_EI_EXT                      0x004094f0 /* GENERAL PURPOSE I/O EDGE INSENSITIVE [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_MASK_EXT                    0x004094f4 /* GENERAL PURPOSE I/O INTERRUPT MASK [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_LEVEL_EXT                   0x004094f8 /* GENERAL PURPOSE I/O INTERRUPT TYPE [65:64] (SGPIO[3:0]) */
#define BCHP_GIO_AON_STAT_EXT                    0x004094fc /* GENERAL PURPOSE I/O INTERRUPT STATUS [65:64] (SGPIO[3:0]) */

/***************************************************************************
 *ODEN_LO - GENERAL PURPOSE I/O OPEN DRAIN ENABLE [17:0]
 ***************************************************************************/
/* GIO_AON :: ODEN_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_ODEN_LO_reserved0_MASK                        0xfffc0000
#define BCHP_GIO_AON_ODEN_LO_reserved0_SHIFT                       18

/* GIO_AON :: ODEN_LO :: oden [17:00] */
#define BCHP_GIO_AON_ODEN_LO_oden_MASK                             0x0003ffff
#define BCHP_GIO_AON_ODEN_LO_oden_SHIFT                            0
#define BCHP_GIO_AON_ODEN_LO_oden_DEFAULT                          0x00000000

/***************************************************************************
 *DATA_LO - GENERAL PURPOSE I/O DATA [17:0]
 ***************************************************************************/
/* GIO_AON :: DATA_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_DATA_LO_reserved0_MASK                        0xfffc0000
#define BCHP_GIO_AON_DATA_LO_reserved0_SHIFT                       18

/* GIO_AON :: DATA_LO :: data [17:00] */
#define BCHP_GIO_AON_DATA_LO_data_MASK                             0x0003ffff
#define BCHP_GIO_AON_DATA_LO_data_SHIFT                            0
#define BCHP_GIO_AON_DATA_LO_data_DEFAULT                          0x00000000

/***************************************************************************
 *IODIR_LO - GENERAL PURPOSE I/O DIRECTION [17:0]
 ***************************************************************************/
/* GIO_AON :: IODIR_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_IODIR_LO_reserved0_MASK                       0xfffc0000
#define BCHP_GIO_AON_IODIR_LO_reserved0_SHIFT                      18

/* GIO_AON :: IODIR_LO :: iodir [17:00] */
#define BCHP_GIO_AON_IODIR_LO_iodir_MASK                           0x0003ffff
#define BCHP_GIO_AON_IODIR_LO_iodir_SHIFT                          0
#define BCHP_GIO_AON_IODIR_LO_iodir_DEFAULT                        0x0003ffff

/***************************************************************************
 *EC_LO - GENERAL PURPOSE I/O EDGE CONFIGURATION [17:0]
 ***************************************************************************/
/* GIO_AON :: EC_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_EC_LO_reserved0_MASK                          0xfffc0000
#define BCHP_GIO_AON_EC_LO_reserved0_SHIFT                         18

/* GIO_AON :: EC_LO :: edge_config [17:00] */
#define BCHP_GIO_AON_EC_LO_edge_config_MASK                        0x0003ffff
#define BCHP_GIO_AON_EC_LO_edge_config_SHIFT                       0
#define BCHP_GIO_AON_EC_LO_edge_config_DEFAULT                     0x00000000

/***************************************************************************
 *EI_LO - GENERAL PURPOSE I/O EDGE INSENSITIVE [17:0]
 ***************************************************************************/
/* GIO_AON :: EI_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_EI_LO_reserved0_MASK                          0xfffc0000
#define BCHP_GIO_AON_EI_LO_reserved0_SHIFT                         18

/* GIO_AON :: EI_LO :: edge_insensitive [17:00] */
#define BCHP_GIO_AON_EI_LO_edge_insensitive_MASK                   0x0003ffff
#define BCHP_GIO_AON_EI_LO_edge_insensitive_SHIFT                  0
#define BCHP_GIO_AON_EI_LO_edge_insensitive_DEFAULT                0x00000000

/***************************************************************************
 *MASK_LO - GENERAL PURPOSE I/O INTERRUPT MASK [17:0]
 ***************************************************************************/
/* GIO_AON :: MASK_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_MASK_LO_reserved0_MASK                        0xfffc0000
#define BCHP_GIO_AON_MASK_LO_reserved0_SHIFT                       18

/* GIO_AON :: MASK_LO :: irq_mask [17:00] */
#define BCHP_GIO_AON_MASK_LO_irq_mask_MASK                         0x0003ffff
#define BCHP_GIO_AON_MASK_LO_irq_mask_SHIFT                        0
#define BCHP_GIO_AON_MASK_LO_irq_mask_DEFAULT                      0x00000000

/***************************************************************************
 *LEVEL_LO - GENERAL PURPOSE I/O INTERRUPT TYPE [17:0]
 ***************************************************************************/
/* GIO_AON :: LEVEL_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_LEVEL_LO_reserved0_MASK                       0xfffc0000
#define BCHP_GIO_AON_LEVEL_LO_reserved0_SHIFT                      18

/* GIO_AON :: LEVEL_LO :: level [17:00] */
#define BCHP_GIO_AON_LEVEL_LO_level_MASK                           0x0003ffff
#define BCHP_GIO_AON_LEVEL_LO_level_SHIFT                          0
#define BCHP_GIO_AON_LEVEL_LO_level_DEFAULT                        0x00000000

/***************************************************************************
 *STAT_LO - GENERAL PURPOSE I/O INTERRUPT STATUS [17:0]
 ***************************************************************************/
/* GIO_AON :: STAT_LO :: reserved0 [31:18] */
#define BCHP_GIO_AON_STAT_LO_reserved0_MASK                        0xfffc0000
#define BCHP_GIO_AON_STAT_LO_reserved0_SHIFT                       18

/* GIO_AON :: STAT_LO :: irq_status [17:00] */
#define BCHP_GIO_AON_STAT_LO_irq_status_MASK                       0x0003ffff
#define BCHP_GIO_AON_STAT_LO_irq_status_SHIFT                      0
#define BCHP_GIO_AON_STAT_LO_irq_status_DEFAULT                    0x00000000

/***************************************************************************
 *ODEN_EXT - GENERAL PURPOSE I/O OPEN DRAIN ENABLE [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: ODEN_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_ODEN_EXT_reserved0_MASK                       0xfffffff0
#define BCHP_GIO_AON_ODEN_EXT_reserved0_SHIFT                      4

/* GIO_AON :: ODEN_EXT :: oden [03:00] */
#define BCHP_GIO_AON_ODEN_EXT_oden_MASK                            0x0000000f
#define BCHP_GIO_AON_ODEN_EXT_oden_SHIFT                           0
#define BCHP_GIO_AON_ODEN_EXT_oden_DEFAULT                         0x00000000

/***************************************************************************
 *DATA_EXT - GENERAL PURPOSE I/O DATA [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: DATA_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_DATA_EXT_reserved0_MASK                       0xfffffff0
#define BCHP_GIO_AON_DATA_EXT_reserved0_SHIFT                      4

/* GIO_AON :: DATA_EXT :: data [03:00] */
#define BCHP_GIO_AON_DATA_EXT_data_MASK                            0x0000000f
#define BCHP_GIO_AON_DATA_EXT_data_SHIFT                           0
#define BCHP_GIO_AON_DATA_EXT_data_DEFAULT                         0x00000000

/***************************************************************************
 *IODIR_EXT - GENERAL PURPOSE I/O DIRECTION [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: IODIR_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_IODIR_EXT_reserved0_MASK                      0xfffffff0
#define BCHP_GIO_AON_IODIR_EXT_reserved0_SHIFT                     4

/* GIO_AON :: IODIR_EXT :: iodir [03:00] */
#define BCHP_GIO_AON_IODIR_EXT_iodir_MASK                          0x0000000f
#define BCHP_GIO_AON_IODIR_EXT_iodir_SHIFT                         0
#define BCHP_GIO_AON_IODIR_EXT_iodir_DEFAULT                       0x00000003

/***************************************************************************
 *EC_EXT - GENERAL PURPOSE I/O EDGE CONFIGURATION [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: EC_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_EC_EXT_reserved0_MASK                         0xfffffff0
#define BCHP_GIO_AON_EC_EXT_reserved0_SHIFT                        4

/* GIO_AON :: EC_EXT :: edge_config [03:00] */
#define BCHP_GIO_AON_EC_EXT_edge_config_MASK                       0x0000000f
#define BCHP_GIO_AON_EC_EXT_edge_config_SHIFT                      0
#define BCHP_GIO_AON_EC_EXT_edge_config_DEFAULT                    0x00000000

/***************************************************************************
 *EI_EXT - GENERAL PURPOSE I/O EDGE INSENSITIVE [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: EI_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_EI_EXT_reserved0_MASK                         0xfffffff0
#define BCHP_GIO_AON_EI_EXT_reserved0_SHIFT                        4

/* GIO_AON :: EI_EXT :: edge_insensitive [03:00] */
#define BCHP_GIO_AON_EI_EXT_edge_insensitive_MASK                  0x0000000f
#define BCHP_GIO_AON_EI_EXT_edge_insensitive_SHIFT                 0
#define BCHP_GIO_AON_EI_EXT_edge_insensitive_DEFAULT               0x00000000

/***************************************************************************
 *MASK_EXT - GENERAL PURPOSE I/O INTERRUPT MASK [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: MASK_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_MASK_EXT_reserved0_MASK                       0xfffffff0
#define BCHP_GIO_AON_MASK_EXT_reserved0_SHIFT                      4

/* GIO_AON :: MASK_EXT :: irq_mask [03:00] */
#define BCHP_GIO_AON_MASK_EXT_irq_mask_MASK                        0x0000000f
#define BCHP_GIO_AON_MASK_EXT_irq_mask_SHIFT                       0
#define BCHP_GIO_AON_MASK_EXT_irq_mask_DEFAULT                     0x00000000

/***************************************************************************
 *LEVEL_EXT - GENERAL PURPOSE I/O INTERRUPT TYPE [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: LEVEL_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_LEVEL_EXT_reserved0_MASK                      0xfffffff0
#define BCHP_GIO_AON_LEVEL_EXT_reserved0_SHIFT                     4

/* GIO_AON :: LEVEL_EXT :: level [03:00] */
#define BCHP_GIO_AON_LEVEL_EXT_level_MASK                          0x0000000f
#define BCHP_GIO_AON_LEVEL_EXT_level_SHIFT                         0
#define BCHP_GIO_AON_LEVEL_EXT_level_DEFAULT                       0x00000000

/***************************************************************************
 *STAT_EXT - GENERAL PURPOSE I/O INTERRUPT STATUS [65:64] (SGPIO[3:0])
 ***************************************************************************/
/* GIO_AON :: STAT_EXT :: reserved0 [31:04] */
#define BCHP_GIO_AON_STAT_EXT_reserved0_MASK                       0xfffffff0
#define BCHP_GIO_AON_STAT_EXT_reserved0_SHIFT                      4

/* GIO_AON :: STAT_EXT :: irq_status [03:00] */
#define BCHP_GIO_AON_STAT_EXT_irq_status_MASK                      0x0000000f
#define BCHP_GIO_AON_STAT_EXT_irq_status_SHIFT                     0
#define BCHP_GIO_AON_STAT_EXT_irq_status_DEFAULT                   0x00000000

#endif /* #ifndef BCHP_GIO_AON_H__ */

/* End of File */
