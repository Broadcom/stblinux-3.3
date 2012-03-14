/***************************************************************************
 *     Copyright (c) 2004-2009, Broadcom Corporation
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <asm/param.h>
#include <fcntl.h>
typedef unsigned short u16;
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/sockios.h>
#include "ethctl.h"
#include "boardparms.h"
//#include "bcmioctl.h"

#define PORT        "port"
static const char *media_names[] = {
	"10baseT", "10baseT-FD", "100baseTx", "100baseTx-FD", "100baseT4",
	"Flow-control", 0,
};

static int mdio_read(int skfd, struct ifreq *ifr, int phy_id, int location)
{
    unsigned short *data = (unsigned short *)(&ifr->ifr_data);

    data[0] = phy_id;
    data[1] = location;

    if (ioctl(skfd, SIOCGMIIREG, ifr) < 0) {
        fprintf(stderr, "SIOCGMIIREG on %s failed: %s\n", ifr->ifr_name,
            strerror(errno));
        return 0;
    }
    return data[3];
}

static void mdio_write(int skfd, struct ifreq *ifr, int phy_id, int location, int value)
{
    unsigned short *data = (unsigned short *)(&ifr->ifr_data);

    data[0] = phy_id;
    data[1] = location;
    data[2] = value;

    if (ioctl(skfd, SIOCSMIIREG, ifr) < 0) {
        fprintf(stderr, "SIOCSMIIREG on %s failed: %s\n", ifr->ifr_name,
            strerror(errno));
    }
}

static int et_dev_port_query(int skfd, struct ifreq *ifr)
{
    int ports = 0;

    ifr->ifr_data = (char*)&ports;
    //if (ioctl(skfd, SIOCGQUERYNUMPORTS, ifr) < 0) {
    //    fprintf(stderr, "interface %s ioctl SIOCGQUERYNUMPORTS error!\n", ifr->ifr_name);
    //    return -1;
    //}
    return ports;;
}

static int et_get_portid(int skfd, struct ifreq *ifr, const char *port)
{
    int portid;
    int emac_ports;

    if ((emac_ports = et_dev_port_query(skfd, ifr)) < 0)
        return -1;

    if (emac_ports == 1) {
        fprintf(stderr, "interface %s is not Ethernet Switch, don't specify port number\n", ifr->ifr_name);
        return -1;
    }

    portid = atoi(port);
    if (portid >= emac_ports) {
        fprintf(stderr, "Invalid port, interface %s Ethernet Switch port range 0 to %d\n", ifr->ifr_name, emac_ports-1);
        return -1;
    }
    else
        return portid;
}

static int et_get_phyid(int skfd, struct ifreq *ifr)
{
    unsigned short *data = (unsigned short *)(&ifr->ifr_data);

    data[0] = 0;
	if (ioctl(skfd, SIOCGMIIPHY, ifr) < 0)
        return -1;

    return data[0];
}

static int parse_media_options(char *option)
{
    int mode = -1;

    if (strcmp(option, "auto") == 0) {
        mode = MEDIA_TYPE_AUTO;
    } else if (strcmp(option, "100FD") == 0) {
        mode = MEDIA_TYPE_100M_FD;
    } else if (strcmp(option, "100HD") == 0) {
        mode = MEDIA_TYPE_100M_HD;
    } else if (strcmp(option, "10FD") == 0) {
        mode = MEDIA_TYPE_10M_FD;
    } else if (strcmp(option, "10HD") == 0) {
        mode = MEDIA_TYPE_10M_HD;
    }
    return mode;
}

static int show_speed_setting(int skfd, struct ifreq *ifr, int phy_id)
{
    int i;
    int bmcr, bmsr, nway_advert, lkpar;

    bmcr = mdio_read(skfd, ifr, phy_id, MII_BMCR);
    bmsr = mdio_read(skfd, ifr, phy_id, MII_BMSR);

    if (bmcr == 0xffff  ||  bmsr == 0x0000) {
        fprintf(stderr, "  No MII transceiver present!.\n");
        return -1;
    }

    nway_advert = mdio_read(skfd, ifr, phy_id, MII_ADVERTISE);
    lkpar = mdio_read(skfd, ifr, phy_id, MII_LPA);

    if (bmcr & BMCR_ANENABLE) {
        fprintf(stderr, "Auto-negotiation enabled.\n");
        if (lkpar & ADVERTISE_LPACK) {
            int negotiated = nway_advert & lkpar & 
                            (ADVERTISE_100BASE4 |
                            ADVERTISE_100FULL |
                            ADVERTISE_100HALF |
                            ADVERTISE_10FULL |
                            ADVERTISE_10HALF );
            int max_capability = 0;
            /* Scan for the highest negotiated capability, highest priority
                (100baseTx-FDX) to lowest (10baseT-HDX). */
            int media_priority[] = {8, 9, 7, 6, 5}; 	/* media_names[i-5] */
            for (i = 0; media_priority[i]; i++) {
                if (negotiated & (1 << media_priority[i])) {
                    max_capability = media_priority[i];
                    break;
                }
            }
            if (max_capability)
			    fprintf(stderr, "The autonegotiated media type is %s.\n",
				    media_names[max_capability - 5]);
		    else
			    fprintf(stderr, "No common media type was autonegotiated!\n"
				    "This is extremely unusual and typically indicates a "
				    "configuration error.\n" "Perhaps the advertised "
				    "capability set was intentionally limited.\n");
	    }
    } else {
        fprintf(stderr, "Auto-negotiation disabled, with\n"
            " Speed fixed at 10%s mbps, %s-duplex.\n",
            bmcr & BMCR_SPEED100 ? "0" : "",
            bmcr & BMCR_FULLDPLX ? "full":"half");
    }
	bmsr = mdio_read(skfd, ifr, phy_id, MII_BMSR);
	bmsr = mdio_read(skfd, ifr, phy_id, MII_BMSR);
    fprintf(stderr, "Link is %s\n", (bmsr & BMSR_LSTATUS) ? "up" : "down");
    return 0;
}

static int et_cmd_media_type_op(int skfd, struct ifreq *ifr, cmd_t *cmd, char** argv)
{
    int phy_id = 0;
    int set = 0;
    int mode = 0;
	int val =0;
    int emac_ports;

    argv = argv+3;;
    if (*argv) {
        if (strncmp(*argv, "port", strlen("port"))) {
            /* parse media type setting */
            if ((mode = parse_media_options(*argv)) < 0) {
                command_help(cmd);
                return -1;
            }
            argv++;
            set = 1;
        }
    }
    if (*argv) {
        /* parse transceiver port number */
        if (!strncmp(*argv, "port", strlen("port")))
            phy_id = et_get_portid(skfd, ifr, *(argv+1));
        if (phy_id < 0) {
            command_help(cmd);
            return -1;
        }
    } else {
        if ((emac_ports = et_dev_port_query(skfd, ifr)) < 0) {
            command_help(cmd);
            return -1;
        }
        if (emac_ports > 1) {
            fprintf(stderr, "interface %s is Ethernet Switch, please use port [0-%d] argument\n", ifr->ifr_name, emac_ports-1);
            command_help(cmd);
            return -1;
        }
    }
    phy_id |= et_get_phyid(skfd, ifr);
    if (set) {
        switch (mode) {
            case MEDIA_TYPE_AUTO:
                val = BMCR_ANENABLE | BMCR_ANRESTART;
                break;
            case MEDIA_TYPE_100M_FD:
                val = BMCR_SPEED100 | BMCR_FULLDPLX;
                break;
            case MEDIA_TYPE_100M_HD:
                val = BMCR_SPEED100;
                break;
            case MEDIA_TYPE_10M_FD:
                val = BMCR_FULLDPLX;
                break;
            case MEDIA_TYPE_10M_HD:
                val = 0;
            break;
        }
	    mdio_write(skfd, ifr, phy_id, MII_BMCR, val);
        if (mode == MEDIA_TYPE_AUTO)
            sleep(2);

    }
    show_speed_setting(skfd, ifr, phy_id);
    return 0;
}

static int et_cmd_phy_reset_op(int skfd, struct ifreq *ifr, cmd_t *cmd, char** argv)
{
    int phy_id = 0;
    int emac_ports;

    if (argv[3]) {
        if (!strncmp(argv[3], "port", strlen("port")))
            phy_id = et_get_portid(skfd, ifr, argv[4]);
        if (phy_id < 0) {
            command_help(cmd);
            return -1;
        }
    } else {
        if ((emac_ports = et_dev_port_query(skfd, ifr)) < 0) {
            command_help(cmd);
            return -1;
        }
        if (emac_ports > 1) {
            fprintf(stderr, "interface %s is Ethernet Switch, please use port [0-%d] argument\n", ifr->ifr_name, emac_ports-1);
            command_help(cmd);
            return -1;
        }
    }
    phy_id |= et_get_phyid(skfd, ifr);

	mdio_write(skfd, ifr, phy_id, MII_BMCR, BMCR_RESET);
    sleep(2);
    show_speed_setting(skfd, ifr, phy_id);
    return 0;
}

static int et_cmd_mii_op(int skfd, struct ifreq *ifr, cmd_t *cmd, char** argv)
{
    int phy_id = 0;
    int set = 0;
	int val = 0;
    int reg;
    int emac_ports;

    argv = argv+3;;
    if (*argv) {
        /* parse register address */
        reg = atoi(*argv);
        if ((reg < 0) || (reg > 31)) {
            command_help(cmd);
            return -1;
        }
        argv++;
    }
    if (*argv) {
        if (strncmp(*argv, "port", strlen("port"))) {
            /* parse register setting value */
            val = strtol(*argv, NULL, 16); 
            set = 1;
            argv++;
        }
        if (*argv) {
            /* parse transceiver port number */
            if (!strncmp(*argv, "port", strlen("port")))
                phy_id = et_get_portid(skfd, ifr, *(argv+1));
            if (phy_id < 0) {
                command_help(cmd);
                return -1;
            }
        }
    } else {
        if ((emac_ports = et_dev_port_query(skfd, ifr)) < 0) {
            command_help(cmd);
            return -1;
        }
        if (emac_ports > 1) {
            fprintf(stderr, "interface %s is Ethernet Switch, please use port [0-%d] argument\n", ifr->ifr_name, emac_ports-1);
            command_help(cmd);
            return -1;
        }
    }
    phy_id |= et_get_phyid(skfd, ifr);
    if (set)
	    mdio_write(skfd, ifr, phy_id, reg, val);
    val = mdio_read(skfd, ifr, phy_id, reg);
    fprintf(stderr, "mii register %d is 0x%04x\n", reg, val);
    return 0;
}

static int et_cmd_vport_enable(int skfd, struct ifreq *ifr)
{
    int err = 0;

   //err = ioctl(skfd, SIOCGENABLEVLAN, ifr);

    return err;
}

static int et_cmd_vport_disable(int skfd, struct ifreq *ifr)
{
    int err = 0;

    //err = ioctl(skfd, SIOCGDISABLEVLAN, ifr);

    return err;
}

static int et_cmd_vport_query(int skfd, struct ifreq *ifr)
{
    int err = 0;
    int ports = 0;

    ifr->ifr_data = (char*)&ports;
    //err = ioctl(skfd, SIOCGQUERYNUMVLANPORTS, ifr);
    if (err == 0)
        fprintf(stderr, "%u\n", ports);

    return err;
}
static int et_cmd_vport_op(int skfd, struct ifreq *ifr, cmd_t *cmd, char** argv)
{
    int err = -1;
    char *arg;

    arg = argv[3];
    if (strcmp(arg, "enable") == 0) {
        err = et_cmd_vport_enable(skfd, ifr);
    } else if (strcmp(arg, "disable") == 0) {
        err = et_cmd_vport_disable(skfd, ifr);
    } else if (strcmp(arg, "query") == 0) {
        err = et_cmd_vport_query(skfd, ifr);
    } else {
        command_help(cmd);
        return 1;
    }
    if (err)
        fprintf(stderr, "command return error!\n");

    return err;
}

static const struct command commands[] = {
    { 0, "media-type", et_cmd_media_type_op, 
      "[option] [port 0-3]\tget/set media type\n"
      "  [option]: auto - auto select\n"
      "            100FD - 100Mb, Full Duplex\n"
      "            100HD - 100Mb, Half Duplex\n"
      "            10FD  - 10Mb,  Full Duplex\n"
      "            10HD  - 10Mb,  Half Duplex\n"
      "  [port 0-3]: required if <interface> is Ethernet Switch"
      },
    { 0, "phy-reset", et_cmd_phy_reset_op, 
      "[port 0-3]\t\t\tsoft reset transceiver\n"
      "  [port 0-3]: required if <interface> is Ethernet Switch"
      },
    { 1, "reg", et_cmd_mii_op, 
      "<[0-31]> [0xhhhh] [port 0-3]\tget/set port mii register\n"
      "  [port 0-3]: required if <interface> is Ethernet Switch"
      },
    { 1, "vport", et_cmd_vport_op, 
      "<enable|disable|query>\t\tenable/disable/port query Switch for VLAN port mapping" },
};

cmd_t *command_lookup(const char *cmd)
{
    int i;

    for (i = 0; i < sizeof(commands)/sizeof(commands[0]); i++) {
        if (!strcmp(cmd, commands[i].name))
            return (cmd_t *)&commands[i];
    }

    return NULL;
}

void command_help(const cmd_t *cmd)
{
    fprintf(stderr, "  %s %s\n\n", cmd->name, cmd->help);
}

void command_helpall(void)
{
    int i;

    for (i = 0; i < sizeof(commands)/sizeof(commands[0]); i++) 
        command_help(commands+i);
}

