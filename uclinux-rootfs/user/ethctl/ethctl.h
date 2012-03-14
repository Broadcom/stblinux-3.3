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

#ifndef _ETHCTL_H
#define _ETHCTL_H

#include <linux/if.h>

/* media type command argument option */
#define MEDIA_TYPE_AUTO         0
#define MEDIA_TYPE_100M_FD      1
#define MEDIA_TYPE_100M_HD      2
#define MEDIA_TYPE_10M_FD       3
#define MEDIA_TYPE_10M_HD       4

typedef struct command cmd_t;
typedef int (cmd_func_t)(int skfd, struct ifreq *ifr, cmd_t *cmd, char** argv);

struct command
{
    int         nargs;
    const char  *name;
    cmd_func_t  *func;
    const char  *help;
};

cmd_t *command_lookup(const char *cmd);
void command_help(const cmd_t *);
void command_helpall(void);

#endif
