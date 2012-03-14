/* Copyright (C) 1998,2001 Free Software Foundation, Inc.

   This file is part of GNU Inetutils.

   GNU Inetutils is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   GNU Inetutils is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GNU Inetutils; see the file COPYING.  If not, write to
   the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA. */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <sys/param.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#ifdef HAVE_OSOCKADDR_H
# include <osockaddr.h>
#endif
#include <protocols/talkd.h>
#include <netdb.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>

#ifndef INADDR_NONE
# define INADDR_NONE -1
#endif

#define USER_ACL_NAME ".talkrc"

extern int debug;
extern unsigned int timeout;
extern time_t max_idle_time;
extern time_t max_request_ttl;
extern char *hostname;

#define os2sin_addr(cp) (((struct sockaddr_in *)&(cp))->sin_addr)

extern CTL_MSG *find_request (CTL_MSG *request);
extern CTL_MSG *find_match (CTL_MSG *request);
extern int process_request(CTL_MSG *mp, struct sockaddr_in *sa_in, CTL_RESPONSE *rp);

extern int print_request (const char *cp, CTL_MSG *mp);
extern int print_response (const char *cp, CTL_RESPONSE *rp);

extern int insert_table (CTL_MSG *request, CTL_RESPONSE *response);
extern int delete_invite (unsigned long id_num);
extern int new_id (void);
extern void read_acl (char *config_file);
extern int acl_match (CTL_MSG *msg, struct sockaddr_in *sa_in);
extern int announce (CTL_MSG *request, char *remote_machine);
