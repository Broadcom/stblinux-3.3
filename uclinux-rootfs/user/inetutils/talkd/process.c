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

#include <intalkd.h>

int find_user (char *name, char *tty);
void do_announce (CTL_MSG *mp, CTL_RESPONSE *rp);

int
process_request (CTL_MSG *msg, struct sockaddr_in *sa_in, CTL_RESPONSE *rp)
{
  CTL_MSG *ptr;

  if (debug)
    {
      print_request ("process_request", msg);
    }

  if (acl_match(msg, sa_in))
    {
      syslog (LOG_NOTICE, "dropping request: %s@%s",
	      msg->l_name, inet_ntoa (sa_in->sin_addr));
      return 1;
    }

  rp->vers = TALK_VERSION;
  rp->type = msg->type;
  rp->id_num = htonl (0);
  if (msg->vers != TALK_VERSION)
    {
      syslog (LOG_ERR, "Bad protocol version %d", msg->vers);
      rp->answer = BADVERSION;
      return 0;
    }

  msg->id_num = ntohl (msg->id_num);
  msg->addr.sa_family = ntohs (msg->addr.sa_family);
  if (msg->addr.sa_family != AF_INET)
    {
      syslog(LOG_ERR, "Bad address, family %d",
	     msg->addr.sa_family);
      rp->answer = BADADDR;
      return 0;
    }
  msg->ctl_addr.sa_family = ntohs (msg->ctl_addr.sa_family);
  if (msg->ctl_addr.sa_family != AF_INET)
    {
      syslog(LOG_WARNING, "Bad control address, family %d",
	     msg->ctl_addr.sa_family);
      rp->answer = BADCTLADDR;
      return 0;
    }
  /* FIXME: compare address and sa_in?*/

  msg->pid = ntohl (msg->pid);

  switch (msg->type)
    {
    case ANNOUNCE:
      do_announce (msg, rp);
      break;

    case LEAVE_INVITE:
      ptr = find_request (msg);
      if (ptr)
	{
	  rp->id_num = htonl (ptr->id_num);
	  rp->answer = SUCCESS;
	}
      else
	insert_table (msg, rp);
      break;

    case LOOK_UP:
      ptr = find_match (msg);
      if (ptr)
	{
	  rp->id_num = htonl (ptr->id_num);
	  rp->addr = ptr->addr;
	  rp->addr.sa_family = htons (ptr->addr.sa_family);
	  rp->answer = SUCCESS;
	}
      else
	rp->answer = NOT_HERE;
      break;

    case DELETE:
      rp->answer = delete_invite (msg->id_num);
      break;

    default:
      rp->answer = UNKNOWN_REQUEST;
      break;
    }

  if (debug)
    print_response("process_request response", rp);

  return 0;
}

void
do_announce (CTL_MSG *mp, CTL_RESPONSE *rp)
{
  struct hostent *hp;
  CTL_MSG *ptr;
  int result;

  result = find_user (mp->r_name, mp->r_tty);
  if (result != SUCCESS)
    {
      rp->answer = result;
      return;
    }

  hp = gethostbyaddr((char*)&os2sin_addr(mp->ctl_addr),
		     sizeof (struct in_addr), AF_INET);
  if (!hp)
    {
      rp->answer = MACHINE_UNKNOWN;
      return;
    }
  ptr = find_request (mp);
  if (!ptr)
    {
      insert_table (mp, rp);
      rp->answer = announce (mp, hp->h_name);
      return;
    }
  if (mp->id_num > ptr->id_num)
    {
      /* Explicit re-announce: update the id_num to avoid duplicates
	 and re-announce the talk. */
      ptr->id_num = new_id ();
      rp->id_num = htonl (ptr->id_num);
      rp->answer = announce (mp, hp->h_name);
    }
  else
    {
      /* a duplicated request, so ignore it */
      rp->id_num = htonl (ptr->id_num);
      rp->answer = SUCCESS;
    }
}

#ifdef HAVE_UTMP_H
# include <utmp.h>
#endif

#ifdef UTMPX
# ifdef HAVE_UTMPX_H
#  include <utmpx.h>
# endif
typedef struct utmpx UTMP;
# define SETUTENT() setutxent()
# define GETUTENT() getutxent()
# define ENDUTENT() endutxent()
#else
typedef struct utmp UTMP;
# define SETUTENT() setutent()
# define GETUTENT() getutent()
# define ENDUTENT() endutent()
#endif

/* Search utmp for the local user */
int
find_user (char *name, char *tty)
{
  UTMP *uptr;
  int status;
  struct stat statb;
  char ftty[sizeof (PATH_DEV) + sizeof (uptr->ut_line)];
  time_t last_time = 0;
  int notty;

  notty = (*tty == '\0');

  status = NOT_HERE;
  strcpy(ftty, PATH_DEV);

  SETUTENT ();

  while ((uptr = GETUTENT ()) != NULL)
    {
#ifdef USER_PROCESS
      if (uptr->ut_type != USER_PROCESS)
	continue;
#endif
      if (!strncmp (uptr->ut_name, name, sizeof(uptr->ut_name)))
	{
	  if (notty)
	    {
	      /* no particular tty was requested */
	      strncpy(ftty + sizeof(PATH_DEV) - 1,
		      uptr->ut_line,
		      sizeof(ftty) - sizeof(PATH_DEV) - 1);
	      ftty[sizeof(ftty) - 1] = 0;

	      if (stat(ftty, &statb) == 0)
		{
		  if (!(statb.st_mode & S_IWGRP))
		    {
		      if (status != SUCCESS)
			status = PERMISSION_DENIED;
		      continue;
		    }
		  if (statb.st_atime > last_time)
		    {
		      last_time = statb.st_atime;
		      strcpy(tty, uptr->ut_line);
		      status = SUCCESS;
		    }
		  continue;
		}
	    }
	  if (!strcmp(uptr->ut_line, tty))
	    {
	      status = SUCCESS;
	      break;
	    }
	}
    }

  ENDUTENT ();
  return status;
}

