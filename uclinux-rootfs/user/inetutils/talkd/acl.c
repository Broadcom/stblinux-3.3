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
#include <pwd.h>
#include <regex.h>
#include <ctype.h>
#include "argcv.h"

typedef struct netdef netdef_t;

struct netdef
{
  netdef_t *next;
  unsigned int ipaddr;
  unsigned int netmask;
};

#define ACT_ALLOW  0
#define ACT_DENY   1

typedef struct acl acl_t;

struct acl
{
  acl_t *next;
  regex_t re;
  netdef_t *netlist;
  int action;
};


acl_t *acl_head, *acl_tail;

#define DOTTED_QUAD_LEN 16

static int
read_address (char **line_ptr, char *ptr)
{
  char *startp = *line_ptr;
  char *endp;
  int dotcount = 0;

  for (endp = startp; *endp; endp++, ptr++)
    if (!(isdigit (*endp) || *endp == '.'))
      break;
    else if (endp < startp + DOTTED_QUAD_LEN)
      {
	if (*endp == '.')
	  dotcount++;
	*ptr = *endp;
      }
    else
      break;
  *line_ptr = endp;
  *ptr = 0;
  return dotcount;
}

static netdef_t *
netdef_parse (char *str)
{
  unsigned int ipaddr, netmask;
  netdef_t *netdef;
  char ipbuf[DOTTED_QUAD_LEN+1];

  if (strcmp (str, "any") == 0)
    {
      ipaddr = 0;
      netmask = 0;
    }
  else
    {
      read_address (&str, ipbuf);
      ipaddr = inet_addr (ipbuf);
      if (ipaddr == INADDR_NONE)
	return NULL;
      if (*str == 0)
	netmask = 0xfffffffful;
      else if (*str != '/')
	return NULL;
      else
	{
	  str++;
	  if (read_address (&str, ipbuf) == 0)
	    {
	      /* netmask length */
	      unsigned int len = strtoul (ipbuf, NULL, 0);
	      if (len > 32)
		return NULL;
	      netmask = 0xfffffffful >> (32-len);
	      netmask <<= (32-len);
	      /*FIXME: hostorder?*/
	    }
	  else
	    netmask = inet_network (ipbuf);
	  netmask = htonl (netmask);
	}
    }

  netdef = malloc (sizeof *netdef);
  if (!netdef)
    {
      syslog (LOG_ERR, "out of memory");
      exit (1);
    }

  netdef->next = NULL;
  netdef->ipaddr = ipaddr;
  netdef->netmask = netmask;

  return netdef;
}

void
read_acl (char *config_file)
{
  FILE *fp;
  int line;
  char buf[128];
  char *ptr;

  if (!config_file)
    return;

  fp = fopen (config_file, "r");
  if (!fp)
    {
      syslog (LOG_ERR, "can't open config file %s: %m", config_file);
      return;
    }

  line = 0;
  while ((ptr = fgets (buf, sizeof buf, fp)))
    {
      int len, i;
      int argc;
      char **argv;
      int  action;
      regex_t re;
      netdef_t *head, *tail;
      acl_t *acl;

      line++;
      len = strlen (ptr);
      if (len > 0 && ptr[len-1] == '\n')
	ptr[--len] = 0;

      while (*ptr && isspace (*ptr))
	ptr++;
      if (!*ptr || *ptr == '#')
	continue;

      argcv_get (ptr, "", &argc, &argv);
      if (argc < 2)
	{
	  syslog (LOG_ERR, "%s:%d: too few fields", config_file, line);
	  argcv_free (argc, argv);
	  continue;
	}

      if (strcmp (argv[0], "allow") == 0)
	action = ACT_ALLOW;
      else if (strcmp (argv[0], "deny") == 0)
	action = ACT_DENY;
      else
	{
	  syslog (LOG_ERR, "%s:%d: unknown keyword", config_file, line);
	  argcv_free (argc, argv);
	  continue;
	}

      if (regcomp (&re, argv[1], 0) != 0)
	{
	  syslog (LOG_ERR, "%s:%d: bad regexp", config_file, line);
	  argcv_free (argc, argv);
	  continue;
	}

      head = tail = NULL;
      for (i = 2; i < argc; i++)
	{
	  netdef_t *cur = netdef_parse (argv[i]);
	  if (!cur)
	    {
	      syslog (LOG_ERR, "%s:%d: can't parse netdef: %s",
		      config_file, line, argv[i]);
	      continue;
	    }
	  if (!tail)
	    head = cur;
	  else
	    tail->next = cur;
	  tail = cur;
	}

      argcv_free (argc, argv);

      acl = malloc (sizeof *acl);
      if (!acl)
	{
	  syslog (LOG_CRIT, "out of memory");
	  exit (1);
	}
      acl->next = NULL;
      acl->action = action;
      acl->netlist = head;
      acl->re = re;

      if (!acl_tail)
	acl_head = acl;
      else
	acl_tail->next = acl;
      acl_tail = acl;
    }

  fclose (fp);
}

static acl_t *
open_users_acl (char *name)
{
  char *filename;
  struct passwd *pw;
  acl_t *mark;

  pw = getpwnam (name);
  if (!pw)
    return NULL;

  filename = malloc (strlen (pw->pw_dir) + sizeof (USER_ACL_NAME) + 2 /* Null and separator.  */);
  if (!filename)
    {
      syslog (LOG_ERR, "out of memory");
      return NULL;
    }

  sprintf (filename, "%s/%s", pw->pw_dir, USER_ACL_NAME);

  mark = acl_tail;
  read_acl (filename);
  free (filename);
  return mark;
}

static void
netdef_free (netdef_t *netdef)
{
  netdef_t *next;

  while (netdef)
    {
      next = netdef->next;
      free (netdef);
      netdef = next;
    }
}

static void
acl_free (acl_t *acl)
{
  acl_t *next;

  while (acl)
    {
      next = acl->next;
      regfree (&acl->re);
      netdef_free (acl->netlist);
      free (acl);
      acl = next;
    }
}

static void
discard_acl (acl_t *mark)
{
  if (mark)
    {
      acl_free (mark->next);
      acl_tail = mark;
      acl_tail->next = NULL;
    }
  else
    acl_head = acl_tail = NULL;
}

int
acl_match (CTL_MSG *msg, struct sockaddr_in *sa_in)
{
  acl_t *acl, *mark;
  unsigned int ip;

  mark = open_users_acl (msg->r_name);
  ip = sa_in->sin_addr.s_addr;
  for (acl = acl_head; acl; acl = acl->next)
    {
      netdef_t *net;

      for (net = acl->netlist; net; net = net->next)
	{
	  if (net->ipaddr == (ip & net->netmask))
	    {
	      if (regexec (&acl->re, msg->l_name, 0, NULL, 0) == 0)
		{
		  discard_acl (mark);
		  return acl->action;
		}
	    }
	}
    }
  discard_acl (mark);
  return ACT_ALLOW;
}
