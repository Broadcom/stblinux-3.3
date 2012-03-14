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

typedef struct request_table table_t;

struct request_table
{
  table_t *next, *prev;
  CTL_MSG request;
  time_t time;
};

static table_t *table;

static void
table_delete (table_t *ptr)
{
  table_t *t;

  if ((t = ptr->prev) != NULL)
    t->next = ptr->next;
  else
    table = ptr->next;
  if ((t = ptr->next) != NULL)
    t->prev = ptr->prev;
  free (ptr);
}

/* Look in the table for an invitation that matches given criteria.
   Linear search: premature optimisation is the root of all
   Evil) */

static CTL_MSG *
lookup_request (CTL_MSG *request, int (*comp)(table_t *, CTL_MSG *, time_t *))
{
  table_t *ptr;
  time_t now;

  time (&now);

  if (debug)
    print_request ("lookup_request", request);

  for (ptr = table; ptr; ptr = ptr->next)
    {
      if (debug)
	print_request ("comparing against: ", &ptr->request);

      if ((ptr->time - now) > max_request_ttl)
	{
	  /* the entry is too old */
	  if (debug)
	    print_request ("deleting expired entry", &ptr->request);
	  table_delete (ptr);
	}
      else
	{
	  if (comp (ptr, request, &now) == 0)
	    {
	      if (debug)
		print_request ("found", &ptr->request);
	      return &ptr->request;
	    }
	}
    }
  if (debug)
    syslog (LOG_DEBUG, "not found");
  return NULL;
}

static int
fuzzy_comp (table_t *ptr, CTL_MSG *request, time_t *unused)
{
  (void)unused;
  if (ptr->request.type == LEAVE_INVITE
      && strcmp(request->l_name, ptr->request.r_name) == 0
      && strcmp(request->r_name, ptr->request.l_name) == 0)
    return 0;
  return 1;
}

/* Look in the table for an invitation that matches the current
   request looking for an invitation */
CTL_MSG *
find_match (CTL_MSG *request)
{
  return lookup_request (request, fuzzy_comp);
}

static int
exact_comp (table_t *ptr, CTL_MSG *request, time_t *now)
{
  if (request->type == ptr->request.type
      && request->pid == ptr->request.pid
      && strcmp(request->r_name, ptr->request.r_name) == 0
      && strcmp(request->l_name, ptr->request.l_name) == 0)
    {
      ptr->time = *now;
      return 0;
    }
  return 1;
}

/* Look for an identical request, as opposed to a complementary
   one as find_match does */

CTL_MSG *
find_request (CTL_MSG *request)
{
  return lookup_request (request, exact_comp);
}

#define MAX_ID 16000

/* Generate a unique non-zero sequence number */
int
new_id ()
{
  static int current_id = 0;

  current_id = (current_id + 1) % MAX_ID;
  /* 0 is reserved, helps to pick up bugs */
  if (current_id == 0)
    current_id = 1;
  return current_id;
}

int
insert_table (CTL_MSG *request, CTL_RESPONSE *response)
{
  table_t *ptr;

  ptr = malloc (sizeof *ptr);
  if (!ptr)
    {
      syslog (LOG_CRIT, "out of memory");
      exit (1);
    }

  request->id_num = new_id ();

  if (debug)
    print_request ("insert_table", request);

  response->id_num = htonl (request->id_num);

  time (&ptr->time);
  ptr->request = *request;
  ptr->prev = NULL;
  ptr->next = table;
  table = ptr;
  return 0;
}

/* Delete the invitation with id 'id_num' */
int
delete_invite (unsigned long id_num)
{
  table_t *ptr;

  for (ptr = table; ptr; ptr = ptr->next)
    if (ptr->request.id_num == id_num)
      {
	table_delete (ptr);
	return SUCCESS;
      }
  return NOT_HERE;
}
