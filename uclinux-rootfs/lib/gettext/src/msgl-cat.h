/* Message list concatenation and duplicate handling.
   Copyright (C) 2001 Free Software Foundation, Inc.
   Written by Bruno Haible <haible@clisp.cons.org>, 2001.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#ifndef _MSGL_CAT_H
#define _MSGL_CAT_H

#include "message.h"
#include "str-list.h"

#include <stdbool.h>

/* These variables control which messages are selected.  */
extern int more_than;
extern int less_than;

/* If true, use the first available translation.
   If false, merge all available translations into one and fuzzy it.  */
extern bool use_first;

/* If true, merge like msgcomm.
   If false, merge like msgcat and msguniq.  */
extern bool msgcomm_mode;

/* If true, omit the header entry.
   If false, keep the header entry present in the input.  */
extern bool omit_header;

extern msgdomain_list_ty *
       catenate_msgdomain_list PARAMS ((string_list_ty *file_list,
					const char *to_code));

#endif /* _MSGL_CAT_H */
