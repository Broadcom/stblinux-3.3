/*
 * Copyright (c) 1995, 2001-2003 Silicon Graphics, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2.1 of the GNU Lesser General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.  Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston MA 02111-1307,
 * USA.
 *
 * Contact information: Silicon Graphics, Inc., 1600 Amphitheatre Pkwy,
 * Mountain View, CA  94043, or:
 *
 * http://www.sgi.com
 *
 * For further information regarding this notice, see:
 *
 * http://oss.sgi.com/projects/GenInfo/SGIGPLNoticeExplan/
 */
#ifndef __HANDLE_H__
#define __HANDLE_H__

#ifdef __cplusplus
extern "C" {
#endif

struct fsdmidata;
struct attrlist_cursor;

extern int  path_to_handle (char *__path, void **__hanp, size_t *__hlen);
extern int  path_to_fshandle (char *__path, void **__hanp, size_t *__hlen);
extern int  handle_to_fshandle (void *__hanp, size_t __hlen, void **__fshanp,
				size_t *__fshlen);
extern void free_handle (void *__hanp, size_t __hlen);
extern int  open_by_handle (void *__hanp, size_t __hlen, int __rw);
extern int  readlink_by_handle (void *__hanp, size_t __hlen, void *__buf,
				size_t __bs);
extern int  attr_multi_by_handle (void *__hanp, size_t __hlen, void *__buf,
				  int __rtrvcnt, int __flags);
extern int  attr_list_by_handle (void *__hanp, size_t __hlen, void *__buf,
				 size_t __bufsize, int __flags,
				 struct attrlist_cursor *__cursor);
extern int  fssetdm_by_handle (void *__hanp, size_t __hlen,
			       struct fsdmidata *__fsdmi);

#ifdef __cplusplus
}
#endif

#endif	/* __HANDLE_H__ */
