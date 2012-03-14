/* libltt header file
 *
 * Copyright 2005-
 *		 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LIBLTT_H
#define _LIBLTT_H

int lttctl_init(void);
int lttctl_destroy(void);
int lttctl_setup_trace(const char *name);
int lttctl_destroy_trace(const char *name);
int lttctl_alloc_trace(const char *name);
int lttctl_start(const char *name);
int lttctl_pause(const char *name);
int lttctl_set_trans(const char *name, const char *trans);
int lttctl_set_channel_enable(const char *name, const char *channel,
		int enable);
int lttctl_set_channel_overwrite(const char *name, const char *channel,
		int overwrite);
int lttctl_set_channel_subbuf_num(const char *name, const char *channel,
		unsigned subbuf_num);
int lttctl_set_channel_subbuf_size(const char *name, const char *channel,
		unsigned subbuf_size);
int lttctl_set_channel_switch_timer(const char *name, const char *channel,
		unsigned switch_timer);

/* Helper functions */
int getdebugfsmntdir(char *mntdir);

#endif /*_LIBLTT_H */
