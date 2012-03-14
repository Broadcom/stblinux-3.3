/*
 * Copyright (c) 2003 Silicon Graphics, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston MA 02111-1307, USA.
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

#include <xfs/libxfs.h>
#include "command.h"

static cmdinfo_t help_cmd;
static void help_onecmd(const char *cmd, const cmdinfo_t *ct);
static void help_oneline(const char *cmd, const cmdinfo_t *ct);

static void
help_all(void)
{
	const cmdinfo_t	*ct;

	for (ct = cmdtab; ct < &cmdtab[ncmds]; ct++)
		help_oneline(ct->name, ct);
	printf(_("\nUse 'help commandname' for extended help.\n"));
}

static int
help_f(
	int		argc,
	char		**argv)
{
	const cmdinfo_t	*ct;

	if (argc == 1) {
		help_all();
		return 0;
	}
	ct = find_command(argv[1]);
	if (ct == NULL) {
		printf(_("command %s not found\n"), argv[1]);
		return 0;
	}
	help_onecmd(argv[1], ct);
	return 0;
}

static void
help_onecmd(
	const char	*cmd,
	const cmdinfo_t	*ct)
{
	help_oneline(cmd, ct);
	if (ct->help)
		ct->help();
}

static void
help_oneline(
	const char	*cmd,
	const cmdinfo_t	*ct)
{
	if (cmd)
		printf("%s ", cmd);
	else {
		printf("%s ", ct->name);
		if (ct->altname)
			printf("(or %s) ", ct->altname);
	}
	if (ct->args)
		printf("%s ", ct->args);
	printf("-- %s\n", ct->oneline);
}

void
help_init(void)
{
	help_cmd.name = _("help");
	help_cmd.altname = _("?");
	help_cmd.cfunc = help_f;
	help_cmd.argmin = 0;
	help_cmd.argmax = 1;
	help_cmd.args = _("[command]");
	help_cmd.oneline = _("help for one or all commands");

	add_command(&help_cmd);
}
