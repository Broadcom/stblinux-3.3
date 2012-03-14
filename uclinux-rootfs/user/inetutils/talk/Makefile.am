AUTOMAKE_OPTIONS = ../ansi2knr

bin_PROGRAMS = @talk_BUILD@

EXTRA_PROGRAMS = talk

talk_SOURCES = ctl.c ctl_transact.c display.c get_addrs.c get_names.c init_disp.c invite.c io.c look_up.c msgs.c talk.c

noinst_HEADERS = talk.h talk_ctl.h

man_MANS = talk.1

INCLUDES = $(NCURSES_INCLUDE) -I$(top_builddir)/include

LDADD = $(LIBCURSES) -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)
