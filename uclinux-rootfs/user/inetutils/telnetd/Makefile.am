AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @telnetd_BUILD@

EXTRA_PROGRAMS = telnetd

telnetd_SOURCES = telnetd.c pty.c term.c utility.c \
 state.c termstat.c slc.c
noinst_HEADERS = telnetd.h

man_MANS = telnetd.8

@PATHDEFS_MAKE@

INCLUDES = $(PATHDEF_DEV) $(PATHDEF_TTY) $(PATHDEF_LOGIN) -I$(top_builddir)/include \
 -I$(top_srcdir) @INCAUTH@

LIBTERMCAP = @LIBTERMCAP@
LIBCRYPT = @LIBCRYPT@
LIBUTIL = @LIBUTIL@

LDADD = -L../libinetutils -L../libtelnet -linetutils -ltelnet \
	$(LIBTERMCAP) $(LIBUTIL) $(LIBCRYPT) @LIBAUTH@

EXTRA_DIST = $(man_MANS)
