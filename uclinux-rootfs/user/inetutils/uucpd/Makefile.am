AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @uucpd_BUILD@

EXTRA_PROGRAMS = uucpd

uucpd_SOURCES = uucpd.c

LIBUTIL = @LIBUTIL@
LIBCRYPT = @LIBCRYPT@

@PATHDEFS_MAKE@

INCLUDES = $(PATHDEF_UUCICO) $(PATHDEF_TTY) -I$(top_builddir)/include

LDADD = -L../libinetutils -linetutils $(LIBUTIL) $(LIBCRYPT)
