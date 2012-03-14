AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @rlogind_BUILD@

EXTRA_PROGRAMS = rlogind

rlogind_SOURCES = rlogind.c

man_MANS = rlogind.8

@PATHDEFS_MAKE@

INCLUDES = $(PATHDEF_DEV) $(PATHDEF_LOGIN) -I$(top_builddir)/include

LIBUTIL = @LIBUTIL@

LDADD = -L../libinetutils -linetutils $(LIBUTIL)

EXTRA_DIST = $(man_MANS)
