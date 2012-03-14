AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @inetd_BUILD@

EXTRA_PROGRAMS = inetd

inetd_SOURCES = inetd.c

man_MANS = inetd.8

@PATHDEFS_MAKE@

INCLUDES = $(PATHDEF_INETDCONF) $(PATHDEF_INETDDIR) -I$(top_builddir)/include

LDADD = -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)
