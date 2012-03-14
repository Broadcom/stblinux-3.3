AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @talkd_BUILD@

EXTRA_PROGRAMS = talkd

talkd_SOURCES = \
 acl.c\
 announce.c\
 print.c\
 process.c\
 table.c\
 talkd.c\
 intalkd.h

man_MANS = talkd.8

@PATHDEFS_MAKE@

INCLUDES = $(PATHDEF_DEV) -I${top_srcdir}/libinetutils -I$(top_builddir)/include

LDADD = -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)
