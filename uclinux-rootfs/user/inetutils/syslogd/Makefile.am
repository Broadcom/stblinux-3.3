AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @syslogd_BUILD@

EXTRA_PROGRAMS = syslogd

syslogd_SOURCES = syslogd.c

man_MANS = syslog.conf.5 syslogd.8

@PATHDEFS_MAKE@

INCLUDES = $(PATHDEF_LOG) $(PATHDEF_LOGCONF) $(PATHDEF_LOGPID) \
	$(PATHDEF_CONSOLE) -I$(top_builddir)/include

LDADD = -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)
