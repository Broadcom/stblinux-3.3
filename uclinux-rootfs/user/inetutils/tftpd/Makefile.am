AUTOMAKE_OPTIONS = ../ansi2knr

inetdaemondir = @inetdaemondir@

inetdaemon_PROGRAMS = @tftpd_BUILD@

INCLUDES = -I$(top_builddir)/include

EXTRA_PROGRAMS = tftpd

tftpd_SOURCES = tftpd.c

man_MANS = tftpd.8

LDADD = -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)
