AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES = -I$(top_builddir)/include

bin_PROGRAMS = @tftp_BUILD@

EXTRA_PROGRAMS = tftp

tftp_SOURCES = main.c tftp.c

noinst_HEADERS = extern.h

man_MANS = tftp.1

LDADD = -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)
