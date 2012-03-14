AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES = -I$(top_builddir)/include

bin_PROGRAMS = @logger_BUILD@

LDADD = -L../libinetutils -linetutils

EXTRA_PROGRAMS = logger

logger_SOURCES = logger.c

man_MANS = logger.1

EXTRA_DIST = $(man_MANS)
