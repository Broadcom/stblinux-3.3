AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES=-I$(top_builddir)/include

noinst_LIBRARIES = @libls_BUILD@

EXTRA_LIBRARIES = libls.a

libls_a_SOURCES = cmp.c stat_flags.c ls.c print.c util.c bsdport.c fts.c

noinst_HEADERS = extern.h ls.h bsdport.h fts.h

EXTRA_DIST = ls.1
