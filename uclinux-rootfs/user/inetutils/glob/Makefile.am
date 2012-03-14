AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES=-I$(top_builddir)/include

noinst_LIBRARIES = @libglob_BUILD@

EXTRA_LIBRARIES = libglob.a

EXTRA_DIST = configure.bat SMakefile

libglob_a_SOURCES = glob.c glob.h fnmatch.c fnmatch.h

