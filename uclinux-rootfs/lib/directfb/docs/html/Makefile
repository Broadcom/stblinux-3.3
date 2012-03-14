# Makefile.in generated automatically by automake 1.4-p6 from Makefile.am

# Copyright (C) 1994, 1995-8, 1999, 2001 Free Software Foundation, Inc.
# This Makefile.in is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.


SHELL = /bin/sh

srcdir = .
top_srcdir = ../..

prefix = /home/sjhill/uClinux-dist-patched/romfs
exec_prefix = ${prefix}

bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${prefix}/share
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${prefix}/info
mandir = ${prefix}/man
includedir = ${prefix}/include
oldincludedir = /usr/include

DESTDIR =

pkgdatadir = $(datadir)/DirectFB
pkglibdir = $(libdir)/DirectFB
pkgincludedir = $(includedir)/DirectFB

top_builddir = ../..

ACLOCAL = /home/sjhill/uClinux-dist-patched/lib/directfb/missing aclocal-1.4 
AUTOCONF = autoconf
AUTOMAKE = /home/sjhill/uClinux-dist-patched/lib/directfb/missing automake-1.4
AUTOHEADER = autoheader

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL} $(AM_INSTALL_PROGRAM_FLAGS)
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL}
transform = s,x,x,

NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
host_alias = mipsel-linux
host_triplet = mipsel-unknown-linux-gnu
AS = mipsel-linux-as
ASFLAGS = 
AVIFILE_CFLAGS = 
AVIFILE_CONFIG = 
AVIFILE_LIBS = 
AVIFILE_PROVIDER = 
CC = mipsel-linux-gcc -L/home/sjhill/uClinux-dist-patched/romfs/lib -I/home/sjhill/uClinux-dist-patched/romfs/include -lm
CXX = mipsel-linux-g++
DATADIR = ${prefix}/share/directfb-0.9.13
DFB_CFLAGS = -D_REENTRANT -ffast-math -Wall -O3  -DFUSION_FAKE
DFB_CFLAGS_OMIT_FRAME_POINTER = -fomit-frame-pointer
DFB_CXXFLAGS = -D_REENTRANT -ffast-math -Wall -O3  -DFUSION_FAKE
DFB_INTERNAL_CFLAGS =  -DFUSION_FAKE
DFB_LDFLAGS = -O2 -g -fno-builtin -I/home/sjhill/uClinux-dist-patched/lib/uClibc/include -I/home/sjhill/uClinux-dist-patched -Dlinux -D__linux__ -D__uClinux__ -Dunix -DEMBED -I/home/sjhill/uClinux-dist-patched/linux-2.4.x/include
DIRECTFB_BINARY_AGE = 0
DIRECTFB_INTERFACE_AGE = 0
DIRECTFB_MAJOR_VERSION = 0
DIRECTFB_MICRO_VERSION = 13
DIRECTFB_MINOR_VERSION = 9
DIRECTFB_VERSION = 0.9.13
DLLTOOL = @DLLTOOL@
ECHO = echo
EXEEXT = 
FREETYPE_CFLAGS = 
FREETYPE_CONFIG = no
FREETYPE_LIBS = 
FREETYPE_PROVIDER = 
GFX_ATI = 
GFX_MATROX = 
GFX_TDFX = 
GIF_PROVIDER = 
INCLUDEDIR = ${prefix}/include/directfb
INTERNALINCLUDEDIR = ${prefix}/include/directfb-internal
JPEG_PROVIDER = 
LIBFLASH = 
LIBJPEG = 
LIBMPEG3 = 
LIBPNG = 
LIBTOOL = $(SHELL) $(top_builddir)/libtool
LIBZ = 
LN_S = ln -s
LT_AGE = 0
LT_CURRENT = 13
LT_RELEASE = 0.9
LT_REVISION = 0
MAINT = #
MAKEINFO = makeinfo
MODULEDIR = ${exec_prefix}/lib/directfb-0.9.13
OBJDUMP = @OBJDUMP@
OBJEXT = o
PACKAGE = DirectFB
PERL = /usr/bin/perl
PNG_PROVIDER = 
RANLIB = mipsel-linux-ranlib
SOPATH = ${exec_prefix}/lib/libdirectfb-0.9.so.13
STRIP = mipsel-linux-strip
VERSION = 0.9.13

EXTRA_DIST = cimlogo.png directfb.png

docs_headers =  	$(top_srcdir)/include/directfb_keyboard.h 	$(top_srcdir)/include/directfb.h

mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = ../../config.h
CONFIG_CLEAN_FILES = 
DIST_COMMON =  Makefile.am Makefile.in


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = tar
GZIP_ENV = --best
all: all-redirect
.SUFFIXES:
$(srcdir)/Makefile.in: # Makefile.am $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	cd $(top_srcdir) && $(AUTOMAKE) --gnu --include-deps docs/html/Makefile

Makefile: $(srcdir)/Makefile.in  $(top_builddir)/config.status
	cd $(top_builddir) \
	  && CONFIG_FILES=$(subdir)/$@ CONFIG_HEADERS= $(SHELL) ./config.status

tags: TAGS
TAGS:


distdir = $(top_builddir)/$(PACKAGE)-$(VERSION)/$(subdir)

subdir = docs/html

distdir: $(DISTFILES)
	@for file in $(DISTFILES); do \
	  d=$(srcdir); \
	  if test -d $$d/$$file; then \
	    cp -pr $$d/$$file $(distdir)/$$file; \
	  else \
	    test -f $(distdir)/$$file \
	    || ln $$d/$$file $(distdir)/$$file 2> /dev/null \
	    || cp -p $$d/$$file $(distdir)/$$file || :; \
	  fi; \
	done
info-am:
info: info-am
dvi-am:
dvi: dvi-am
check-am: all-am
check: check-am
installcheck-am:
installcheck: installcheck-am
install-exec-am:
install-exec: install-exec-am

install-data-am:
install-data: install-data-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am
install: install-am
uninstall-am:
uninstall: uninstall-am
all-am: Makefile all-local
all-redirect: all-am
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) AM_INSTALL_PROGRAM_FLAGS=-s install
installdirs:


mostlyclean-generic:

clean-generic:

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES)
	-rm -f config.cache config.log stamp-h stamp-h[0-9]*

maintainer-clean-generic:
mostlyclean-am:  mostlyclean-generic

mostlyclean: mostlyclean-am

clean-am:  clean-generic mostlyclean-am

clean: clean-am

distclean-am:  distclean-generic clean-am distclean-local
	-rm -f libtool

distclean: distclean-am

maintainer-clean-am:  maintainer-clean-generic distclean-am
	@echo "This command is intended for maintainers to use;"
	@echo "it deletes files that may require special tools to rebuild."

maintainer-clean: maintainer-clean-am

.PHONY: tags distdir info-am info dvi-am dvi check check-am \
installcheck-am installcheck install-exec-am install-exec \
install-data-am install-data install-am install uninstall-am uninstall \
all-local all-redirect all-am all installdirs mostlyclean-generic \
distclean-generic clean-generic maintainer-clean-generic clean \
mostlyclean distclean maintainer-clean


all-local: stamp-docs

stamp-docs: $(top_srcdir)/tools/gendoc.pl $(docs_headers) Makefile.am
	rootme=`pwd`; \
	$(PERL) $(top_srcdir)/tools/gendoc.pl $(docs_headers) \
	&& echo timestamp > $$rootme/stamp-docs

distclean-local:
	rm -f *.html stamp-docs

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
