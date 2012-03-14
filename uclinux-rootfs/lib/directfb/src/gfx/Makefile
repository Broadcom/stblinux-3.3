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

SUBDIRS = generic

INCLUDES =  	-I$(top_srcdir)/include		-I$(top_srcdir)/src


CFLAGS = $(DFB_CFLAGS)

internalincludedir = $(INTERNALINCLUDEDIR)/gfx

internalinclude_HEADERS =  	clip.h				convert.h			util.h


noinst_LTLIBRARIES = libdirectfb_gfx.la

libdirectfb_gfx_la_SOURCES =  	clip.c				convert.c			util.c


libdirectfb_gfx_la_LIBADD =  	generic/libdirectfb_generic.la

mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = ../../config.h
CONFIG_CLEAN_FILES = 
LTLIBRARIES =  $(noinst_LTLIBRARIES)


DEFS = -DHAVE_CONFIG_H -I. -I$(srcdir) -I../..
CPPFLAGS = 
LDFLAGS = -O2 -g -fno-builtin -I/home/sjhill/uClinux-dist-patched/lib/uClibc/include -I/home/sjhill/uClinux-dist-patched -Dlinux -D__linux__ -D__uClinux__ -Dunix -DEMBED -I/home/sjhill/uClinux-dist-patched/linux-2.4.x/include
LIBS = -ldl -lpthread 
libdirectfb_gfx_la_LDFLAGS = 
libdirectfb_gfx_la_DEPENDENCIES =  generic/libdirectfb_generic.la
libdirectfb_gfx_la_OBJECTS =  clip.lo convert.lo util.lo
COMPILE = $(CC) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
LTCOMPILE = $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
CCLD = $(CC)
LINK = $(LIBTOOL) --mode=link $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(LDFLAGS) -o $@
HEADERS =  $(internalinclude_HEADERS)

DIST_COMMON =  Makefile.am Makefile.in


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = tar
GZIP_ENV = --best
SOURCES = $(libdirectfb_gfx_la_SOURCES)
OBJECTS = $(libdirectfb_gfx_la_OBJECTS)

all: all-redirect
.SUFFIXES:
.SUFFIXES: .S .c .lo .o .obj .s
$(srcdir)/Makefile.in: # Makefile.am $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	cd $(top_srcdir) && $(AUTOMAKE) --gnu --include-deps src/gfx/Makefile

Makefile: $(srcdir)/Makefile.in  $(top_builddir)/config.status
	cd $(top_builddir) \
	  && CONFIG_FILES=$(subdir)/$@ CONFIG_HEADERS= $(SHELL) ./config.status


mostlyclean-noinstLTLIBRARIES:

clean-noinstLTLIBRARIES:
	-test -z "$(noinst_LTLIBRARIES)" || rm -f $(noinst_LTLIBRARIES)

distclean-noinstLTLIBRARIES:

maintainer-clean-noinstLTLIBRARIES:

.c.o:
	$(COMPILE) -c $<

# FIXME: We should only use cygpath when building on Windows,
# and only if it is available.
.c.obj:
	$(COMPILE) -c `cygpath -w $<`

.s.o:
	$(COMPILE) -c $<

.S.o:
	$(COMPILE) -c $<

mostlyclean-compile:
	-rm -f *.o core *.core
	-rm -f *.$(OBJEXT)

clean-compile:

distclean-compile:
	-rm -f *.tab.c

maintainer-clean-compile:

.c.lo:
	$(LIBTOOL) --mode=compile $(COMPILE) -c $<

.s.lo:
	$(LIBTOOL) --mode=compile $(COMPILE) -c $<

.S.lo:
	$(LIBTOOL) --mode=compile $(COMPILE) -c $<

mostlyclean-libtool:
	-rm -f *.lo

clean-libtool:
	-rm -rf .libs _libs

distclean-libtool:

maintainer-clean-libtool:

libdirectfb_gfx.la: $(libdirectfb_gfx_la_OBJECTS) $(libdirectfb_gfx_la_DEPENDENCIES)
	$(LINK)  $(libdirectfb_gfx_la_LDFLAGS) $(libdirectfb_gfx_la_OBJECTS) $(libdirectfb_gfx_la_LIBADD) $(LIBS)

install-internalincludeHEADERS: $(internalinclude_HEADERS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(internalincludedir)
	@list='$(internalinclude_HEADERS)'; for p in $$list; do \
	  if test -f "$$p"; then d= ; else d="$(srcdir)/"; fi; \
	  echo " $(INSTALL_DATA) $$d$$p $(DESTDIR)$(internalincludedir)/$$p"; \
	  $(INSTALL_DATA) $$d$$p $(DESTDIR)$(internalincludedir)/$$p; \
	done

uninstall-internalincludeHEADERS:
	@$(NORMAL_UNINSTALL)
	list='$(internalinclude_HEADERS)'; for p in $$list; do \
	  rm -f $(DESTDIR)$(internalincludedir)/$$p; \
	done

# This directory's subdirectories are mostly independent; you can cd
# into them and run `make' without going through this Makefile.
# To change the values of `make' variables: instead of editing Makefiles,
# (1) if the variable is set in `config.status', edit `config.status'
#     (which will cause the Makefiles to be regenerated when you run `make');
# (2) otherwise, pass the desired values on the `make' command line.



all-recursive install-data-recursive install-exec-recursive \
installdirs-recursive install-recursive uninstall-recursive  \
check-recursive installcheck-recursive info-recursive dvi-recursive:
	@set fnord $(MAKEFLAGS); amf=$$2; \
	dot_seen=no; \
	target=`echo $@ | sed s/-recursive//`; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    dot_seen=yes; \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	   || case "$$amf" in *=*) exit 1;; *k*) fail=yes;; *) exit 1;; esac; \
	done; \
	if test "$$dot_seen" = "no"; then \
	  $(MAKE) $(AM_MAKEFLAGS) "$$target-am" || exit 1; \
	fi; test -z "$$fail"

mostlyclean-recursive clean-recursive distclean-recursive \
maintainer-clean-recursive:
	@set fnord $(MAKEFLAGS); amf=$$2; \
	dot_seen=no; \
	rev=''; list='$(SUBDIRS)'; for subdir in $$list; do \
	  rev="$$subdir $$rev"; \
	  test "$$subdir" != "." || dot_seen=yes; \
	done; \
	test "$$dot_seen" = "no" && rev=". $$rev"; \
	target=`echo $@ | sed s/-recursive//`; \
	for subdir in $$rev; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	   || case "$$amf" in *=*) exit 1;; *k*) fail=yes;; *) exit 1;; esac; \
	done && test -z "$$fail"
tags-recursive:
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  test "$$subdir" = . || (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) tags); \
	done

tags: TAGS

ID: $(HEADERS) $(SOURCES) $(LISP)
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	here=`pwd` && cd $(srcdir) \
	  && mkid -f$$here/ID $$unique $(LISP)

TAGS: tags-recursive $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) $(LISP)
	tags=; \
	here=`pwd`; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
   if test "$$subdir" = .; then :; else \
	    test -f $$subdir/TAGS && tags="$$tags -i $$here/$$subdir/TAGS"; \
   fi; \
	done; \
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	test -z "$(ETAGS_ARGS)$$unique$(LISP)$$tags" \
	  || (cd $(srcdir) && etags -o $$here/TAGS $(ETAGS_ARGS) $$tags  $$unique $(LISP))

mostlyclean-tags:

clean-tags:

distclean-tags:
	-rm -f TAGS ID

maintainer-clean-tags:

distdir = $(top_builddir)/$(PACKAGE)-$(VERSION)/$(subdir)

subdir = src/gfx

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
	for subdir in $(SUBDIRS); do \
	  if test "$$subdir" = .; then :; else \
	    test -d $(distdir)/$$subdir \
	    || mkdir $(distdir)/$$subdir \
	    || exit 1; \
	    chmod 777 $(distdir)/$$subdir; \
	    (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) top_distdir=../$(top_distdir) distdir=../$(distdir)/$$subdir distdir) \
	      || exit 1; \
	  fi; \
	done
clip.lo clip.o : clip.c ../../include/directfb.h \
	../../include/directfb_keyboard.h ../../src/misc/util.h \
	../../config.h ../../src/core/coredefs.h ../../src/misc/conf.h \
	../../src/core/fusion/fusion_types.h clip.h
convert.lo convert.o : convert.c ../../include/directfb.h \
	../../include/directfb_keyboard.h ../../src/core/core.h \
	../../src/core/fusion/fusion_types.h \
	../../src/core/fusion/lock.h ../../src/core/coretypes.h \
	../../src/core/coredefs.h ../../config.h ../../src/misc/conf.h \
	../../src/misc/util.h ../../src/misc/memcpy.h convert.h
util.lo util.o : util.c ../../include/directfb.h \
	../../include/directfb_keyboard.h ../../src/core/coretypes.h \
	../../src/core/state.h ../../src/core/gfxcard.h \
	../../src/core/fusion/lock.h \
	../../src/core/fusion/fusion_types.h util.h \
	../../src/core/surfaces.h ../../src/core/fusion/object.h \
	../../src/core/fusion/list.h ../../src/core/fusion/ref.h \
	../../src/core/fusion/reactor.h

info-am:
info: info-recursive
dvi-am:
dvi: dvi-recursive
check-am: all-am
check: check-recursive
installcheck-am:
installcheck: installcheck-recursive
install-exec-am:
install-exec: install-exec-recursive

install-data-am: install-internalincludeHEADERS
install-data: install-data-recursive

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am
install: install-recursive
uninstall-am: uninstall-internalincludeHEADERS
uninstall: uninstall-recursive
all-am: Makefile $(LTLIBRARIES) $(HEADERS)
all-redirect: all-recursive
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) AM_INSTALL_PROGRAM_FLAGS=-s install
installdirs: installdirs-recursive
installdirs-am:
	$(mkinstalldirs)  $(DESTDIR)$(internalincludedir)


mostlyclean-generic:

clean-generic:

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES)
	-rm -f config.cache config.log stamp-h stamp-h[0-9]*

maintainer-clean-generic:
mostlyclean-am:  mostlyclean-noinstLTLIBRARIES mostlyclean-compile \
		mostlyclean-libtool mostlyclean-tags \
		mostlyclean-generic

mostlyclean: mostlyclean-recursive

clean-am:  clean-noinstLTLIBRARIES clean-compile clean-libtool \
		clean-tags clean-generic mostlyclean-am

clean: clean-recursive

distclean-am:  distclean-noinstLTLIBRARIES distclean-compile \
		distclean-libtool distclean-tags distclean-generic \
		clean-am
	-rm -f libtool

distclean: distclean-recursive

maintainer-clean-am:  maintainer-clean-noinstLTLIBRARIES \
		maintainer-clean-compile maintainer-clean-libtool \
		maintainer-clean-tags maintainer-clean-generic \
		distclean-am
	@echo "This command is intended for maintainers to use;"
	@echo "it deletes files that may require special tools to rebuild."

maintainer-clean: maintainer-clean-recursive

.PHONY: mostlyclean-noinstLTLIBRARIES distclean-noinstLTLIBRARIES \
clean-noinstLTLIBRARIES maintainer-clean-noinstLTLIBRARIES \
mostlyclean-compile distclean-compile clean-compile \
maintainer-clean-compile mostlyclean-libtool distclean-libtool \
clean-libtool maintainer-clean-libtool uninstall-internalincludeHEADERS \
install-internalincludeHEADERS install-data-recursive \
uninstall-data-recursive install-exec-recursive \
uninstall-exec-recursive installdirs-recursive uninstalldirs-recursive \
all-recursive check-recursive installcheck-recursive info-recursive \
dvi-recursive mostlyclean-recursive distclean-recursive clean-recursive \
maintainer-clean-recursive tags tags-recursive mostlyclean-tags \
distclean-tags clean-tags maintainer-clean-tags distdir info-am info \
dvi-am dvi check check-am installcheck-am installcheck install-exec-am \
install-exec install-data-am install-data install-am install \
uninstall-am uninstall all-redirect all-am all installdirs-am \
installdirs mostlyclean-generic distclean-generic clean-generic \
maintainer-clean-generic clean mostlyclean distclean maintainer-clean


# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
