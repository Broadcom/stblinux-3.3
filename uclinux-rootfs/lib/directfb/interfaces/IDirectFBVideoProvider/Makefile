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

idirectfbvideoproviderdir = $(MODULEDIR)/interfaces/IDirectFBVideoProvider
#AVIFILE_PROVIDER_LTLIB = libidirectfbvideoprovider_avifile.la
AVIFILE_PROVIDER_LTLIB = 
#LIBMPEG3_PROVIDER_LTLIB = libidirectfbvideoprovider_libmpeg3.la
LIBMPEG3_PROVIDER_LTLIB = 
#FLASH_PROVIDER_LTLIB = libidirectfbvideoprovider_swf.la
FLASH_PROVIDER_LTLIB = 

INCLUDES =  	-I$(top_srcdir)/include		-I$(top_srcdir)/src 		-DDATADIR=\"${prefix}/share/directfb-0.9.13\"


CFLAGS = $(DFB_CFLAGS)
CXXFLAGS = $(DFB_CXXFLAGS) $(AVIFILE_CFLAGS)

idirectfbvideoprovider_LTLIBRARIES =  	libidirectfbvideoprovider_v4l.la		$(AVIFILE_PROVIDER_LTLIB)			$(FLASH_PROVIDER_LTLIB)				$(LIBMPEG3_PROVIDER_LTLIB)


libidirectfbvideoprovider_avifile_la_SOURCES = idirectfbvideoprovider_avifile.cpp

libidirectfbvideoprovider_avifile_la_LIBADD = $(AVIFILE_LIBS)

libidirectfbvideoprovider_avifile_la_LDFLAGS = -avoid-version -module

libidirectfbvideoprovider_swf_la_SOURCES = idirectfbvideoprovider_swf.c

libidirectfbvideoprovider_swf_la_LIBADD = $(LIBFLASH)

libidirectfbvideoprovider_swf_la_LDFLAGS = -avoid-version -module

libidirectfbvideoprovider_libmpeg3_la_SOURCES = idirectfbvideoprovider_libmpeg3.c

libidirectfbvideoprovider_libmpeg3_la_LIBADD = $(LIBMPEG3)

libidirectfbvideoprovider_libmpeg3_la_LDFLAGS = -avoid-version -module

libidirectfbvideoprovider_v4l_la_SOURCES = idirectfbvideoprovider_v4l.c

libidirectfbvideoprovider_v4l_la_LDFLAGS = -avoid-version -module
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = ../../config.h
CONFIG_CLEAN_FILES = 
LTLIBRARIES =  $(idirectfbvideoprovider_LTLIBRARIES)


DEFS = -DHAVE_CONFIG_H -I. -I$(srcdir) -I../..
CPPFLAGS = 
LDFLAGS = -O2 -g -fno-builtin -I/home/sjhill/uClinux-dist-patched/lib/uClibc/include -I/home/sjhill/uClinux-dist-patched -Dlinux -D__linux__ -D__uClinux__ -Dunix -DEMBED -I/home/sjhill/uClinux-dist-patched/linux-2.4.x/include
LIBS = -ldl -lpthread 
libidirectfbvideoprovider_v4l_la_LIBADD = 
libidirectfbvideoprovider_v4l_la_OBJECTS =  \
idirectfbvideoprovider_v4l.lo
libidirectfbvideoprovider_avifile_la_DEPENDENCIES = 
libidirectfbvideoprovider_avifile_la_OBJECTS =  \
idirectfbvideoprovider_avifile.lo
libidirectfbvideoprovider_swf_la_DEPENDENCIES = 
libidirectfbvideoprovider_swf_la_OBJECTS =  \
idirectfbvideoprovider_swf.lo
libidirectfbvideoprovider_libmpeg3_la_DEPENDENCIES = 
libidirectfbvideoprovider_libmpeg3_la_OBJECTS =  \
idirectfbvideoprovider_libmpeg3.lo
CXXCOMPILE = $(CXX) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS)
LTCXXCOMPILE = $(LIBTOOL) --mode=compile $(CXX) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS)
CXXLD = $(CXX)
CXXLINK = $(LIBTOOL) --mode=link $(CXXLD) $(AM_CXXFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@
COMPILE = $(CC) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
LTCOMPILE = $(LIBTOOL) --mode=compile $(CC) $(DEFS) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
CCLD = $(CC)
LINK = $(LIBTOOL) --mode=link $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(LDFLAGS) -o $@
DIST_COMMON =  Makefile.am Makefile.in


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = tar
GZIP_ENV = --best
SOURCES = $(libidirectfbvideoprovider_v4l_la_SOURCES) $(libidirectfbvideoprovider_avifile_la_SOURCES) $(libidirectfbvideoprovider_swf_la_SOURCES) $(libidirectfbvideoprovider_libmpeg3_la_SOURCES)
OBJECTS = $(libidirectfbvideoprovider_v4l_la_OBJECTS) $(libidirectfbvideoprovider_avifile_la_OBJECTS) $(libidirectfbvideoprovider_swf_la_OBJECTS) $(libidirectfbvideoprovider_libmpeg3_la_OBJECTS)

all: all-redirect
.SUFFIXES:
.SUFFIXES: .S .c .cpp .lo .o .obj .s
$(srcdir)/Makefile.in: # Makefile.am $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	cd $(top_srcdir) && $(AUTOMAKE) --gnu --include-deps interfaces/IDirectFBVideoProvider/Makefile

Makefile: $(srcdir)/Makefile.in  $(top_builddir)/config.status
	cd $(top_builddir) \
	  && CONFIG_FILES=$(subdir)/$@ CONFIG_HEADERS= $(SHELL) ./config.status


mostlyclean-idirectfbvideoproviderLTLIBRARIES:

clean-idirectfbvideoproviderLTLIBRARIES:
	-test -z "$(idirectfbvideoprovider_LTLIBRARIES)" || rm -f $(idirectfbvideoprovider_LTLIBRARIES)

distclean-idirectfbvideoproviderLTLIBRARIES:

maintainer-clean-idirectfbvideoproviderLTLIBRARIES:

install-idirectfbvideoproviderLTLIBRARIES: $(idirectfbvideoprovider_LTLIBRARIES)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(idirectfbvideoproviderdir)
	@list='$(idirectfbvideoprovider_LTLIBRARIES)'; for p in $$list; do \
	  if test -f $$p; then \
	    echo "$(LIBTOOL)  --mode=install $(INSTALL) $$p $(DESTDIR)$(idirectfbvideoproviderdir)/$$p"; \
	    $(LIBTOOL)  --mode=install $(INSTALL) $$p $(DESTDIR)$(idirectfbvideoproviderdir)/$$p; \
	  else :; fi; \
	done

uninstall-idirectfbvideoproviderLTLIBRARIES:
	@$(NORMAL_UNINSTALL)
	list='$(idirectfbvideoprovider_LTLIBRARIES)'; for p in $$list; do \
	  $(LIBTOOL)  --mode=uninstall rm -f $(DESTDIR)$(idirectfbvideoproviderdir)/$$p; \
	done

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

libidirectfbvideoprovider_v4l.la: $(libidirectfbvideoprovider_v4l_la_OBJECTS) $(libidirectfbvideoprovider_v4l_la_DEPENDENCIES)
	$(LINK) -rpath $(idirectfbvideoproviderdir) $(libidirectfbvideoprovider_v4l_la_LDFLAGS) $(libidirectfbvideoprovider_v4l_la_OBJECTS) $(libidirectfbvideoprovider_v4l_la_LIBADD) $(LIBS)

libidirectfbvideoprovider_avifile.la: $(libidirectfbvideoprovider_avifile_la_OBJECTS) $(libidirectfbvideoprovider_avifile_la_DEPENDENCIES)
	$(CXXLINK) -rpath $(idirectfbvideoproviderdir) $(libidirectfbvideoprovider_avifile_la_LDFLAGS) $(libidirectfbvideoprovider_avifile_la_OBJECTS) $(libidirectfbvideoprovider_avifile_la_LIBADD) $(LIBS)

libidirectfbvideoprovider_swf.la: $(libidirectfbvideoprovider_swf_la_OBJECTS) $(libidirectfbvideoprovider_swf_la_DEPENDENCIES)
	$(LINK) -rpath $(idirectfbvideoproviderdir) $(libidirectfbvideoprovider_swf_la_LDFLAGS) $(libidirectfbvideoprovider_swf_la_OBJECTS) $(libidirectfbvideoprovider_swf_la_LIBADD) $(LIBS)

libidirectfbvideoprovider_libmpeg3.la: $(libidirectfbvideoprovider_libmpeg3_la_OBJECTS) $(libidirectfbvideoprovider_libmpeg3_la_DEPENDENCIES)
	$(LINK) -rpath $(idirectfbvideoproviderdir) $(libidirectfbvideoprovider_libmpeg3_la_LDFLAGS) $(libidirectfbvideoprovider_libmpeg3_la_OBJECTS) $(libidirectfbvideoprovider_libmpeg3_la_LIBADD) $(LIBS)
.cpp.o:
	$(CXXCOMPILE) -c $<
.cpp.obj:
	$(CXXCOMPILE) -c `cygpath -w $<`
.cpp.lo:
	$(LTCXXCOMPILE) -c $<

tags: TAGS

ID: $(HEADERS) $(SOURCES) $(LISP)
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	here=`pwd` && cd $(srcdir) \
	  && mkid -f$$here/ID $$unique $(LISP)

TAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) $(LISP)
	tags=; \
	here=`pwd`; \
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

subdir = interfaces/IDirectFBVideoProvider

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
idirectfbvideoprovider_libmpeg3.lo idirectfbvideoprovider_libmpeg3.o : \
	idirectfbvideoprovider_libmpeg3.c ../../include/directfb.h \
	../../include/directfb_keyboard.h \
	../../include/directfb_internals.h ../../src/core/coretypes.h \
	../../src/media/idirectfbvideoprovider.h ../../src/misc/util.h \
	../../config.h ../../src/core/coredefs.h ../../src/misc/conf.h \
	../../src/core/fusion/fusion_types.h ../../src/misc/mem.h \
	../../src/core/layers.h ../../src/core/fusion/lock.h \
	../../src/core/state.h ../../src/core/surfaces.h \
	../../src/core/fusion/object.h ../../src/core/fusion/list.h \
	../../src/core/fusion/ref.h ../../src/core/fusion/reactor.h \
	../../src/core/gfxcard.h ../../src/gfx/convert.h \
	../../src/misc/memcpy.h ../../src/display/idirectfbsurface.h \
	../../src/interface_implementation.h ../../src/core/input.h
idirectfbvideoprovider_swf.lo idirectfbvideoprovider_swf.o : \
	idirectfbvideoprovider_swf.c ../../include/directfb.h \
	../../include/directfb_keyboard.h \
	../../include/directfb_internals.h ../../src/core/coretypes.h \
	../../src/media/idirectfbvideoprovider.h \
	../../src/core/coredefs.h ../../config.h ../../src/misc/conf.h \
	../../src/core/fusion/fusion_types.h ../../src/misc/util.h \
	../../src/core/state.h ../../src/core/gfxcard.h \
	../../src/core/fusion/lock.h ../../src/core/layers.h \
	../../src/core/surfaces.h ../../src/core/fusion/object.h \
	../../src/core/fusion/list.h ../../src/core/fusion/ref.h \
	../../src/core/fusion/reactor.h \
	../../src/display/idirectfbsurface.h ../../src/misc/mem.h \
	../../src/interface_implementation.h ../../src/core/input.h
idirectfbvideoprovider_v4l.lo idirectfbvideoprovider_v4l.o : \
	idirectfbvideoprovider_v4l.c ../../include/directfb.h \
	../../include/directfb_keyboard.h \
	../../include/directfb_internals.h ../../src/core/coretypes.h \
	../../src/media/idirectfbvideoprovider.h ../../src/core/core.h \
	../../src/core/fusion/fusion_types.h \
	../../src/core/fusion/lock.h ../../src/core/coredefs.h \
	../../config.h ../../src/misc/conf.h ../../src/misc/util.h \
	../../src/core/state.h ../../src/core/gfxcard.h \
	../../src/core/layers.h ../../src/core/surfaces.h \
	../../src/core/fusion/object.h ../../src/core/fusion/list.h \
	../../src/core/fusion/ref.h ../../src/core/fusion/reactor.h \
	../../src/core/surfacemanager.h \
	../../src/display/idirectfbsurface.h ../../src/misc/mem.h \
	../../src/misc/memcpy.h ../../src/interface_implementation.h \
	../../src/core/input.h

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

install-data-am: install-idirectfbvideoproviderLTLIBRARIES
install-data: install-data-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am
install: install-am
uninstall-am: uninstall-idirectfbvideoproviderLTLIBRARIES
uninstall: uninstall-am
all-am: Makefile $(LTLIBRARIES)
all-redirect: all-am
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) AM_INSTALL_PROGRAM_FLAGS=-s install
installdirs:
	$(mkinstalldirs)  $(DESTDIR)$(idirectfbvideoproviderdir)


mostlyclean-generic:

clean-generic:

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES)
	-rm -f config.cache config.log stamp-h stamp-h[0-9]*

maintainer-clean-generic:
mostlyclean-am:  mostlyclean-idirectfbvideoproviderLTLIBRARIES \
		mostlyclean-compile mostlyclean-libtool \
		mostlyclean-tags mostlyclean-generic

mostlyclean: mostlyclean-am

clean-am:  clean-idirectfbvideoproviderLTLIBRARIES clean-compile \
		clean-libtool clean-tags clean-generic mostlyclean-am

clean: clean-am

distclean-am:  distclean-idirectfbvideoproviderLTLIBRARIES \
		distclean-compile distclean-libtool distclean-tags \
		distclean-generic clean-am
	-rm -f libtool

distclean: distclean-am

maintainer-clean-am:  maintainer-clean-idirectfbvideoproviderLTLIBRARIES \
		maintainer-clean-compile maintainer-clean-libtool \
		maintainer-clean-tags maintainer-clean-generic \
		distclean-am
	@echo "This command is intended for maintainers to use;"
	@echo "it deletes files that may require special tools to rebuild."

maintainer-clean: maintainer-clean-am

.PHONY: mostlyclean-idirectfbvideoproviderLTLIBRARIES \
distclean-idirectfbvideoproviderLTLIBRARIES \
clean-idirectfbvideoproviderLTLIBRARIES \
maintainer-clean-idirectfbvideoproviderLTLIBRARIES \
uninstall-idirectfbvideoproviderLTLIBRARIES \
install-idirectfbvideoproviderLTLIBRARIES mostlyclean-compile \
distclean-compile clean-compile maintainer-clean-compile \
mostlyclean-libtool distclean-libtool clean-libtool \
maintainer-clean-libtool tags mostlyclean-tags distclean-tags \
clean-tags maintainer-clean-tags distdir info-am info dvi-am dvi check \
check-am installcheck-am installcheck install-exec-am install-exec \
install-data-am install-data install-am install uninstall-am uninstall \
all-redirect all-am all installdirs mostlyclean-generic \
distclean-generic clean-generic maintainer-clean-generic clean \
mostlyclean distclean maintainer-clean


# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
