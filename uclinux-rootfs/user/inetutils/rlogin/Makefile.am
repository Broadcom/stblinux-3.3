AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES = -I$(top_builddir)/include

bin_PROGRAMS = @rlogin_BUILD@

EXTRA_PROGRAMS = rlogin

rlogin_SOURCES = rlogin.c

man_MANS = rlogin.1

LDADD = -L../libinetutils -linetutils

EXTRA_DIST = $(man_MANS)

SUIDMODE = -o root -m 4775

install-exec-hook:
	-@$(INSTALL_PROGRAM) $(bin_PROGRAMS) $(SUIDMODE) $(AM_INSTALL_PROGRAM_FLAGS) $(DESTDIR)$(bindir)/`echo $(bin_PROGRAMS)|sed '$(transform)'` ; \
	if test $$? -ne 0; then \
		echo 'INSTALL-ERROR:'; \
		echo 'INSTALL-ERROR: THIS PROGRAM($(bin_PROGRAMS)) MUST BE INSTALLED SETUID ROOT'; \
		echo 'INSTALL-ERROR: $(INSTALL_PROGRAM) $(bin_PROGRAMS) $(SUIDMODE) $(DESTDIR)$(bindir)/$(bin_PROGRAMS)'; \
		echo 'INSTALL-ERROR:'; \
	fi
