AUTOMAKE_OPTIONS = ../ansi2knr

bin_PROGRAMS = @telnet_BUILD@

EXTRA_PROGRAMS = telnet

telnet_SOURCES = authenc.c commands.c main.c network.c ring.c sys_bsd.c \
	telnet.c terminal.c tn3270.c utilities.c

noinst_HEADERS = defines.h externs.h fdset.h general.h ring.h types.h

man_MANS = telnet.1

INCLUDES = -DTERMCAP -DLINEMODE -DKLUDGELINEMODE -DENV_HACK -I$(top_builddir)/include

LIBTERMCAP = @LIBTERMCAP@
LIBCRYPT = @LIBCRYPT@

LDADD = -L../libinetutils -L../libtelnet -linetutils -ltelnet \
	$(LIBTERMCAP) $(LIBCRYPT) $(LIBAUTH)

EXTRA_DIST = $(man_MANS)
