AUTOMAKE_OPTIONS = ../ansi2knr

bin_PROGRAMS = @whois_BUILD@

INCLUDES = -I$(top_builddir)/include

EXTRA_PROGRAMS = whois

whois_SOURCES = whois.c

EXTRA_DIST = as_del_list ip_del_list tld_serv_list \
  make_as_del.pl  make_ip_del.pl make_tld_serv.pl \
  as_del.h ip_del.h tld_serv.h

noinst_HEADERS = data.h as_del.h ip_del.h tld_serv.h whois.h
BUILT_SOURCES=as_del.h ip_del.h tld_serv.h

LDADD = -L../libinetutils -linetutils


as_del.h: $(srcdir)/as_del_list $(srcdir)/make_as_del.pl
	perl $(srcdir)/make_as_del.pl < $(srcdir)/as_del_list > as_del.h

ip_del.h: $(srcdir)/ip_del_list $(srcdir)/make_ip_del.pl
	perl $(srcdir)/make_ip_del.pl < $(srcdir)/ip_del_list > ip_del.h

tld_serv.h: $(srcdir)/tld_serv_list $(srcdir)/make_tld_serv.pl
	perl $(srcdir)/make_tld_serv.pl < $(srcdir)/tld_serv_list > tld_serv.h
