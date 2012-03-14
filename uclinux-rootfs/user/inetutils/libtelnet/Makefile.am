AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES = -I$(top_builddir)/include @INCAUTH@

noinst_LIBRARIES = libtelnet.a

libtelnet_a_SOURCES = auth.c enc_des.c encrypt.c forward.c genget.c getent.c kerberos.c kerberos5.c misc.c read_passwd.c

noinst_HEADERS = auth-proto.h auth.h enc-proto.h encrypt.h key-proto.h misc-proto.h misc.h
