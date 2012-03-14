AUTOMAKE_OPTIONS = ../ansi2knr

INCLUDES = -I$(top_builddir)/include

noinst_LIBRARIES = libicmp.a

libicmp_a_SOURCES = libping.c \
 icmp_echo.c \
 icmp_timestamp.c \
 icmp_address.c \
 icmp_cksum.c

noinst_HEADERS = icmp.h ping.h 
