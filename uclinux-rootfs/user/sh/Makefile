# Makefile for minix-sh
AXIS_TOP_DIR=.
AXIS_USABLE_LIBS = UCLIBC GLIBC
include ./Rules.make

USE_READLINE = false
HAVE_MALLOC = false

CFLAGS	+= -DCOMPIPE -DUSE_SIGSETJMP

ifeq ($(strip $(HAVE_MALLOC)),true)
CFLAGS	+= -DHAVE_MALLOC
endif

ifeq ($(strip $(USE_READLINE)),true)
CFLAGS	+= -DUSE_READLINE
LDLIBS	+= -lreadline -ltermcap
endif

PROGS	= mish

OBJ = sh1.o sh2.o sh3.o sh4.o sh5.o sh6.o

all:	$(PROGS)

$(PROGS):	$(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ):	sh.h Makefile

install: all
	$(INSTALL) -d $(prefix)/bin
	$(INSTALL) -m 0755 mish $(prefix)/bin
	
romfs:
	$(ROMFSINST) /bin/$(PROGS)

clean:
	rm -f mish *.coff *.o *.bak core
