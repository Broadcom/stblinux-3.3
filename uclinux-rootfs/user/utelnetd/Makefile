PROGS     = utelnetd

INSTDIR   = $(prefix)/bin/
INSTMODE  = 0755
INSTOWNER = root
INSTGROUP = root

OBJS = utelnetd.o

all: $(PROGS)

CFLAGS   += -DSHELLPATH=\"$(run_prefix)/bin/login\" -Wall

ifneq ("","$(DEBUG)")
CFLAGS   += -DDEBUG -g
endif

$(PROGS): $(OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

.PHONY: install
install: $(PROGS)
	$(INSTALL) -d $(INSTDIR)
	$(INSTALL) -m $(INSTMODE) -o $(INSTOWNER) -g $(INSTGROUP) $(PROGS) $(INSTDIR)

romfs:
	$(ROMFSINST) /sbin/$(PROGS)

.PHONY: clean
clean:
	rm -f $(PROGS) *.o core
