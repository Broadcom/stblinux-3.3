
EXEC = dhcpcd
OBJS = arp.o  buildmsg.o  cache.o  client.o  dhcpcd.o  dhcpconfig.o  peekfd.o  signals.o  udpipgen.o
CFLAGS += -D_BSD_SOURCE=1

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

