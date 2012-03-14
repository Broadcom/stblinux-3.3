
all: makefile affinity.c
	$(CC) -o affinity affinity.c

clean: 
	rm -rf affinity

romfs:
	$(ROMFSINST) /bin/affinity
	$(ROMFSINST) /bin/twin

