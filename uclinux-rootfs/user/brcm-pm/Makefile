include $(ROOTDIR)/config.arch

CFLAGS += -I. -L. -Wall

.PHONY: all
all: libbrcm-pm.a pmtest

libbrcm-pm.a: pmlib.o
	$(AR) r $@ $<
	$(RANLIB) $@

pmlib.o: pmlib.c pmlib.h
	$(CC) $(CFLAGS) -c $< -o $@

pmtest: pmtest.c libbrcm-pm.a
	$(CC) $(CFLAGS) $< -o $@ -lbrcm-pm

.PHONY: clean distclean romfs
clean:
	rm -f libbrcm-pm.a pmtest pmlib.o

distclean: clean

romfs:
	$(ROMFSINST) -e CONFIG_USER_BRCM_PM pmtest /bin/pmtest
