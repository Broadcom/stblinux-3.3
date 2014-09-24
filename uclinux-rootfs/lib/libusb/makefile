include $(ROOTDIR)/config.arch

all:
	./configure --host=$(MACHINE)-linux --prefix=$(ROMFSDIR)
	make -f Makefile SUBDIRS="."

all-% install-% clean-% distclean-%:
	$(MAKE) -f Makefile $@

clean:
	[ ! -f Makefile ] || make -f Makefile clean SUBDIRS="."

distclean: clean
	[ ! -f Makefile ] || make -f Makefile distclean SUBDIRS="."
	rm -f Makefile

romfs:
	make -f Makefile install SUBDIRS="."

.PHONY: all clean distclean romfs all-% install-% clean-% distclean-%
