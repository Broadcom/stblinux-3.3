include $(ROOTDIR)/config.arch

all:
	./configure --host=$(MACHINE)-linux --prefix=$(ROMFSDIR)
	make -f Makefile SUBDIRS="."

all-% install-% clean-% distclean-%:
	$(MAKE) -f Makefile $@

clean:
	make -f Makefile clean SUBDIRS="."

distclean: clean
	-make -f Makefile distclean SUBDIRS="."
	rm -f Makefile

romfs:
	make -f Makefile install SUBDIRS="."
