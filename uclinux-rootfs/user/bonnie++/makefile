include $(ROOTDIR)/config.arch

all:
	./configure --host=$(MACHINE)-linux
	make -f Makefile

clean:
	make -f Makefile clean

distclean: clean
	make -f Makefile clean

romfs:
	$(ROMFSINST) -e CONFIG_USER_BONNIEXX_BONNIEXX bonnie++ /bin/bonnie++
	$(ROMFSINST) -e CONFIG_USER_BONNIEXX_BONNIEXX zcav /bin/zcav
