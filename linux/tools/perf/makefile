include $(ROOTDIR)/config.arch

MY_CFLAGS = -I$(ROOTDIR)/lib/elfutils/libelf -I$(ROOTDIR)/lib/gettext/intl -w
MY_LDFLAGS = -L$(ROOTDIR)/lib/elfutils/libelf \
	-L$(ROOTDIR)/lib/gettext/intl/.libs \
	-lelf -lintl
COMMON_MAKEARGS = ARCH=mips CROSS_COMPILE=mipsel-linux- \
	EXTRA_CFLAGS="$(MY_CFLAGS)" LDFLAGS="$(MY_LDFLAGS)"
OPTION_MAKEARGS = NO_DWARF=1 WERROR=0 NO_LIBPYTHON=1

all :
	make -f Makefile prefix=$(ROOTDIR)/romfs $(COMMON_MAKEARGS) \
	$(OPTION_MAKEARGS)

all-% install-% clean-% distclean-%:
	$(MAKE) -f Makefile $@

clean:
	make -f Makefile $(COMMON_MAKEARGS) clean

distclean: clean
	-make -f Makefile $(COMMON_MAKEARGS) distclean

romfs:
	$(STRIP) perf
	$(ROMFSINST) perf /bin/perf
