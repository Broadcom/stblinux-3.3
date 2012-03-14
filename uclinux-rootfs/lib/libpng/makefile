all:
	ln -fs scripts/makefile.linux Makefile
	make CC=$(CC) RANLIB=$(RANLIB) AR=$(AR) prefix=$(ROMFSDIR) -f Makefile
	make CC=$(CC) prefix=$(ROMFSDIR) -f Makefile install
	make CC=$(CC) prefix=$(ROMFSDIR) -f Makefile install-headers

distclean clean:
	make -f Makefile clean
	rm -f Makefile

romfs:
	make CC=$(CC) prefix=$(ROMFSDIR) -f Makefile install-shared
	rm -f $(ROMFSDIR)/bin/libpng*-config
	rm -f $(ROMFSDIR)/lib/libpng*.*a
	rm -rf $(ROMFSDIR)/lib/pkgconfig
