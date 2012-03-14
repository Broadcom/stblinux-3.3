default:
	make LDFLAGS='$(filter-out -Wl, $(LDFLAGS))' \
		CFLAGS="-fPIC" \
		CFG="--host=$(MACHINE)-linux --prefix=$(ROMFSDIR)" -f Makefile
	make -f Makefile
	make -f Makefile install

distclean clean:
	make -f Makefile distclean

romfs:
	rm -f $(ROMFSDIR)/bin/freetype-config
	rm -f $(ROMFSDIR)/lib/libfreetype.*a
	rm -f $(ROMFSDIR)/share/aclocal/freetype.m4
