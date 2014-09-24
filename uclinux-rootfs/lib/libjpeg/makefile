default:
	CC=$(CC) RANLIB=$(RANLIB) LDFLAGS='$(filter-out -Wl, $(LDFLAGS))' CFLAGS='$(CFLAGS)' \
	./configure LIBTOOL=$(ROOTDIR)/tools/libtool --host=`arch`-linux-gnu --enable-shared --prefix=$(ROMFSDIR)
	make CC=$(CC) LIBTOOL=$(ROOTDIR)/tools/libtool -f Makefile
	make CC=$(CC) LIBTOOL=$(ROOTDIR)/tools/libtool -f Makefile install-lib

clean distclean:
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

romfs:
	rm -f $(ROMFSDIR)/lib/libjpeg.*a

.PHONY: default clean distclean romfs
