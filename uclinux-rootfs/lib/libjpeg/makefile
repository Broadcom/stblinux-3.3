default:
	CC=$(CC) RANLIB=$(RANLIB) LDFLAGS='$(filter-out -Wl, $(LDFLAGS))' CFLAGS='$(CFLAGS)' \
	./configure LIBTOOL=$(ROOTDIR)/tools/libtool --host=`arch`-linux-gnu --enable-shared --prefix=$(ROMFSDIR)
	make CC=$(CC) LIBTOOL=$(ROOTDIR)/tools/libtool -f Makefile
	make CC=$(CC) LIBTOOL=$(ROOTDIR)/tools/libtool -f Makefile install-lib

distclean clean:
	make -f Makefile distclean

romfs:
	rm -f $(ROMFSDIR)/lib/libjpeg.*a
