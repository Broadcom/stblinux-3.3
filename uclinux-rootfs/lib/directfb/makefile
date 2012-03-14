FLAGS=-L$(ROMFSDIR)/lib -I$(ROMFSDIR)/include -lm

default:
	make all

Makefile:	configure
	CC="$(CC) $(FLAGS)" CPP="$(CC) -E" LD="$(LD) $(FLAGS)" \
		LDFLAGS='$(filter-out -Wl, $(LDFLAGS))' CFLAGS='$(CFLAGS)' \
		./configure --build=`arch`-linux-gnu --target=$(MACHINE)-linux-gnu --includedir=$(ROOTDIR)/$(LINUXDIR)/include --prefix=$(ROMFSDIR) \
		--enable-shared --disable-mmx --disable-avifile --disable-gif

configure:	configure.in
	autoconf

romfs:
	make install
	rm -f $(ROMFSDIR)/bin/directfb-*
#	rm -f $(ROMFSDIR)/lib/libdirectfb.*a
	rm -rf $(ROMFSDIR)/lib/pkgconfig

include Makefile
