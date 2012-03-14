#
# Makefile for libtermcap. It is intended for Linux/ELF only.
#

LIBNAME=termcap

VERSION=2.0.8
SONAME_VERSION=2

# CC=gcc   <--- set by makefile
# CFLAGS=-Os -I.  <--- set by makefile
# AR=ar    <--- set by makefile

ARFLAGS=ucvr
MAKEINFO=makeinfo

OWNER=root.root

STATIC_LIB=lib$(LIBNAME).a
SHARED_LIB=lib$(LIBNAME).so.$(VERSION)
SONAME_SHARED_LIB=lib$(LIBNAME).so.$(SONAME_VERSION)

TARGETS=$(STATIC_LIB) $(SHARED_LIB)

# Where is include and dir located?
prefix=$(PREFIX)

.c.o:
	$(CC) $(CFLAGS) -c $<
	$(CC) $(CFLAGS) -fPIC -c $< -o pic/$@

SRCS = termcap.c tparam.c version.c
OBJS= $(SRCS:.c=.o)

# all lib: pic .depend $(TARGETS) info
all lib: pic .depend $(TARGETS)

$(STATIC_LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

$(SHARED_LIB): $(OBJS)
	cd pic; \
	$(CC) -shared -o ../$@ -Wl,-soname,$(SONAME_SHARED_LIB) $(OBJS) -lc
	ln -sf $(SHARED_LIB) libtermcap.so
pic:
	-if [ ! -d pic ]; then mkdir pic; fi

install: lib install-dirs install-data
	-if [ -f $(prefix)/lib/$(SHARED_LIB) ]; then \
	   mkdir -p $(prefix)/lib/backup; \
	   mv $(prefix)/lib/$(SHARED_LIB) \
		$(prefix)/lib/backup/$(SHARED_LIB).$$$$; \
	fi
	-if [ $(prefix) = "/" ]; then \
	  if [ -x /sbin/ldconfig -o -x /etc/ldconfig ]; then \
	    ldconfig; \
	  fi; \
	  cp -f $(STATIC_LIB) /usr/lib; \
	  cp $(SHARED_LIB) /lib; \
	  rm -f /lib/libtermcap.so; \
	  rm -f /usr/lib/libtermcap.so; \
	  ln -s /lib/$(SHARED_LIB) /usr/lib/libtermcap.so; \
	  cp -f termcap.h /usr/include; \
	  cp termcap.info* /usr/info; \
	  chown $(OWNER) \
		/usr/info/termcap.info* \
		/usr/lib/$(STATIC_LIB) \
		/usr/lib/libtermcap.so \
		/usr/include/termcap.h; \
	else \
	  cp -f $(STATIC_LIB) $(prefix)/usr/lib; \
	  rm -f $(prefix)/lib/libtermcap.so; \
	  cp -f $(SHARED_LIB) $(prefix)/lib; \
	  ln -sf $(SHARED_LIB) $(prefix)/lib/$(SONAME_SHARED_LIB); \
	  ln -sf $(SHARED_LIB) $(prefix)/lib/libtermcap.so; \
	fi

install-dirs:
	-if [ $(prefix) = "/" ]; then \
	  mkdir -p /usr/lib /usr/info /usr/include /etc /lib; \
	fi

install-data:
	-if [ $(prefix) = "/" ]; then \
	  cp termcap.src /etc/termcap; \
	  #chown $(OWNER) /etc/termcap; \
	fi

info: termcap.info

termcap.info: termcap.texi
	$(MAKEINFO) termcap.texi --output=$@

clean:
	-rm -fr .depend core a.out *.o *.so pic $(TARGETS)

distclean: clean

depend: .depend

.depend: $(SRCS) Makefile
	$(CC) $(CFLAGS) -M $(SRCS) | \
		sed -e 's,^[ 	]*\(.*\.o\)[ 	]*:,\1 pic/\1:,' > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif
