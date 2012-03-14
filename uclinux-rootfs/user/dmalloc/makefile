TOOL_DIR_PREFIX=$(shell dirname $(shell which $(CC)))/..

ifneq (,$(findstring linux,$(CROSS)))
	TARGET=$(CROSS:linux-=linux)
	TARGET_ARCH ?= $(CROSS:-linux-=)
else
	ifneq (,$(findstring uclibc,$(CROSS)))
		TARGET_ARCH ?= $(CROSS:-uclibc-=)
		TARGET=$(TARGET_ARCH)-linux
	else
		echo "FIXME: Cannot determine target architecture"
		exit 99
	endif
endif

all:
	# This is optional software, so it may not be cleaned even with a make distclean
	# and will faile to build if we switch from LE to BE build and vice versa.
	-test -f Makefile && make -f Makefile distclean
	-rm -f Makefile config.cache config.h config.status
	CC=$(CC) LD=$(LD) CFLAGS="$(MY_CFLAGS)" LDFLAGS="$(MY_LDFLAGS)" CXXFLAGS="$(MY_CXXFLAGS)" CPPFLAGS="$(MY_CPPFLAGS)" \
	VERSION=\"5.5.2\" PACKAGE=\"dmalloc\" \
		./configure \
		--target=$(TARGET)		\
		--host=$(TARGET)	\
		--build=i386-linux 
	make -f Makefile

clean:
	make -f Makefile clean

distclean: clean
	make -f Makefile distclean
	-rm -f Makefile config.cache config.h config.status

romfs:
	$(ROMFSINST) -e CONFIG_USER_DMALLOC_DMALLOC	dmalloc	/bin/dmalloc

