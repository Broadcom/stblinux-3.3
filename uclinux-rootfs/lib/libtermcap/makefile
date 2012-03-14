include $(ROOTDIR)/config.arch

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
PREFIX=$(ROOTDIR)/romfs
MY_CFLAGS +=

CC=$(TARGET)-gcc
LD=$(TARGET)-ld
AR=$(TARGET)-ar

CFLAGS=-Os -I.

ARFLAGS=ucvr

all:
	make -f Makefile

clean:
	make -f Makefile clean

distclean:
	make -f Makefile distclean

romfs:
	$(ROMFSINST) -e CONFIG_LIB_LIBTERMCAP /lib/libtermcap.so
	$(ROMFSINST) -e CONFIG_LIB_LIBTERMCAP -s libtermcap.so \
		/lib/libtermcap.so.2
