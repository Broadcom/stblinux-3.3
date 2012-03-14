PASSTHRU		:= clean distclean

ifdef CONFIG_USER_GDB

all:
	make -f Makefile

else
sinclude $(ROOTDIR)/config.arch

TOOL_DIR_PREFIX=$(shell dirname $(shell which $(CC)))/..
UCLIBC_INC = ../../uClibc/include
UCLIBC_LIB = ../../uClibc/lib
ifneq (,$(findstring linux,$(CROSS)))
	TARGET=$(CROSS:linux-=linux)
else
	ifneq (,$(findstring uclibc,$(CROSS)))
		TARGET_ARCH ?= $(CROSS:-uclibc-=)
		TARGET=$(TARGET_ARCH)-linux
	else
#		echo "FIXME: Cannot determine target architecture"
#		exit 99
	endif
endif


all:
	if [ ! -e Makefile ]; then \
	( \
	AR=$(CROSS)ar						\
	AS=$(CROSS)as						\
	LD=$(CROSS)ld						\
	NM=$(CROSS)nm						\
	CC=$(CROSS)gcc						\
	GCC=$(CROSS)gcc						\
	CXX=$(CROSS)g++						\
	RANLIB=$(CROSS)-ranlib					\
	gdb_cv_func_sigsetjmp=yes					\
	ac_cv_type_uintptr_t=yes					\
	gt_cv_func_gettext_libintl=yes				\
	ac_cv_func_dcgettext=yes					\
	bash_cv_func_strcoll_broken=no				\
	bash_cv_must_reinstall_sighandlers=no				\
	bash_cv_func_sigsetjmp=present				\
	bash_cv_have_mbstate_t=yes					\
	srv_linux_thread_db=yes					\
	export STD_INC="$(TOOL_DIR_PREFIX)/$(TARGET)/include"; \
	export GCC_INC="$(TOOL_DIR_PREFIX)/$(TARGET)/include"; \
	test -f config.cache && rm -f config.cache; \
	./configure			\
		--prefix=/	\
		--target=$(TARGET)						\
		--build=i686-pc-linux-gnu						\
		--host=$(TARGET)						\
		--disable-nls						\
		--without-uiout --disable-gdbmi				\
		--disable-tui --disable-gdbtk --without-x			\
		--without-included-gettext	\
	) ; \
	fi
	make -f Makefile  CFLAGS+=-L../../../../romfs/lib/

#distclean: clean
#	-rm config.cache
#	make -f Makefile distclean
#	rm Makefile

endif

.PHONY: $(PASSTHRU)
$(PASSTHRU):
	make -f Makefile $@

romfs:
	$(ROMFSINST) -e CONFIG_USER_GDBSERVER_GDBSERVER /bin/gdbserver
	$(ROMFSINST) -e CONFIG_USER_GDBSERVER_GDBREPLAY /bin/gdbreplay
