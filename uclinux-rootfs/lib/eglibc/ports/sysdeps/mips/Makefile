ifeq ($(subdir),misc)
sysdep_headers += regdef.h fpregdef.h sys/regdef.h sys/fpregdef.h \
		  sys/asm.h sgidefs.h
endif

ifeq ($(subdir),setjmp)
sysdep_routines += setjmp_aux
endif

ifeq ($(subdir),rt)
librt-sysdep_routines += rt-sysdep
endif

ifeq ($(subdir),debug)
CFLAGS-backtrace.c += -funwind-tables
endif

ifeq ($(subdir),csu)
# get offset to rtld_global._dl_platform
gen-as-const-headers += rtld-global-offsets.sym
CPPFLAGS-crti.S += $(pic-ccflag)
CPPFLAGS-crtn.S += $(pic-ccflag)
endif

ifeq ($(subdir),nptl)
CPPFLAGS-pt-crti.S += $(pic-ccflag)
CPPFLAGS-crtn.S += $(pic-ccflag)
endif

ifeq ($(subdir),string)
sysdep_routines += memcpy-bmips
endif

ASFLAGS-.os += $(pic-ccflag)

CPPFLAGS-memcpy.S += -DMEMCPY_NAME=__glibc_memcpy
