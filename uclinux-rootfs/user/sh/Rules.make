# Hey Emacs, this Makefile is in -*- makefile -*- mode!
## $Id: Rules.elinux,v 1.57 2001/10/25 16:43:02 pkj Exp $

# Try 'make axishelp' to display the possible targets

makefrag_name = .target-makefrag

# Set VERBOSE_FRAG to nothing to print that stuff if you want it, 
# it clutters up the screen so it's @ by default.
VERBOSE_FRAG = @

# By default we do not build statically except for one target (clinux),
# but it can be enabled/disabled with the targets static and nostatic.
AXIS_STATIC =

AXIS_WARNING_PREFIX = ---------- WARNING:
AXIS_ERROR_PREFIX   = ---------- ERROR:
AXIS_USE_UCLIBC_DEPRECATED = Makefile in $(shell pwd) sets deprecated \
USE_UCLIBC.  It should set AXIS_USABLE_LIBS = UCLIBC
AXIS_LIB_MISMATCH = The code in $(shell pwd) cannot be linked with\
 $(AXIS_WILL_USE_LIB)

# Help "function" to add slashes before spaces
slashify = $(shell echo "$(1)" | sed 's/ /\\ /g')

###########################################################################
# Start by deciding what we are building for

ifndef AXIS_BUILDTYPE
# If we have a .target-makefrag, read stuff from it.
ifeq ($(makefrag_name),$(wildcard $(makefrag_name)))
include $(makefrag_name)
endif # An $(makefrag_name) existing in current directory.

# OBSOLETE: Backward compatibility with the old .target_* files.
ifndef AXIS_BUILDTYPE
ifeq (.target_elinux,$(wildcard .target_*))
$(warning $(AXIS_WARNING_PREFIX) Use of .target_elinux is deprecated, and support for it will be removed in the near future)
AXIS_BUILDTYPE = elinux
endif
ifeq (.target_clinux,$(wildcard .target_*))
$(warning $(AXIS_WARNING_PREFIX) Use of .target_clinux is deprecated, and support for it will be removed in the near future)
AXIS_BUILDTYPE = clinux
endif
ifeq (.target_host,$(wildcard .target_*))
$(warning $(AXIS_WARNING_PREFIX) Use of .target_host is deprecated, and support for it will be removed in the near future)
AXIS_BUILDTYPE = host
endif
endif

endif # !AXIS_BUILDTYPE

# Build for host if the build type is not defined.
ifndef AXIS_BUILDTYPE
AXIS_BUILDTYPE = host
endif

# Should we build with debug information or not?
ifeq ($(AXIS_DEBUG),debug)
AXIS_USE_DEBUG = yes
else
AXIS_USE_DEBUG = no
endif

#--------------------------------------------------------------------------

# Deduce other build variables from AXIS_BUILDTYPE.

AXIS_BUILDTYPE_KNOWN = no

ifeq ($(AXIS_BUILDTYPE),cris-axis-linux-gnu)
CLINUX = 1
AXIS_WILL_USE_LIB = GLIBC
AXIS_BUILDTYPE_KNOWN = yes
endif # AXIS_BUILDTYPE == cris-axis-linux-gnu

ifeq ($(AXIS_BUILDTYPE),cris-axis-linux-gnuuclibc)
CLINUX = 1
AXIS_WILL_USE_LIB = UCLIBC
AXIS_BUILDTYPE_KNOWN = yes
endif # AXIS_BUILDTYPE == cris-axis-linux-gnuuclibc

ifeq ($(AXIS_BUILDTYPE),clinux)
CLINUX = 1
AXIS_WILL_USE_LIB = UCLIBC
# Default static
AXIS_STATIC = static
AXIS_BUILDTYPE_KNOWN = yes
endif # AXIS_BUILDTYPE == clinux

ifeq ($(AXIS_BUILDTYPE),elinux)
ELINUX = 1
AXIS_WILL_USE_LIB = UCLIBC
AXIS_BUILDTYPE_KNOWN = yes
endif # AXIS_BUILDTYPE == elinux

ifeq ($(AXIS_BUILDTYPE),host)
HOST = 1
# We want to be able to use gdb with full debug info for host compiled
# programs.
ifneq ($(AXIS_DEBUG),no)
AXIS_USE_DEBUG = yes
endif
AXIS_BUILDTYPE_KNOWN = yes
endif # AXIS_BUILDTYPE == host

###########################################################################

ifdef ELINUX
# This is for gcc-cris & Co. of R25 or higher.  Get it; don't
# use older versions or R25 without glibc installed.

# Without "-melinux", the compiler will not get the right
# include-files and have the right defines etc. so it should be
# seen as part of CC rather than CFLAGS.
CC_FLAGS    = -melinux

TARGET_TYPE = elinux
CRIS        = 1

# This is for compatibility with cris-dist-1.7, where a.out was the
# default, but which does not understand -mcrisaout.  We'll only do the
# costly assignment if we don't inherit the value, which is a " " or
# -mcrisaout.
ifndef AXIS_LD_OUTPUT_FORMAT_OPTION
AXIS_LD_OUTPUT_FORMAT_OPTION := $(shell `gcc-cris -print-prog-name=ld` -V | sed -n -e 's/.* crisaout/-mcrisaout/p') #
endif

# ELINUX always uses uclibc.  Though a check that the app can use uclibc
# is in order.
ifndef USE_UCLIBC

ifdef AXIS_USABLE_LIBS
ifneq ($(filter $(AXIS_USABLE_LIBS),UCLIBC),UCLIBC)
$(error $(AXIS_ERROR_PREFIX) $(AXIS_LIB_MISMATCH))
else
# OBSOLETE: Define legacy macro.
USE_UCLIBC = 1
endif # UCLIBC not in AXIS_USABLE_LIBS
endif # AXIS_USABLE_LIBS

else
# OBSOLETE: If you get an error about missing separator here
# upgrade your make!
$(warning $(AXIS_WARNING_PREFIX) $(AXIS_USE_UCLIBC_DEPRECATED))
endif # !USE_UCLIBC
endif # ELINUX

#--------------------------------------------------------------------------

ifdef CLINUX

ifndef ELINUXDIR
ELINUXDIR := $(shell echo `pwd`/../os/linux)
endif

# Here we will check AXIS_WILL_USE_LIB against AXIS_USABLE_LIBS which
# represents what the thing we want to compile can be linked with.  If
# library and compile variable settings (like CC) aren't supposed to be
# set or are not applicable, then AXIS_USABLE_LIBS should not be set.

# OBSOLETE: First some backwards compatibility: each app used to set
# USE_UCLIBC in its makefile.
ifdef USE_UCLIBC
$(warning $(AXIS_WARNING_PREFIX) $(AXIS_USE_UCLIBC_DEPRECATED))
AXIS_USABLE_LIBS += UCLIBC
# Let's unset the original var to fall in line with expected case.
USE_UCLIBC =
endif

ifdef AXIS_USABLE_LIBS
ifneq ($(filter $(AXIS_USABLE_LIBS),$(AXIS_WILL_USE_LIB)),$(AXIS_WILL_USE_LIB))
$(error $(AXIS_ERROR_PREFIX) $(AXIS_LIB_MISMATCH))
endif
endif # AXIS_USABLE_LIBS

# OBSOLETE: Set legacy variables used in various unknown Makefiles.
# FIXME: Kill the USE_* variables when all apps are changed to use
# AXIS_WILL_USE_LIB instead.  Until then, introduce no new use or setting
# of USE_*.
ifeq ($(AXIS_WILL_USE_LIB),GLIBC)
USE_GLIBC = 1
else # AXIS_WILL_USE_LIB != GLIBC (i.e. currently implying UCLIBC).
ifeq ($(AXIS_WILL_USE_LIB),UCLIBC)
USE_UCLIBC = 1
else
$(error $(AXIS_ERROR_PREFIX) Unexpected library $(AXIS_WILL_USE_LIB).)
endif
endif # AXIS_WILL_USE_LIB != GLIBC

# Now that we've checked that AXIS_WILL_USE_LIB matches AXIS_USABLE_LIBS,
# we set the other build vars.

CC_FLAGS  = -mlinux

ifeq ($(AXIS_WILL_USE_LIB),UCLIBC)
# Same as ELINUX but with different options (to build as ELF).
CC_FLAGS += -DCRISMMU
else
ifeq ($(AXIS_WILL_USE_LIB),GLIBC)
# Using glibc, linking dynamically like on the host.

# We prefer getting <asm/...> and <linux/...> from the kernel we compile,
# rather than what was installed with the compiler, so put it first in the
# system include path.  Though we have to go through a separate directory
# with just the linux and asm dirs, rather than use $(ELINUXDIR)/include,
# lest we also drag in the other subdirs, which collide with the normal
# gcc-cris -mlinux system-includes, for example <net/...>.

CC_FLAGS += -isystem $(prefix)/include
else
$(error $(AXIS_ERROR_PREFIX) Unexpected library $(AXIS_WILL_USE_LIB).)
endif # AXIS_WILL_USE_LIB = GLIBC
endif # AXIS_WILL_USE_LIB = UCLIBC

TARGET_TYPE = clinux
CRIS        = 1

AXIS_LD_OUTPUT_FORMAT_OPTION = -mcrislinux

endif # CLINUX

#--------------------------------------------------------------------------

ifdef CRIS

ifeq ($(AXIS_WILL_USE_LIB),UCLIBC)
# OBSOLETE: Delete the UCLIBC macro once all usage is exterminated.  Usage
# should be of if[n]eq ($(AXIS_WILL_USE_LIB),UCLIBC).
UCLIBC    = 1

# We use a wrapper (gcc_cris) when compiling with uC-libc
CC        = gcc_cris $(CC_FLAGS) -muclibc=$(prefix)
else
CC        = gcc-cris $(CC_FLAGS)
endif

CPP       = $(CC) -E
CXX       = g++-cris $(CC_FLAGS) -xc++

# This is for building for the CRIS architecture.

OBJCOPY   = objcopy-cris
LD        = ld-cris $(AXIS_LD_OUTPUT_FORMAT_OPTION)
AR        = ar-cris
RANLIB    = ranlib-cris

run_prefix = 

else

# This is for building for the local host.

RANLIB    = ranlib

run_prefix = $(prefix)

endif # !CRIS

#--------------------------------------------------------------------------

ifndef prefix
ifdef AXIS_TOP_DIR
prefix    = $(AXIS_TOP_DIR)/target/$(AXIS_BUILDTYPE)
else
# OBSOLETE: The $(EROOT) and $(HROOT) variables should not be used any more.
ifdef CRIS
ifdef EROOT
prefix = $(EROOT)
$(warning $(AXIS_WARNING_PREFIX) Use of $$EROOT is deprecated! Please use $$AXIS_TOP_DIR instead)
endif
else
ifdef HROOT
prefix = $(HROOT)
$(warning $(AXIS_WARNING_PREFIX) Use of $$HROOT is deprecated! Please use $$AXIS_TOP_DIR instead)
endif 
endif # CRIS
endif # AXIS_TOP_DIR
endif # !prefix

export prefix

#--------------------------------------------------------------------------

# Define CFLAGS to something sensible.
CFLAGS    = -Wall

ifeq ($(AXIS_USE_DEBUG),yes)
# Add suitable compiler flags for debugging.
CFLAGS   += -O0 -g -fno-omit-frame-pointer
else
CFLAGS   += -O2
endif

# OBSOLETE: Add ELINUX define.
ifdef ELINUX
CFLAGS   += -DELINUX
endif

# Can't use -pedantic due to use of long long in standard includes. :(
CXXFLAGS  = $(CFLAGS) -Wno-ctor-dtor-privacy -ansi -pipe

# Define LDFLAGS to something sensible.
LDFLAGS   =

ifeq ($(AXIS_WILL_USE_LIB),GLIBC)
LDFLAGS  += -L$(prefix)/lib
endif

# Add -s flag for stripping if not building for debug. (The -s flag must not
# be specified when building statically for elinux, however.)
ifneq ($(AXIS_USE_DEBUG),yes)
ifneq ($(AXIS_STATIC),static)
LDFLAGS  += -s
else
ifndef ELINUX
LDFLAGS  += -s
endif
endif
endif # AXIS_USE_DEBUG != yes

ifeq ($(AXIS_STATIC),static)
LDFLAGS  += -static
else
ifdef ELINUX
LDFLAGS  += -shlib
endif
endif # AXIS_STATIC = static

INSTALL   = install_elinux

###########################################################################
# The following is a set of standard rules to try to make sure we build
# and install the correct files.

# Make verify_builddir a dependency of all build and install rules to make
# sure the build directory is set correctly before trying to actually build
# anything.

all:	 	all-recurse
all-recurse:	pre-all-recurse
pre-all-recurse:	verify_builddir

depend:			depend-recurse
depend-recurse:		pre-depend-recurse
pre-depend-recurse:	verify_builddir

install:		install-recurse
install-recurse:	pre-install-recurse 
pre-install-recurse:	verify_builddir

uninstall:		uninstall-recurse
uninstall-recurse:	pre-uninstall-recurse
pre-uninstall-recurse:	verify_builddir

clean:		clean-recurse
clean-recurse:	pre-clean-recurse

ifndef ROOT_MAKEFRAG
ROOT_MAKEFRAG := $(shell echo `pwd`/$(makefrag_name))

# OBSOLETE: This rule allows old .target_* files to be converted to
# .target-makefrag by running make configsubs in the root directory
$(call slashify,$(ROOT_MAKEFRAG)):
ifdef AXIS_BUILDTYPE
	$(MAKE) $(AXIS_BUILDTYPE)
else
	@echo "You must specify the target type!"
	exit 1
endif
endif # !ROOT_MAKEFRAG

## 
## The following special targets exists:

## axishelp                   - This help
axishelp:
	@grep '^## ' $(APPS)/Rules.elinux

## 
## The following build types exist:
## cris-axis-linux-gnu        - CRIS/Linux 2.4 with shared glibc
## cris-axis-linux-gnuuclibc  - CRIS/Linux 2.4 with shared uClibc
## clinux                     - CRIS/Linux 2.4 with static uClibc
## elinux                     - CRIS/Linux 2.0 NO_MMU with uC-libc
## host                       - Host build with debug
cris-axis-linux-gnu cris-axis-linux-gnuuclibc clinux elinux host:
	$(VERBOSE_FRAG)echo AXIS_BUILDTYPE=$@ > .tmp$(makefrag_name)
ifeq ($(AXIS_DEBUG),debug)
	$(VERBOSE_FRAG)echo AXIS_DEBUG=$(AXIS_DEBUG) >> .tmp$(makefrag_name)
endif
	@$(MAKE) configsubs AXIS_BUILDTYPE=$@ ROOT_MAKEFRAG="$(shell echo `pwd`/.tmp$(makefrag_name))" || ( rm -f .tmp$(makefrag_name); exit 1 )
	@rm -f .tmp$(makefrag_name)

## 
## The following build options exist:
## debug                      - Enable debug
## nodebug                    - Do not enable debug
## static                     - Enable static linking
## nostatic                   - Do not enable static linking
debug nodebug static nostatic:
	@option=`echo $@ | sed 's/^no//'`;\
	target=`echo AXIS_$$option | tr a-z A-Z`;\
	grep -v "$$target" $(makefrag_name) > .tmp$(makefrag_name) 2> /dev/null; \
	if test "$$option" = "$@"; then \
	  echo $$target=$@ >> .tmp$(makefrag_name); \
	  neg_check='!'; \
	fi; \
	$(MAKE) configsubs AXIS_CHECK_STRING="$$target=$$option" AXIS_NEG_CHECK=$$neg_check ROOT_MAKEFRAG="$(shell echo `pwd`/.tmp$(makefrag_name))" || ( rm -f .tmp$(makefrag_name); exit 1 )
	@rm -f .tmp$(makefrag_name)

ifndef AXIS_CHECK_STRING
AXIS_CHECK_STRING = "AXIS_BUILDTYPE=$(AXIS_BUILDTYPE)"
AXIS_NEG_CHECK = !
endif

checkclean:
	@if ! test -f .target_$(AXIS_BUILDTYPE) \
	    && ( ! test -f $(makefrag_name) \
	         || $(AXIS_NEG_CHECK) grep $(AXIS_CHECK_STRING) $(makefrag_name) > /dev/null ); then \
	  NO_SUBDIR_RECURSION=1 $(MAKE) --no-print-directory clean ROOT_MAKEFRAG="$(ROOT_MAKEFRAG)"; \
	fi

configsubs: 		configsubs-recurse
configsubs-recurse:	pre-configsubs-recurse
pre-configsubs-recurse:	$(call slashify,$(ROOT_MAKEFRAG))

configsubs: checkclean
	-@if ! cmp -s "$(ROOT_MAKEFRAG)" $(makefrag_name); then \
	  rm -f .target*; \
	  echo "cp \"$(ROOT_MAKEFRAG)\" $(makefrag_name)"; \
	  cp "$(ROOT_MAKEFRAG)" $(makefrag_name); \
	fi

# These are hooks that can be used to have rules executed before the
# recursive rules are checked.
pre-all-recurse pre-depend-recurse pre-install-recurse pre-uninstall-recurse pre-clean-recurse pre-configsubs-recurse:

# Recursive rules to make all, depend, install, uninstall, clean and configsubs
all-recurse depend-recurse install-recurse uninstall-recurse clean-recurse configsubs-recurse:
ifneq ($(NO_SUBDIR_RECURSION),1)
	@subdirs="$(SUBDIRS)"; \
	for subdir in $$subdirs; do \
	  if test -d "$$subdir"; then \
	    target=`echo $@ | sed 's/-recurse//'`; \
	    echo "Making $$target in $$subdir for $(AXIS_BUILDTYPE)"; \
	    $(MAKE) -C "$$subdir" $$target ROOT_MAKEFRAG="$(ROOT_MAKEFRAG)" || exit 1; \
	  else \
	    echo "The directory '$$subdir' does not exist or is not a directory!"; \
	    exit 1; \
	  fi; \
	done
endif

verify_builddir:	verify_buildtype
ifndef prefix
	@echo "You must define either the environment variable \$$AXIS_TOP_DIR to where your"
	@echo "source tree is located, or \$$prefix to where anything (independent of target"
	@echo "type) should be installed!"
	@exit 1
endif

verify_buildtype:
ifneq ($(AXIS_BUILDTYPE_KNOWN),yes)
	@echo "Unknown build type '$(AXIS_BUILDTYPE)'!"
	@echo "Use 'make axishelp' to get a list of valid build types."
	@exit 1
endif
