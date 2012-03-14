#
# STB Linux build system v2.2
# Copyright (C) 2011 Broadcom Corporation
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

SAVEVARS	:= PATH SHELL HOME USER MFLAGS MAKEFLAGS MAKELEVEL \
		MAKEOVERRIDES TERM DISPLAY
unexport $(shell perl -w bin/cleanenv.pl $(SAVEVARS))

# don't pass command line options like SUBDIRS
MAKEOVERRIDES	:=

ROOTDIR		:= $(shell pwd)
LINUXDIR	:= linux
LINUX_CONFIG	:= $(ROOTDIR)/$(LINUXDIR)/.config
CONFIG_CONFIG	:= $(ROOTDIR)/config/.config
ARCH_CONFIG	:= $(ROOTDIR)/config.arch
ROMFSDIR	:= $(ROOTDIR)/romfs
ROMFSINST	:= $(ROOTDIR)/bin/romfs-inst.sh
LIBCDIR		:= uClibc

CFG		:= perl -w bin/config.pl
CHKPATH		:= perl -w bin/checkpath.pl

SHELL		:= /bin/bash
PATH		:= $(shell $(CHKPATH) -a)
export ROOTDIR LINUXDIR LINUX_CONFIG CONFIG_CONFIG ARCH_CONFIG ROMFSDIR \
	ROMFSINST LIBCDIR SHELL PATH

# hostname of TFTP server (defaults to the build machine)
TFTPHOST	:= $(shell hostname)

# root directory of TFTP server
TFTPBOOT	:= /tftpboot

# full (local) TFTP path, where "make install" will copy the images.
# Typical usage: user overrides TFTPDIR on the command line, and
# this Makefile will derive the correct TFTPPATH from TFTPDIR/TFTPBOOT
TFTPDIR		:= $(TFTPBOOT)/$$USER

# relative (remote) TFTP path under $(TFTPBOOT), for use by the TFTP client
TFTPPATH	:= $(subst $(TFTPBOOT)/,,$(TFTPDIR))

PLAT		= $(shell cat .target)
ARCH		= $(shell cat .arch)

VERSION		:= $(shell cat version)
DOTVERSION	:= $(patsubst 26%,2.6.%,$(patsubst 3%,3.%,$(VERSION)))

REL_TFTPHOST	:= stb-irva-01
REL_TFTPDIR	:= $(TFTPBOOT)/$(VERSION)
RELOPTS		:= TFTPHOST=$(REL_TFTPHOST) TFTPDIR=$(REL_TFTPDIR)

MAKEOPTS	:= TFTPHOST=$(TFTPHOST) TFTPDIR=$(TFTPDIR) TFTPPATH=$(TFTPPATH)

#############################################################################
# BASIC KERNEL/ROOTFS BUILDS
#############################################################################

.PHONY: images
images: .target
	$(MAKE) $(MAKEOPTS) initrd_kernel
	$(MAKE) $(MAKEOPTS) kernel
	bin/build_rootfs_images.sh
	@echo ""
	@echo "Build complete.  Images are in images/"
	@echo ""
	@echo "To install under $(TFTPDIR), use \"make install\""
	@echo ""

.PHONY: initrd_kernel
initrd_kernel: .target chkpath
	mkdir -p images
	# prepare rootfs skeleton
	if [ -z "$(SUBDIRS)" ]; then \
		rm -rf $(ROMFSDIR) ; \
		cp -a skel romfs || exit 1 ; \
		chmod -R a+x romfs/{bin,sbin} ; \
		mkdir -p romfs/{bin,sbin,etc,dev,proc,sys,lib,root,tmp} ; \
		mkdir -p romfs/lib/modules ; \
		mkdir -p romfs/mnt/{hd,usb,flash,nfs} ; \
		mkdir -p romfs/var/{log,run,tmp} romfs/{opt,data} ; \
		ln -s bin/busybox romfs/init ; \
		ln -s /tmp/resolv.conf romfs/etc/resolv.conf ; \
		ln -s /proc/mounts romfs/etc/mtab ; \
		rm -f romfs/etc/brcmstb.conf ; \
		echo "TFTPHOST=$(TFTPHOST)" > romfs/etc/brcmstb.conf ; \
		echo "TFTPPATH=$(TFTPPATH)" >> romfs/etc/brcmstb.conf ; \
		echo "PLAT=$(PLAT)" >> romfs/etc/brcmstb.conf ; \
		echo "VERSION=$(VERSION)" >> romfs/etc/brcmstb.conf ; \
	fi
	# build/install rootfs contents
	if [[ "$(SUBDIRS)" != linux* ]]; then \
		$(MAKE) -C lib all \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		$(MAKE) -C user all \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		$(MAKE) -j1 -C lib romfs \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		$(MAKE) -j1 -C user romfs \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
	fi
	# configure kernel for initramfs, then build it
	$(CFG) initramfs
	if [ "$(ARCH)" = "mips" ]; then \
		$(MAKE) -C $(LINUXDIR) SUBDIRS= ARCH=mips vmlinux || exit 1; \
		gzip -9 < $(LINUXDIR)/vmlinux > \
			images/vmlinuz-initrd-$(PLAT) || exit 1; \
	else \
		$(MAKE) -C $(LINUXDIR) SUBDIRS= ARCH=arm zImage || exit 1; \
		cp -f $(LINUXDIR)/arch/arm/boot/zImage \
			images/vmlinuz-initrd-$(PLAT) || exit 1; \
	fi

.PHONY: kernel
kernel: .target chkpath
	mkdir -p images
	$(CFG) noinitramfs
	if [ "$(ARCH)" = "mips" ]; then \
		$(MAKE) -C $(LINUXDIR) ARCH=mips vmlinux || exit 1; \
		gzip -9 < $(LINUXDIR)/vmlinux > \
			images/vmlinuz-$(PLAT) || exit 1; \
	else \
		$(MAKE) -C $(LINUXDIR) SUBDIRS= ARCH=arm zImage || exit 1; \
		cp -f $(LINUXDIR)/arch/arm/boot/zImage \
			images/vmlinuz-$(PLAT) || exit 1; \
	fi

.PHONY: kernel_sign
kernel_sign: .target chkpath
	@if [ -z "$(SIGLIST)" ]; then \
		echo "!!!No signature list file is specified!!!"; exit 1;\
	fi; \
	#Generate initram fs skeleton
	rm -rf $(ROOTDIR)/initramfs; \
	mkdir -p initramfs/{bin,dev,etc,proc,root,sbin,tmp};\
	chmod -R a+x initramfs/{bin,sbin}
	#build rootfs signing app.
	$(MAKE) -j1 -C user all \
		BUILD_ROOT_SIGN=y SIGLIST=$(SIGLIST) ROOTFSDIR=$(ROOTFSDIR)
	$(MAKE) -j1 -C user romfs \
		BUILD_ROOT_SIGN=y SIGLIST=$(SIGLIST) ROOTFSDIR=$(ROOTFSDIR)
	#build kernel.
	$(CFG) initramfs-sign 
	if [ "$(ARCH)" = "mips" ]; then \
		$(MAKE) -C $(LINUXDIR) vmlinux SUBDIRS= ARCH=mips || exit 1; \
		gzip -9 < $(LINUXDIR)/vmlinux > images/vmlinuz-sign-$(PLAT); \
	else \
		echo "ERROR: this target does not support ARM"; \
		exit 1; \
	fi

#############################################################################
# PRIMARY BUILD TARGETS
#############################################################################

.PHONY: vmlinuz-initrd-%
vmlinuz-initrd-%:
	$(MAKE) $(MAKEOPTS) defaults-$*
	$(MAKE) $(MAKEOPTS) initrd_kernel

.PHONY: vmlinuz-%
vmlinuz-%:
	$(MAKE) $(MAKEOPTS) defaults-$*
	$(MAKE) $(MAKEOPTS) kernel

.PHONY: kernels-%
kernels-%:
	$(MAKE) $(MAKEOPTS) vmlinuz-initrd-$*
	$(MAKE) $(MAKEOPTS) vmlinuz-$*

.PHONY: images-%
images-%:
	$(MAKE) $(MAKEOPTS) defaults-$*
	$(MAKE) $(MAKEOPTS) images

.PHONY: release
release:
	# make sure this is a clean view
	@if [ -e .dirty -o -e images -o -e release ]; then \
		echo "ERROR: \"make release\" must be run on a pristine view"; \
		exit 1 ; \
	fi
	# fix perms and create source release
	../tools/set_extraversion.pl --drop-pre
	chmod -R u+w . $(LINUXDIR)/
	chmod -R 0755 bin/
	cd .. && tools/makedelta.pl
	export LINK=`readlink $(LINUXDIR)`; \
		export STBLINUX=`basename $$LINK`; \
		tar -C .. -jcf ../stblinux-$(DOTVERSION).tar.bz2 $$STBLINUX
	tar -C .. -jcf ../uclinux-rootfs-$(DOTVERSION).tar.bz2 uclinux-rootfs
	# build all platforms
	for x in $(shell cat misc/release_builds); do \
		$(MAKE) $(RELOPTS) images-$$x || exit 1; \
	done
	# move files into the release directory
	mkdir -p release/src
	mv ../stblinux-$(DOTVERSION).tar.bz2 \
		../uclinux-rootfs-$(DOTVERSION).tar.bz2 \
		../delta-*.patch* \
		release/src/
	mkdir release/$(VERSION)
	mv images/* release/$(VERSION)/

#############################################################################
# MISC CHECKS
#############################################################################

.target:
	@echo "Configuration defaults not set.  Try: \"make help\""
	@exit 1

.PHONY: chkpath
chkpath:
	@$(CHKPATH)

#############################################################################
# BUILD CONFIGURATION
#############################################################################

.PHONY: silentoldconfig
silentoldconfig: .target chkpath
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) silentoldconfig
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) prepare scripts
	$(MAKE) -C lib/uClibc oldconfig
	$(MAKE) -C user/busybox silentoldconfig
	$(MAKE) -C config oldconfig

.PHONY: oldconfig
oldconfig: .target chkpath
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) oldconfig
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) prepare scripts
	$(MAKE) -C lib/uClibc oldconfig
	$(MAKE) -C user/busybox oldconfig
	$(MAKE) -C config oldconfig

.PHONY: defaults-%
defaults-%: chkpath
	@touch .dirty
	@chmod -R 0755 config/scripts/{Configure,Menuconfig} bin
	$(CFG) defaults $*
	$(MAKE) -j1 silentoldconfig
	@echo "***************************************************************"
	@echo "* Build configuration"
	@echo "*"
	@echo "* Target:                  $*"
	@echo "* Version:                 $(DOTVERSION)"
	@echo "* TFTP host:               $(TFTPHOST)"
	@echo "* Remote TFTP path:        $(TFTPPATH)"
	@echo "* Local TFTP path:         $(TFTPDIR)"
	@echo "***************************************************************"
	@echo ""

.PHONY: save_defaults
save_defaults: .target
	$(CFG) save_defaults $(PLAT)

.PHONY: menuconfig-linux menuconfig-vendor menuconfig-uclibc \
	menuconfig-busybox xconfig-linux
menuconfig-linux: .target
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) menuconfig

menuconfig-vendor: .target
	$(MAKE) -C config menuconfig

menuconfig-uclibc: .target
	$(MAKE) -C lib/uClibc menuconfig

menuconfig-busybox: .target
	$(MAKE) -C user/busybox menuconfig

xconfig-linux: .target
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) xconfig

#############################################################################
# INSTALL/CLEAN
#############################################################################

.PHONY: install
install:
	mkdir -p $(TFTPDIR)
	cp -f -p images/* $(TFTPDIR)/
	@echo ""
	@echo "Images have been copied to $(TFTPDIR)"
	@echo ""

.PHONY: clean
clean:
	if [ -e .target ]; then \
		if [ -z "$(SUBDIRS)" ]; then \
			$(MAKE) -C lib clean ; \
			$(MAKE) -C user clean ; \
			$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) clean ; \
		elif [[ "$(SUBDIRS)" != "linux*" ]]; then \
			$(MAKE) -C lib clean \
				INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
			$(MAKE) -C user clean \
				INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		elif [[ "$(SUBDIRS)" = "linux*" ]]; then \
			$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) clean ; \
		fi ; \
		if [ -d user/root_sign ]; then \
			$(MAKE) -C user/root_sign clean; \
		fi ;\
	fi

.PHONY: distclean
distclean:
	if [ -e .target ]; then \
		$(MAKE) -C lib distclean clean ; \
		$(MAKE) -C user distclean clean ; \
		$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) mrproper ; \
		$(MAKE) -C config clean ; \
	fi
	rm -rf romfs tmp
	rm -f $(CONFIG_CONFIG) $(ARCH_CONFIG) .target .arch

#############################################################################
# INFORMATIONAL TARGETS
#############################################################################

.PHONY: help
help:
	@echo ""
	@echo "STB Linux build targets:"
	@echo ""
	@echo "vmlinuz-<target>           - Build non-initramfs kernel for TARGET"
	@echo "vmlinuz-initrd-<target>    - Build initramfs kernel for TARGET"
	@echo "kernels-<target>           - Build both kernels for TARGET"
	@echo "images-<target>            - Build kernels + flash images for TARGET"
	@echo ""
	@echo "clean                      - Clean objects but not configurations"
	@echo "distclean                  - Clean everything"
	@echo ""
	@echo "defaults-<target>          - Install default config files for TARGET"
	@echo "menuconfig-linux           - Customize Linux config"
	@echo "menuconfig-vendor          - Customize user/lib package config"
	@echo "menuconfig-uclibc          - Customize uClibc config"
	@echo "menuconfig-busybox         - Customize busybox config"
	@echo "save_defaults              - Overwrite default config with current settings"
	@echo ""
	@echo "images                     - Build initramfs kernel for current target,"
	@echo "                             then build flash images / tarballs"
	@echo "initrd_kernel              - Build initramfs kernel for current target"
	@echo "kernel                     - Build non-initramfs kernel for current target"
	@echo ""
	@echo "install                    - Copy all kernels/images/tarballs to TFTPDIR"
	@echo ""
	@echo "release                    - Create source tarballs, then build all targets"
	@echo ""
	@echo "Examples:"
	@echo ""
	@echo "  make images-7335b0       - Build all images for 7335b0"
	@echo "  make vmlinuz-7335b0      - Build non-initrd kernel for 7335b0"
	@echo ""
