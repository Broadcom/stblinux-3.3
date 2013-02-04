#
# Makefile for binary-only MoCA 2.0 support
#

include $(ROOTDIR)/config.arch

.PHONY: all clean distclean
all clean distclean:

.PHONY: romfs
romfs:
	mkdir -p $(ROMFSDIR)/etc/moca
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		$(MACHINE)/mocad /bin/mocad
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		$(MACHINE)/mocap /bin/mocap
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		fw/moca20core-gen21.bin /etc/moca/moca20core-gen21.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		fw/moca20core-gen22.bin /etc/moca/moca20core-gen22.bin
