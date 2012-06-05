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
	$(ROMFSINST) -e CONFIG_USER_MOCA_GEN1 \
		fw/moca20core-gen1.bin /etc/moca/moca20core-gen1.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_GEN2 \
		fw/moca20core-gen2.bin /etc/moca/moca20core-gen2.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_GEN3 \
		fw/moca20core-gen3.bin /etc/moca/moca20core-gen3.bin
