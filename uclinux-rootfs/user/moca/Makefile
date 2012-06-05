#
# Makefile for binary-only MoCA 1.1 support
#

include $(ROOTDIR)/config.arch

.PHONY: all clean distclean
all clean distclean:

.PHONY: romfs
romfs:
	mkdir -p $(ROMFSDIR)/etc/moca
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA1 \
		$(MACHINE)/mocad /bin/mocad
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA1 \
		$(MACHINE)/mocactl /bin/mocactl
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA1 \
		$(MACHINE)/soapserver /bin/soapserver
	$(ROMFSINST) -e CONFIG_USER_MOCA_GEN1 \
		fw/mocacore-gen1.bin /etc/moca/mocacore-gen1.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_GEN2 \
		fw/mocacore-gen2.bin /etc/moca/mocacore-gen2.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_GEN3 \
		fw/mocacore-gen3.bin /etc/moca/mocacore-gen3.bin
