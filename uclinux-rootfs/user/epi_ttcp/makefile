.EXPORT_ALL_VARIABLES:
include $(ROOTDIR)/config.arch
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
UCLIBC_INC = -I$(ROOTDIR)/uClibc/include
LINUX_INC=-I$(ROOTDIR)/$(LINUXDIR)/include
# THT: Order is important, as we want to use the uClibc headers first.
MY_CFLAGS += $(UCLIBC_INC) $(LINUX_INC)

all: epi_ttcp.c
	$(CC) $(MY_CFLAGS) $(CFLAGS) epi_ttcp.c -o epi_ttcp

clean:
	rm -rf epi_ttcp

distclean: clean

romfs:
	$(ROMFSINST) -e CONFIG_USER_EPI_TTCP /usr/bin/epi_ttcp
