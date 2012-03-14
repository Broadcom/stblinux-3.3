#
# Makefile for xfsprogs/libxfs.a
#
# 09/05/03 Ton Truong
#

LIBDISK = libdisk.so

CFLAGS += -I../include -I. -I.. \
		-I$(ROOTDIR)/romfs/usr/include \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" \
		-fPIC

CFILES = drivers.c fstype.c pttype.c
HFILES = drivers.h fstype.h pttype.h md.h xvm.h evms.h
LINUX_DRIVERS = md.c xvm.c evms.c lvm.c

CFILES += $(LINUX_DRIVERS)

OBJS   =  $(CFILES:.c=.o)

LDSHARED=$(CC) --shared -Wl,-soname,$(LIBXFS)

all: $(ROOTDIR)/romfs/lib/$(LIBDISK)

$(LIBDISK) : $(OBJS)
	$(RM) -f $@
	$(LDSHARED) -o $@ $(OBJS)
	
$(ROOTDIR)/romfs/lib/$(LIBDISK): $(LIBDISK)
	cp -f $(LIBDISK) $@
	$(CROSS)strip $@

romfs:

clean:
	-$(RM) -f $(OBJS) $(LIBDISK)
