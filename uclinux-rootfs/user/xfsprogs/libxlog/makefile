#
# Makefile for xfsprogs/libxfs.a
#
# 09/05/03 Ton Truong
#

LIBXLOG = libxlog.so

LDSHARED=$(CC) --shared -Wl,-soname,$(LIBXLOG)

CFLAGS += -I../include -I.. -I. \
		-I$(ROOTDIR)/romfs/usr/include \
		-D _LARGEFILE_SOURCE -D _LARGEFILE64_SOURCE \
		-D __USE_LARGEFILE64 -D __USE_FILE_OFFSET64 \
		-DHAVE_BUILDDEFS -DHAVE_32BIT_LONG -DHAVE_32BIT_PTR \
		-DVERSION="\"2.5.6\"" \
		-fPIC

CFILES = xfs_log_recover.c util.c

OBJS   =  $(CFILES:.c=.o)

all: $(ROOTDIR)/romfs/lib/$(LIBXLOG)


$(LIBXLOG) : $(OBJS)
	$(RM) -f $@
	$(LDSHARED) -o $@ $(OBJS)
	
$(ROOTDIR)/romfs/lib/$(LIBXLOG): $(LIBXLOG)
	cp -f $(LIBXLOG) $@
	$(CROSS)strip $@

romfs:

clean:
	rm -f $(OBJS) $(LIBXLOG)
