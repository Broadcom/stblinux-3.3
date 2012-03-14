include $(ROOTDIR)/$(LINUXDIR)/.config

ifeq ($(CONFIG_CPU_LITTLE_ENDIAN),y)
	TARGET_ENDIAN = __MIPSEL__
else
	TARGET_ENDIAN = __MIPSEB__
endif

TARGETS = mkcramfs cramfsck
#BUILD_TARGETS = $(patsubst %,build/%,$(TARGETS))
BUILD_TARGETS = build/mkcramfs

MY_CFLAGS = -I. -I$(ROOTDIR)/$(LINUXDIR)/include -D $(TARGET_ENDIAN)

BUILD_CFLAGS += $(MY_CFLAGS)
BUILD_CC = gcc
#CFLAGS += -I. -I$(ROOTDIR)/$(LINUXDIR)/include $(INCZ) -D__USE_BSD

CFLAGS += -I. $(INCZ) -D $(TARGET_ENDIAN)
LDPATH += -L$(ROOTDIR)/lib/zlib

SYMLINKS = $(ZINCS) # $(FSINCS)
ZINCS = zlib.h zconf.h
FSINCS = nodelist.h
OBJECTS = mkcramfs.o cramfsck.o
BUILD_OBJECTS = $(patsubst %,build/%,$(OBJECTS))

all: $(SYMLINKS) $(TARGETS) build $(BUILD_TARGETS)

.PHONY:
build:
	[ -d build ] || mkdir build
	
$(ZINCS):
	ln -sf $(ROOTDIR)/lib/zlib/$@ $@

mkcramfs: mkcramfs.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ mkcramfs.o $(LDPATH) -lz $(LDLIBS)

cramfsck: cramfsck.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ cramfsck.o $(LDPATH) -lz $(LDLIBS)

romfs:
	$(ROMFSINST) -e CONFIG_USER_CRAMFS_MKCRAMFS /bin/mkcramfs
	$(ROMFSINST) -e CONFIG_USER_CRAMFS_CRAMFSCK /bin/cramfsck

clean:
	-rm -rf $(TARGETS) build *.elf *.gdb *.o


build/mkcramfs: build/mkcramfs.o
	$(BUILD_CC) $(BUILD_CFLAGS) -o $@ $< -lz


build/cramfsck: build/cramfsck.o
	$(BUILD_CC) $(BUILD_CFLAGS) -o $@ $< -lz

build/%.o: %.c
	$(BUILD_CC) $(BUILD_CFLAGS) -c -o $@ $<

build/%: %.c
	$(BUILD_CC) $(BUILD_CFLAGS) -o $@ $<

