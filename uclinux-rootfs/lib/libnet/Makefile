
# CFLAGS += -I../libc/include

SRC = net.c
OBJS = openraw.o closeraw.o setifflags.o resetifflags.o setifaddr.o \
	setifdstaddr.o setifbrdaddr.o setifnetmask.o setifmtu.o \
	addroute.o maskaddress.o setifhwaddr.o

LIBNET = libnet.a


all: $(LIBNET)

$(LIBNET): $(OBJS)
	$(AR) rcv $(LIBNET) $(OBJS)
	$(RANLIB) $(LIBNET)

$(OBJS): $(SRC)
	$(CC) $(CFLAGS) -DL_$(basename $@) -o $(basename $@).o -c $^

romfs:

clean:
	-rm -f $(LIBNET) *.o

