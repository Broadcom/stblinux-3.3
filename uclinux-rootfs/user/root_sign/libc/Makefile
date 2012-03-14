TARGET := libc.a 

SRCS = string.c stdio.c libc_init.c malloc.c memcpy.c mmap.c ctypes.c sys.c

OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	@rm -f $@
	$(AR) cr $@ $(OBJS)

clean:
	@rm -f $(TARGET) $(OBJS) 
