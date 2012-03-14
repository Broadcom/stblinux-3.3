
all: chat.h chat_c.c chat_s.c makefile
	$(CC) -DTHREAD1 -o chat_c_tp1 chat_c.c
	$(CC) -DTHREAD0 -o chat_c_tp0 chat_c.c
	$(CC) -o chat_c chat_c.c
	$(CC) -o chat_s chat_s.c

clean: 
	rm -rf chat_c_tp0 chat_c_tp1 chat_c chat_s 

romfs:
	$(ROMFSINST) /bin/chat_c_tp0
	$(ROMFSINST) /bin/chat_c_tp1
	$(ROMFSINST) /bin/chat_c
	$(ROMFSINST) /bin/chat_s
