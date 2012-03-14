
/*
 * chat.h - header file for chat room benchmark.
 *
 * Copyright (C) 2000 IBM
 *
 * Written by Bill Hartner (bhartner@us.ibm.com) 10 Dec 2000
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * constants
 */
#define STACK_SIZE		(16384)
#define CLIENTS_PER_ROOM	(20)
#define BUFFER_SIZE		(100)
#define CLIENT_START		(0x11)
#define CLIENT_END		(0x22)
#define CLONE_FLAGS	(CLONE_VM | CLONE_SIGHAND | CLONE_FS | CLONE_FILES)

typedef struct START_MESSAGE
{
	int nrooms;
	int nmessages;
} START_MESSAGE_S;

typedef struct THREAD_INFO
{
	int roomnumber;
	int roomsem;
	int snd_socket;
	int rcv_socket;
	int msg_snd;
	int msg_rcv;
	char buffer[BUFFER_SIZE];
	char pad[BUFFER_SIZE % sizeof(int)];
} THREAD_INFO_S;

