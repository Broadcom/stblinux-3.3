/*
 * chat_s.c - server for chat room benchmark.
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
 *
 * 10 Dec 2000 - 1.0.1 Initial version. Bill Hartner
 *
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <wait.h>

#include "chat.h"

/*
 * system calls
 */
 
int __clone (int (*fn) (void *arg), void *thread_stack, int flags, void *arg, ...);

/*
 * prototypes for this file.
 */
int myinit(void);
int srv_snd(void *arg);
int srv_rcv(void *arg);
int wait_for_message(int s, unsigned char server_message);
int mycleanup(void);

/*
 * global data
 */
char *s_send_stack;		/* send thread's stack */
char *s_receive_stack;		/* receive thread's stack */

int srvport = 9999;		/* default port number */
int nrooms;			/* number of rooms */
int nmessages;			/* number of messages */
int sdebug = 0;			/* debug flags */

unsigned int s_msg_snd_total;
unsigned int s_msg_rcv_total;

struct timezone tz1;
struct timeval  tv1;
struct timezone tz2;
struct timeval  tv2;
struct timeval  tvr;

int s_donesem;

struct sockaddr_in servername;
START_MESSAGE_S start_message;
THREAD_INFO_S *ti;

int main (int argc, char *argv[])
{
	int clone_err;
	int clone_var;
	int i,j,k;
	int s0;   /* control socket */
	int s1;   /* listening socket */
	int s2;   /* server thread socket */
	int rc;
	int exit_rc = 0;
	int wait_status;
	int clinamelen;
	unsigned long testnum = 0;
	struct sockaddr_in clientname;
	struct sembuf mysembuf;
	unsigned char control_buffer;

	/* parse parms and init some stuff */

	setbuf(stdout,NULL);

	if (argc < 2 || argc > 3) {
		printf ("Usage: chat_s server_ip_addr [server_port]\n");
		exit(1);
	}

	if (!inet_aton(argv[1], &servername.sin_addr.s_addr)) {
		printf ("Invalid Server IP address\n");
		exit(1);
	}

	if (argc > 2) srvport = atoi(argv[2]);

	servername.sin_family = AF_INET;
	servername.sin_port   = htons(srvport);
	
	/* create the server's listening socket. */

	s1 = socket (PF_INET, SOCK_STREAM, 0);
	if (s1 == -1) {
		exit_rc = errno;
		perror ("SVR: socket() ");
		goto SRV_exit;
	}
	
	if (sdebug) printf ("SVR: listen socket = %u\n", s1);

	/* bind the server's port number/ip addr to the socket */

	rc = bind (s1, (struct sockaddr *) &servername, sizeof(servername));
        if (rc) {
		exit_rc = errno;
	        perror ("SVR: bind() ");
		goto SRV_exit;
        }

	/* allow the socket to accept connections. */

	rc = listen (s1, 4096);
	if (rc) {
		exit_rc = errno;
		perror ("SVR: listen() ");
		goto SRV_exit;
	}

restart:

	testnum++;

	/* accept a connection from the client for the control socket. */

	printf ("\nSRV: waiting for client to connect the control socket...\n");

	s0 = accept (s1, (struct sockaddr *)&clientname, &clinamelen);
	if (s0 == -1) {
		exit_rc = errno;
		perror ("SVR: accept() on control socket ");
		goto SRV_exit;
	}

	rc = recv (s0,
		   &start_message,
		   sizeof(start_message),
		   0x100);
	if (rc == -1) {
		exit_rc = errno;
		perror ("SRT: recv() of start_message");
		goto SRV_exit;
	}

	nrooms = start_message.nrooms;
	nmessages = start_message.nmessages;

	printf ("SRV: client connected, rooms = %d messages = %d\n",nrooms,nmessages);

	if (myinit()) exit(rc);

	/*
	 * MAIN server loop where connections are accepted
	 * and a server receive and send thread is created to handle the
	 * connection.
	 */

	for (i = 0 ; i < nrooms ; i++) {
		for (k = 0 ; k < CLIENTS_PER_ROOM ; k++) {

			clone_var = i*CLIENTS_PER_ROOM+k;

		        if (sdebug) printf ("SVR: accept() srv socket %u\n",s1);

	        	clinamelen = sizeof(clientname);
		        s2 = accept (s1,
				     (struct sockaddr *)&clientname,
				     &clinamelen);
			if (s2 == -1) {
				exit_rc = errno;
				perror ("SVR: accept() ");
				goto SRV_exit;
			}

			/* create srv_rcv for this connection. */

			ti[clone_var].rcv_socket = s2;
			ti[clone_var].snd_socket = s2;

			if (sdebug) printf ("SVR: connection on sock %u\n", s2);
			
			clone_err = __clone(&srv_rcv, 
				   &s_receive_stack[(clone_var+1)*STACK_SIZE],
				   CLONE_FLAGS,
				   (void*)clone_var);
			if (clone_err == -1) {
				exit_rc = errno;
				perror ("SRV: clone() ");
				goto SRV_exit;
			}
			
			clone_err = __clone(&srv_snd,
        	                      &s_send_stack[(clone_var+1)*STACK_SIZE],
                	              CLONE_FLAGS,
                        	      (void*)clone_var);
			if (clone_err == -1) {
				exit_rc = errno;
				perror ("SRV: clone() ");
				goto SRV_exit;
			}
		}
	}

	if (sdebug) printf ("SRV: sending CLIENT_START message to client\n");

	/*
	 * send a control character to the main_client_thread to let him know
	 * we are ready to start.
	 */
	control_buffer = CLIENT_START;

	rc = send (s0, &control_buffer, 1, 0);
	if (rc != 1) {
		exit_rc = errno;
		perror ("SRV: send() CLIENT_START msg ");
		goto SRV_exit;
	}

	/*
	 * wait until all of the server threads complete
	 */

	printf ("SRV: waiting for server threads to complete...\n");

	mysembuf.sem_num = 0;
	mysembuf.sem_op  = 0;
	mysembuf.sem_flg = 0;

	rc = semop (s_donesem, &mysembuf, 1);
	if (rc == -1) {
		exit_rc = errno;
		perror ("SRV: semop(s_donesem) ");
		goto SRV_exit;
	}
	
	printf ("SRV: server threads completed.\n");

	/*
	 * now wait until the main client thread has completed.
	 */

	printf ("SRV: waiting for CLIENT_END message from client...\n");

	exit_rc = wait_for_message(s0, CLIENT_END);
	if (exit_rc)
		goto SRV_exit;

	if (sdebug) printf ("SRV: received CLIENT_END message from client\n");

	/* wait for the server threads to die */

	for (i = 0 ; i < nrooms * CLIENTS_PER_ROOM ; i++) {
		rc = waitpid (-1, &wait_status, __WCLONE);
		if (rc == -1) {
			exit_rc = errno;
			perror ("SRV: wait() ");
			goto SRV_exit;
		}
		rc = waitpid (-1, &wait_status, __WCLONE);
		if (rc == -1) {
			exit_rc = errno;
			perror ("SRV: wait() ");
			goto SRV_exit;
		}
	}

	/* close the sockets used by send/receive threads */

	for (i = 0 ; i < nrooms * CLIENTS_PER_ROOM ; i++) {
		close(ti[i].snd_socket);
	}
	
	for (i = 0 ; i < nrooms ; i++) {
		for (k = 0 ; k < CLIENTS_PER_ROOM ; k++) {
			s_msg_snd_total += ti[i*CLIENTS_PER_ROOM+k].msg_snd;
			s_msg_rcv_total += ti[i*CLIENTS_PER_ROOM+k].msg_rcv;

			if (sdebug) printf ("%u:%u roomnumber   = %u\n", i, k, ti[i*CLIENTS_PER_ROOM+k].roomnumber);
			if (sdebug) printf ("%u:%u roomsem      = %u\n", i, k, ti[i*CLIENTS_PER_ROOM+k].roomsem);
			if (sdebug) printf ("%u:%u snd_socket = %u\n", i, k, ti[i*CLIENTS_PER_ROOM+k].snd_socket);
			if (sdebug) printf ("%u:%u rcv_socket = %u\n", i, k, ti[i*CLIENTS_PER_ROOM+k].rcv_socket);
      
			if (sdebug) printf ("%u:%u msg_snd    = %u\n", i, k, ti[i*CLIENTS_PER_ROOM+k].msg_snd);
			if (sdebug) printf ("%u:%u msg_rcv    = %u\n", i, k, ti[i*CLIENTS_PER_ROOM+k].msg_rcv);
      
		}
	}

	printf ("SRV(%u) %u messages received, %u messages sent\n", testnum, s_msg_rcv_total, s_msg_snd_total); 

	if (s_msg_rcv_total != nrooms*CLIENTS_PER_ROOM*nmessages) {
		printf ("SRV: expected to receive %u msgs, received %u msgs\n",
			nrooms*CLIENTS_PER_ROOM*nmessages,
			s_msg_rcv_total);
	}

	close(s0);
	mycleanup();
	goto restart;
	
SRV_exit:
	close(s0);
	close(s1);
	mycleanup();
	exit(exit_rc);
}

/***********************************************/

int srv_snd(void *arg)
{
	int i,j;
	int rc;
	int tn = (int)arg;
	int startroom = ((tn / CLIENTS_PER_ROOM) * CLIENTS_PER_ROOM);
	int semnum = tn % CLIENTS_PER_ROOM;
	int exit_rc = 0;
	int s_send_size;
	int s_send_buffer[BUFFER_SIZE/4];
	struct sembuf mysembuf[2];

	if (sdebug)
		printf ("SST(%u): startroom = %u, semnum = %u\n",
			tn,
			startroom,
			semnum);

	/*
	 * first get the message from the server receive thread and
	 * then resend it to the other clients in the room.
	 */
	rc = 0;
	s_send_size = BUFFER_SIZE;

	for (i = 0 ; i < nmessages ; i++) {
		/*
		 * wait on the sema4 that indicates a message to broadcast to
		 * all clinets in the room.
		 */
		mysembuf[0].sem_num = semnum;
		mysembuf[0].sem_op  = -1;
		mysembuf[0].sem_flg = 0;

		rc = semop (ti[tn].roomsem, mysembuf, 1);
		if (rc == -1) {
			exit_rc = errno;
			perror ("SST: semop(roomsem) ");
			goto srv_snd_err;
		}
		
		for (j = startroom ; j < startroom+CLIENTS_PER_ROOM ; j++) {
			/*
			 * skip the client sender.
			 */
			if (j != tn) {
try_again:
				rc = send (ti[j].snd_socket,
					   s_send_buffer,
					   s_send_size,
					   0);
				if (rc == -1) {
					exit_rc = errno;
					perror ("SST: snd() error ");
					goto srv_snd_err;
				}
				
				if (rc == s_send_size) {
					/*
					 * entire message was sent;
					 */
					s_send_size = BUFFER_SIZE;
					ti[tn].msg_snd++;
				} else {
					/*
					 * partial message.
					 */
					s_send_size -= rc;
					goto try_again;
				}
			} /* if j != tn */
		} /* for */
	} /* for */
	
srv_snd_err:

	/*
	 * tell the main server thread that we are done that we are done.
	 */
	mysembuf[0].sem_num = 0;
	mysembuf[0].sem_op  = -1;
	mysembuf[0].sem_flg = 0;

	if (sdebug) printf ("SST(%u): calling semop(s_donesem)\n", tn);

	rc = semop (s_donesem, mysembuf, 1);

	if (rc == -1) {
		exit_rc = errno;
		perror ("SST: semop(s_donesem) ");
	}	

	if (sdebug) printf ("SST(%u): exiting\n", tn);
	return(exit_rc);
} /* end srv_snd */

int srv_rcv(void *arg)
{
	int i;
	int rc;
	int tn = (int)arg;
	int startroom = ((tn / CLIENTS_PER_ROOM) * CLIENTS_PER_ROOM);
	int semnum = tn % CLIENTS_PER_ROOM;
	int exit_rc = 0;
	int s_receive_buffer[BUFFER_SIZE/4];
	struct sembuf mysembuf;

	if (sdebug)
		printf ("SRT(%u): startroom = %u, semnum = %u\n",
			tn,
			startroom,
			semnum);

	rc = 0;
	for (i = 0 ; i < nmessages ; i++) {
	
		rc = recv (ti[tn].rcv_socket,
			   s_receive_buffer,
			   BUFFER_SIZE,
			   0x100);
		if (rc == -1) {
			exit_rc = errno;
			perror ("SRT: recv() ");
			goto srv_rcv_err;
		}
		
		if (sdebug) printf ("SRT(%u): recv() of %u bytes\n", tn, rc);
		
		if (rc == BUFFER_SIZE) {
			/*
			 * tell the server send thread to broadcast to
			 * all clinets in the room.
			 */

			mysembuf.sem_num = semnum;
			mysembuf.sem_op  = 1;
			mysembuf.sem_flg = 0;

			rc = semop (ti[tn].roomsem, &mysembuf, 1);
			if (rc == -1) {
				exit_rc = errno;
				perror ("SRT: semop(roomsem) ");
				goto srv_rcv_err;
			}
			ti[tn].msg_rcv++;
		} else {
			printf ("SRT(%u): recv(), part rcv, rc = %u\n", tn, rc);
			goto srv_rcv_err;
		}
	}
	
srv_rcv_err:

	/*
	 * tell the main server thread that we are done that we are done.
	 */
	mysembuf.sem_num = 0;
	mysembuf.sem_op  = -1;
	mysembuf.sem_flg = 0;

	if (sdebug) printf ("SRT(%u): calling semop(s_donesem)\n", tn);

	rc = semop (s_donesem, &mysembuf, 1);

	if (rc == -1) {
		exit_rc = errno;
		perror ("SRT: semop(s_donesem) ");
	}

	if (sdebug) printf ("SRT(%u): exiting\n", tn);
	return(exit_rc);
} /* end srv_rcv */

int mycleanup(void)
{
	int i;
	int exit_rc = 0;
	int rc = 0;
	
	free(ti);
	free(s_send_stack);
	free(s_receive_stack);
	
	rc = semctl(s_donesem, 0, IPC_RMID, 0);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CLEANUP: semctl(s_donesem) failed ");
	}

	for (i = 0 ; i < nrooms ; i++) {
		rc = semctl(ti[i*CLIENTS_PER_ROOM].roomsem, 0, IPC_RMID, 0);
		if (rc == -1) {
			exit_rc = errno;
			perror ("CLEANUP: semctl(roomsem) failed ");
		}	
	}

	return(exit_rc);
}

int myinit(void)
{
	int i,j,k;
	int roomsem;
	int exit_rc = 0;
	int rc;
	struct sembuf mysembuf;

	s_msg_snd_total = 0;
	s_msg_rcv_total = 0;

	/*
	 * allocate storage for the room info.
	 */
	ti = malloc(CLIENTS_PER_ROOM*nrooms*sizeof(struct THREAD_INFO));
	if (ti == NULL)
	{
		exit_rc = errno;
		perror ("INIT: malloc of 'ti' failed ");
		goto out_init;
	}

	for (i = 0 ; i < nrooms ; i++) {
		/*
		 * server uses the room sem
		 */
		roomsem = semget (IPC_PRIVATE,
				  CLIENTS_PER_ROOM,
				  IPC_CREAT | IPC_EXCL | 0660);
		if (roomsem == -1) {
			exit_rc = errno;
			perror("INIT: semget(roomsem) ");
			goto out_init;
		}
		if (sdebug) printf("INIT: roomsem(%u) = %u.\n", i, roomsem);

		for (k = 0 ; k < CLIENTS_PER_ROOM ; k++) {
			ti[i*CLIENTS_PER_ROOM+k].roomnumber     = i;
			ti[i*CLIENTS_PER_ROOM+k].roomsem        = roomsem;
			ti[i*CLIENTS_PER_ROOM+k].snd_socket   = 0;
			ti[i*CLIENTS_PER_ROOM+k].rcv_socket   = 0;
			ti[i*CLIENTS_PER_ROOM+k].msg_snd      = 0;
			ti[i*CLIENTS_PER_ROOM+k].msg_rcv      = 0;
		}
	}

	s_send_stack = malloc(CLIENTS_PER_ROOM*nrooms*STACK_SIZE);
	if (s_send_stack == NULL)
	{
		exit_rc = errno;
		perror ("INIT: malloc of 's_send_stack' failed");
		goto out_init;
	 }

	s_receive_stack = malloc(CLIENTS_PER_ROOM*nrooms*STACK_SIZE);
	if (s_receive_stack == NULL)
	{
		exit_rc = errno;
		perror ("INIT: malloc of 's_receive_stack' failed");
		goto out_init;
	}

	s_donesem = semget (IPC_PRIVATE, 1, IPC_CREAT | IPC_EXCL | 0660);
	if (s_donesem == -1) {
		exit_rc = errno;
		perror("INIT: semget(s_donesem) ");
		goto out_init;
	} else {
	      if (sdebug) printf("INIT: s_donesem = %u.\n", s_donesem);
	}

	rc = semctl(s_donesem, 0, SETVAL, nrooms*CLIENTS_PER_ROOM*2);
	if (rc == -1)
	{
		exit_rc = errno;
		perror("INIT: semctl(s_donesem) ");
		goto out_init;
	}

out_init:

	return(exit_rc);
}

int wait_for_message(int s, unsigned char message)
{
	int rc;
	int exit_rc;
	unsigned char control_buffer;

	rc = recv (s, &control_buffer, 1, 0x100);
	if (rc != -1) {
		if (rc == 1 && control_buffer == message)
			exit_rc = 0;
		else {
			exit_rc = -1;
			printf ("SRV: srv msg err %X %X\n",
				(unsigned int)control_buffer,
				(unsigned int)message);
		}
	} else
		exit_rc = errno;
		
  return(exit_rc);
}
