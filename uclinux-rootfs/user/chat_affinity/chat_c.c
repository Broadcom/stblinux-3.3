/*
 * chat_c.c - client for chat room benchmark.
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
 * 10 Dec 2005 - Modified to add process affinity. Troy Trammel
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
#include <linux/unistd.h>

#include "chat.h"

/*
 * system calls
 */
int __clone (int (*fn) (void *arg), void *thread_stack, int flags, void *arg, ...);
// TDT
_syscall3 (int, sched_setaffinity, pid_t, pid, unsigned int, len, unsigned long *, user_mask_ptr)
_syscall3 (int, sched_getaffinity, pid_t, pid, unsigned int, len, unsigned long *, user_mask_ptr)

/*
 * prototypes for this file.
 */
int myinit(void);
int main_client_thread(void);
int cli_snd(void *arg);
int cli_rcv(void *arg);
int create_client_socket (int *s);
int wait_for_message(int s, unsigned char server_message);
int mycleanup(void);

/*
 * global data
 */
char *c_send_stack;		/* send thread's stack */
char *c_receive_stack;		/* receive thread's stack */
char *pAddr = 0;		/* server's IP address */

int srvport = 9999;		/* default port number */
int nrooms = 10;		/* default number of rooms */
int nmessages = 100;		/* default number of messages */
int cdebug = 0;			/* debug flags */

unsigned int c_msg_snd_total = 0;	
unsigned int c_msg_rcv_total = 0;

int clientsemall;
int c_donesem;

struct sockaddr_in servername;
const struct sockaddr *svr = (struct sockaddr *) &servername;

START_MESSAGE_S	start_message;
THREAD_INFO_S	*ti;
struct timeval  tvr;

int main (int argc, char *argv[])
{
	int i,k;
	int rc;
	int exit_rc = 0;
	//long long x;
	//long long y;
	float x,y;

	/*
	 * parse parms and init some stuff.
	 */

	setbuf(stdout,NULL);

	if (argc < 2 || argc > 5) {
		printf ("Usage: chat_c server_ip_addr [num_rooms] [num_messages] [server_port]\n");
		exit(1);
	}

	if (!inet_aton(argv[1], &servername.sin_addr.s_addr)) {
		printf ("Invalid Server IP address\n");
		exit(1);
	}

	if (argc > 2) nrooms    = atoi(argv[2]);
	if (argc > 3) nmessages = atoi(argv[3]);
	if (argc > 4) srvport   = atoi(argv[4]);


#if defined(THREAD0) || defined(THREAD1)
{
	// Set processor affinity
	unsigned long new_mask;
	unsigned long cur_mask;
	unsigned int len = sizeof(new_mask);
	pid_t pid;
	pid = getpid();
#ifdef THREAD0
	new_mask = 1;
#else
	new_mask = 2;
#endif

	if (sched_getaffinity(pid, len, &cur_mask) < 0) {
		printf("error: could not get pid %d's affinity.\n", pid);
		return -1;
	}

	printf(" pid %d's old affinity: %08lx\n", pid, cur_mask);

	if (sched_setaffinity(pid, len, &new_mask)) {
		printf("error: could not set pid %d's affinity.\n", pid);
		return -1;
	}

	if (sched_getaffinity(pid, len, &cur_mask) < 0) {
		printf("error: could not get pid %d's affinity.\n", pid);
		return -1;
	}

	printf(" pid %d's new affinity: %08lx\n", pid, cur_mask);
}
#endif

	servername.sin_family      = AF_INET;
	servername.sin_port        = htons(srvport);
	
	/* create the server's listening socket. */

	if (exit_rc = myinit())
		goto CLI_exit;
	if (exit_rc = main_client_thread())
		goto CLI_exit;

	for (i = 0 ; i < nrooms ; i++) {
		for (k = 0 ; k < CLIENTS_PER_ROOM ; k++) {

		        c_msg_snd_total += ti[i*CLIENTS_PER_ROOM+k].msg_snd;
		        c_msg_rcv_total += ti[i*CLIENTS_PER_ROOM+k].msg_rcv;

			if (cdebug) printf ("%u:%u snd_socket = %u\n",
					i,
					k,
					ti[i*CLIENTS_PER_ROOM+k].snd_socket);
			if (cdebug) printf ("%u:%u rcv_socket = %u\n",
					i,
					k,
					ti[i*CLIENTS_PER_ROOM+k].rcv_socket);
			if (cdebug) printf ("%u:%u msg_snd    = %u\n",
					i,
					k,
					ti[i*CLIENTS_PER_ROOM+k].msg_snd);
			if (cdebug) printf ("%u:%u msg_rcv    = %u\n",
					i,
					k,
					ti[i*CLIENTS_PER_ROOM+k].msg_rcv);
		}
	}

	printf ("Simulator version  : %u.%u.%u\n", 1,0,1 );
	printf ("Messages sent      : %u\n", c_msg_snd_total );
	printf ("Messages received  : %u\n", c_msg_rcv_total );
	printf ("Total messages     : %u\n", c_msg_rcv_total+c_msg_snd_total);
	printf ("Elapsed time       : %u.%0.2u seconds\n",
		tvr.tv_sec,
		tvr.tv_usec/1000);
		
	x = tvr.tv_sec + (float)tvr.tv_usec/1000000;
	y = (c_msg_snd_total + c_msg_rcv_total) ;

	printf ("Average throughput : %10.2f messages per second\n", y/x);

	if (c_msg_snd_total != nrooms*CLIENTS_PER_ROOM*nmessages)
		printf ("CLI: expected to snd %u msg, only sent %u msg\n",
			nrooms*CLIENTS_PER_ROOM*nmessages,
		c_msg_snd_total);
CLI_exit:

	mycleanup();

	exit(exit_rc);
}

int main_client_thread(void)
{
	int clone_err = 0;
	int clone_var;
	int stack_index;
	int i,j,k,s;
	int rc;
	int exit_rc = 0;
	int s0;                   /* control socket */
	int wait_status;
	int clients_created = 0;
	struct sembuf mysembuf1;
	struct sembuf mysembuf2;
	unsigned char control_buffer;

	struct timezone tz1;
	struct timeval  tv1;
	struct timezone tz2;
	struct timeval  tv2;

	/*
	 * create the control socket used to talk to the server.
	 */

	exit_rc = create_client_socket (&s0);
	if (exit_rc)
		goto out_main;
	/*
	 * tell the server how many rooms and messages.
	 */
	start_message.nrooms = nrooms;
	start_message.nmessages = nmessages;
	rc = send (s0, &start_message, sizeof(start_message), 0);
	if (rc != sizeof(start_message)) {
		exit_rc = errno;
		perror ("CLI: send() start_message ");
		goto out_main;
	}
	
	/*
	 * create threads via __clone that will connect to the server thread.
	 * loop creates enough clients to satisfy the user specified
	 * number of rooms.
	 */

	for (i = 0 ; i < nrooms ; i++) {
		printf ("Creating room number %u ...\n", i+1);
		for (k = 0 ; k < CLIENTS_PER_ROOM ; k++) {
			clone_var   = i*CLIENTS_PER_ROOM+k;
			stack_index = (clone_var*STACK_SIZE)+STACK_SIZE;

			exit_rc = create_client_socket (&s);
			if (exit_rc)
				goto out_main;
			
			ti[clone_var].snd_socket = s;
			ti[clone_var].rcv_socket = s;

			clone_err = __clone(&cli_snd,
					    &c_send_stack[stack_index],
					    CLONE_FLAGS,
					    (void*)clone_var);
				
			if (clone_err == -1) {
				exit_rc = errno;
				perror ("CLI: clone() ");
				goto out_main;
			}

			clients_created++;
			clone_err = __clone(&cli_rcv,
						&c_receive_stack[stack_index],
						CLONE_FLAGS,
						(void*)clone_var);

			if (clone_err == -1) {
				exit_rc = errno;
				perror ("CLI: clone() ");
				goto out_main;
			}
		
			clients_created++;
		}
		printf ("%u connections so far.\n", (i+1)*CLIENTS_PER_ROOM);
	}

	/* wait for the server to let us start. */

	if (cdebug) printf ("CLI: waiting for CLIENT_START msg from srv\n");

	exit_rc = wait_for_message(s0, CLIENT_START);
	if (exit_rc)
		goto out_main;

	if (cdebug) printf ("CLI: rcved CLIENT_START msg from srvr\n");

	/* clear the sema4 to allow client threads to start together. */

	mysembuf1.sem_num = 0;
	mysembuf1.sem_op  = clients_created;
	mysembuf1.sem_flg = 0;

	mysembuf2.sem_num = 0;
	mysembuf2.sem_op  = 0;
	mysembuf2.sem_flg = 0;

	printf ("Running the test ...\n");
	
	/* get start time */

	rc = gettimeofday (&tv1, &tz1);
	if (rc) {
		exit_rc = errno;
		perror ("gettimeofday failed on tv1 ");
		goto out_main;
	}

	/* start all of the clients */

	rc = semop (clientsemall, &mysembuf1, 1);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CLI: semop(clientsemall) failed ");
		goto out_main;
	}

	/* wait until all of the clients complete */

	rc = semop (c_donesem, &mysembuf2, 1);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CLI: semop(c_donesem) failed ");
		goto out_main;
	}
	
	/* get the end time */

	rc = gettimeofday (&tv2, &tz2);
	if (rc) {
		exit_rc = rc;
		perror ("gettimeofday failed on tv2 ");
		goto out_main;
	}

	printf ("Test complete.\n\n");

	timersub(&tv2, &tv1, &tvr); /* tvr now contains result of tv2-tv1 */

	for (i = 0 ; i < nrooms * CLIENTS_PER_ROOM ; i++) {
		rc = waitpid (-1, &wait_status, __WCLONE);
		if (rc == -1) {
			exit_rc = errno;
			perror ("SRV: wait() ");
		}
		rc = waitpid (-1, &wait_status, __WCLONE);
		if (rc == -1) {
			exit_rc = errno;
			perror ("SRV: wait() ");
		}
	}
	
	for (i = 0 ; i < nrooms * CLIENTS_PER_ROOM ; i++) {
		close(ti[i].snd_socket);
	}

	/*
	 * send a control character to the main_server_thread to let him know
	 * we are done;
	 */

	control_buffer = CLIENT_END;

	rc = send (s0, &control_buffer, 1, 0);
	if (rc != 1) {
		exit_rc = errno;
		perror ("CLI: send() CLIENT_END control byte ");
		goto out_main;
	}
  
out_main:

	close(s0);
	return(exit_rc);
}

int cli_snd(void *arg)
{
	int i;
	int s1;
	int rc;
	int tn = (int)arg;
	int exit_rc = 0;
	struct sembuf mysembuf;
	int c_send_buffer[BUFFER_SIZE/4];
	int c_send_size;

	for (i = 0 ; i < BUFFER_SIZE/4 ; i++) c_send_buffer[i] = tn;

	/*
	 * wait for the client_main_thread to release us.
	 */
	mysembuf.sem_num = 0;
	mysembuf.sem_op  = -1;
	mysembuf.sem_flg = 0;

	if (cdebug) printf ("CST(%u): calling semop(clientsemall)\n", tn);

	rc = semop (clientsemall, &mysembuf, 1);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CST: semop(clientsemall) failed");
		close(ti[tn].rcv_socket);
		goto snd_exit;
	}	
	
	if (cdebug) printf ("CST(%u): returned from semop(clientsemall)\n", tn);
	
	/* send the number of messages */

	c_send_size = BUFFER_SIZE;
	s1 = ti[tn].snd_socket;

	for (i = 0 ; i < nmessages ; i++) {
	
try_again:

		rc = send (s1, c_send_buffer, c_send_size, 0);
		if (rc == -1) {
			exit_rc = errno;
			perror ("CST send() ");
			goto snd_exit;
		}
		
		if (cdebug) printf ("CST(%u): send() of %u bytes\n", tn, rc);
		
		if (rc == c_send_size) {
			c_send_size = BUFFER_SIZE;
			ti[tn].msg_snd++;
		} else {
			c_send_size -= rc;
			printf ("CST(%u): send(), not all data sent, rc = %u\n",
				tn,
				rc);
			goto try_again;
		}
	}

	/* tell the main client thread that we are done that we are done. */

	mysembuf.sem_num = 0;
	mysembuf.sem_op  = -1;
	mysembuf.sem_flg = 0;

	if (cdebug) printf ("CST(%u): calling semop(c_donesem)\n", tn);

	rc = semop (c_donesem, &mysembuf, 1);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CST: semop(c_donesem) failed ");
	}

snd_exit:

	return(exit_rc);
}

int cli_rcv(void *arg)
{
	int i;
	int rc;
	int s1;
	int tn = (int)arg;
	int nwaitfor;
	int exit_rc = 0;
	struct sembuf mysembuf;
	int c_receive_buffer[BUFFER_SIZE/4];

	nwaitfor = (CLIENTS_PER_ROOM - 1) * nmessages;

	/*
	 * wait for the client_main_thread to release us.
	 */
	mysembuf.sem_num = 0;
	mysembuf.sem_op  = -1;
	mysembuf.sem_flg = 0;

	if (cdebug) printf ("CRT(%u): calling semop(clientsemall)\n", tn);

	rc = semop (clientsemall, &mysembuf, 1);
	if (rc == -1) {
		exit_rc = errno;
  		perror ("CRT: semop(clientsemall) failed ");
		close(ti[tn].rcv_socket);
		goto rcv_exit;
	}	
	
	if (cdebug) printf ("CRT(%u): returned from semop(clientsemall)\n", tn);

	/* receive the messages from the server send thread. */

	s1 =  ti[tn].rcv_socket;

	for (i = 0 ; i < nwaitfor ; i++) {

		rc = recv (s1, c_receive_buffer, BUFFER_SIZE, 0x100);

		if (rc == -1) {
			exit_rc = errno;
			perror ("CRT: recv() ");
			goto rcv_exit;
		}

		if (cdebug) printf ("CRT(%u): recv() %u bytes\n",tn,rc);

		if (rc == BUFFER_SIZE)
			ti[tn].msg_rcv++;
		else {
			printf ("CRT(%u): only %u rcved\n", tn, rc);
			exit_rc = -1;
			goto rcv_exit;
		}
	}

	/* tell the main client thread that we are done that we are done. */

	mysembuf.sem_num = 0;
	mysembuf.sem_op  = -1;
	mysembuf.sem_flg = 0;

	if (cdebug) printf ("CRT(%u): calling semop(c_donesem)\n", tn);

	rc = semop (c_donesem, &mysembuf, 1);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CRT: semop(c_donesem) failed ");
	}

rcv_exit:

	return(exit_rc);

}

int mycleanup(void)
{
	int exit_rc = 0;
	int rc;

	rc = semctl(c_donesem, 0, IPC_RMID, 0);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CLEANUP: semctl(c_donesem) failed ");
		goto cleanup_exit;
	}

	rc = semctl(clientsemall, 0, IPC_RMID, 0);
	if (rc == -1) {
		exit_rc = errno;
		perror ("CLEANUP: semctl(clientsemall) failed ");
		goto cleanup_exit;
	}

cleanup_exit:

	return(exit_rc);
}

int myinit(void)
{
	int i,j,k;
	int exit_rc = 0;
	int rc;
	struct sembuf mysembuf;

	/*
	 * allocate storage for the room info.
	 */
	ti = malloc(CLIENTS_PER_ROOM*nrooms*sizeof(struct THREAD_INFO));
	if (ti == NULL) {
		exit_rc = errno;
		perror ("INIT: malloc of 'ti' failed");
		goto init_exit;
	}

	for (i = 0 ; i < nrooms ; i++) {
		for (k = 0 ; k < CLIENTS_PER_ROOM ; k++) {
			ti[i*CLIENTS_PER_ROOM+k].roomnumber     = i;
			ti[i*CLIENTS_PER_ROOM+k].roomsem	= 0;
			ti[i*CLIENTS_PER_ROOM+k].snd_socket	= 0;
			ti[i*CLIENTS_PER_ROOM+k].rcv_socket	= 0;
			ti[i*CLIENTS_PER_ROOM+k].msg_snd	= 0;
			ti[i*CLIENTS_PER_ROOM+k].msg_rcv	= 0;
		}
	}

	/* allocate storage for stacks. */

	c_send_stack = malloc(CLIENTS_PER_ROOM*nrooms*STACK_SIZE);
	if (c_send_stack == NULL) {
		exit_rc = errno;
		perror ("INIT: malloc of 'c_send_stack' failed ");
		goto init_exit;
	}

	c_receive_stack = malloc(CLIENTS_PER_ROOM*nrooms*STACK_SIZE);
	if (c_receive_stack == NULL) {
		exit_rc = errno;
		perror ("INIT: malloc of 'c_receive_stack' failed ");
		goto init_exit;
	}

	/* This sem is used to release all client threads. */

	clientsemall = semget (IPC_PRIVATE, 1, IPC_CREAT | IPC_EXCL | 0660);
	if (clientsemall == -1) {
		exit_rc = errno;
		perror ("INIT: semget(clientsemall) failed ");
		goto init_exit;
	} else if (cdebug) printf("INIT: clientsemall = %u.\n", clientsemall);

	/*
	 * This sem is waited on by the main server thread and cleared by the
	 * main client thread so that the main server thread knows when the
	 * main clinet thread is done.
	 */
	 
	c_donesem = semget (IPC_PRIVATE, 1, IPC_CREAT | IPC_EXCL | 0660);
	if (c_donesem == -1) {
		exit_rc = errno;
		perror("INIT: semget(c_donesem) failed ");
		goto init_exit;
	} else if (cdebug) printf("INIT: c_donesem = %u.\n", c_donesem);

	rc = semctl(c_donesem, 0, SETVAL, nrooms*CLIENTS_PER_ROOM*2);
	if (rc == -1) {
		exit_rc = errno;
		perror ("INIT: semctl(c_donesem) failed ");
		goto init_exit;
	}

init_exit:

	return(exit_rc);
}

int create_client_socket (int *s)
{
	int s1;
	int rc;
	int exit_rc = 0;

	/* create a socket that will connect to the server.*/

	s1 = socket (PF_INET, SOCK_STREAM, 0);
	if (s1 == -1) {  
		exit_rc = errno;
		perror ("CLI: error creating client socket");
		goto out_cr;
	}

	/* connect the socket to the server. */

	if (cdebug) printf ("CLI: connect() on cli socket %u\n", s1);

	rc = connect (s1, svr, sizeof(servername));
	if (!rc) {
		*s = s1;
		if (cdebug) printf ("CLI: socket %u connected to srv\n", s1);
	} else {
		exit_rc = errno;
		perror ("CLI: error connecting socket ");
		close(s1);
	}
  
out_cr:  
	return(exit_rc);
}

int wait_for_message(int s, unsigned char message)
{
	int rc;
	int exit_rc = 0;
	unsigned char control_buffer;

	rc = recv (s, &control_buffer, 1, 0x100);
	if (rc != -1) {
		if (rc == 1 && control_buffer == message) {
			exit_rc = 0;
		} else {
			exit_rc = -1;
			printf ("CLI: msg err %X %X\n",
				(unsigned int)control_buffer,
				(unsigned int)message);
		}
	} else {
		exit_rc = errno;
		perror ("CLI: recv() ");
	}
	return(exit_rc);
}
