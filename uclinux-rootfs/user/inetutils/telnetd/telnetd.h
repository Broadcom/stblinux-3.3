/* Copyright (C) 1998,2001 Free Software Foundation, Inc.

   This file is part of GNU Inetutils.

   GNU Inetutils is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   GNU Inetutils is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GNU Inetutils; see the file COPYING.  If not, write to
   the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA. */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <syslog.h>
#include <string.h>
#ifdef HAVE_STRINGS_H
# include <strings.h>
#endif
#ifdef HAVE_SYS_FILIO_H
# include <sys/filio.h>
#endif
#ifdef HAVE_SYS_TTY_H
# include <sys/tty.h>
#endif
#ifdef HAVE_SYS_PTYVAR_H
# include <sys/ptyvar.h>
#endif
#ifdef HAVE_STROPTS_H
# include <stropts.h>
#endif
#include <sys/ioctl.h>

#include <arpa/telnet.h>
#include <libtelnet/auth.h>

#ifndef	P
# if defined PROTOTYPES || (defined __STDC__ && __STDC__)
#  define P(x)	x
# else
#  define P(x)	()
# endif
#endif

#if defined(HAVE_TERMIOS_H)
# include <termios.h>
#elif defined(HAVE_TERMIO_H)
# include <termio.h>
#else
# include <sgtty.h>
#endif

#define obstack_chunk_alloc malloc
#define obstack_chunk_free free
#include <obstack.h>

#ifndef HAVE_CC_T
typedef unsigned char cc_t;
#endif

typedef enum debug_mode
{
    debug_options,
    debug_report,
    debug_net_data,
    debug_pty_data,
    debug_auth,
    debug_max_mode
} debug_mode_t;

#define MAX_DEBUG_LEVEL 100

extern int debug_level[];

#define DEBUG(mode,level,c) if (debug_level[mode]>=level) c

struct telnetd_clocks
{
  int system;           /* what the current time is */         
  int echotoggle;	/* last time user entered echo character */ 
  int modenegotiated;	/* last time operating mode negotiated */
  int didnetreceive;	/* last time we read data from network */
  int ttypesubopt;	/* ttype subopt is received */         
  int tspeedsubopt;	/* tspeed subopt is received */        
  int environsubopt;	/* environ subopt is received */       
  int oenvironsubopt;	/* old environ subopt is received */   
  int xdisplocsubopt;	/* xdisploc subopt is received */      
  int baseline;		/* time started to do timed action */  
  int gotDM;		/* when did we last see a data mark */ 
};
#define	settimer(x)	(clocks.x = ++clocks.system)
#define	sequenceIs(x,y)	(clocks.x < clocks.y)

/*
 * Structures of information for each special character function.
 */
typedef struct
{
  unsigned char	flag;  /* the flags for this function */
  cc_t		val;   /* the value of the special character */
} slcent, *Slcent;

typedef struct
{
  slcent defset;       /* the default settings */
  slcent current;      /* the current settings */
  cc_t *sptr;	       /* a pointer to the char in */
		       /* system data structures */
} slcfun, *Slcfun;

#ifdef HAVE_UNAME
   /* Prefix and suffix if the IM string can be generated from uname.  */
#  define UNAME_IM_PREFIX "\r\n"
#  define UNAME_IM_SUFFIX " (%h) (%t)\r\n\n"
#else /* ! HAVE_UNAME */
#  define UNAME_IM_PREFIX "\r\n"
#  define UNAME_IM_SUFFIX "\r\n"
#endif

/* ********************* */
/* State machine */
/*
 * We keep track of each side of the option negotiation.
 */

#define	MY_STATE_WILL		0x01
#define	MY_WANT_STATE_WILL	0x02
#define	MY_STATE_DO		0x04
#define	MY_WANT_STATE_DO	0x08

/*
 * Macros to check the current state of things
 */

#define	my_state_is_do(opt)		(options[opt]&MY_STATE_DO)
#define	my_state_is_will(opt)		(options[opt]&MY_STATE_WILL)
#define my_want_state_is_do(opt)	(options[opt]&MY_WANT_STATE_DO)
#define my_want_state_is_will(opt)	(options[opt]&MY_WANT_STATE_WILL)

#define	my_state_is_dont(opt)		(!my_state_is_do(opt))
#define	my_state_is_wont(opt)		(!my_state_is_will(opt))
#define my_want_state_is_dont(opt)	(!my_want_state_is_do(opt))
#define my_want_state_is_wont(opt)	(!my_want_state_is_will(opt))

#define	set_my_state_do(opt)		(options[opt] |= MY_STATE_DO)
#define	set_my_state_will(opt)		(options[opt] |= MY_STATE_WILL)
#define	set_my_want_state_do(opt)	(options[opt] |= MY_WANT_STATE_DO)
#define	set_my_want_state_will(opt)	(options[opt] |= MY_WANT_STATE_WILL)

#define	set_my_state_dont(opt)		(options[opt] &= ~MY_STATE_DO)
#define	set_my_state_wont(opt)		(options[opt] &= ~MY_STATE_WILL)
#define	set_my_want_state_dont(opt)	(options[opt] &= ~MY_WANT_STATE_DO)
#define	set_my_want_state_wont(opt)	(options[opt] &= ~MY_WANT_STATE_WILL)

/*
 * Tricky code here.  What we want to know is if the MY_STATE_WILL
 * and MY_WANT_STATE_WILL bits have the same value.  Since the two
 * bits are adjacent, a little arithmatic will show that by adding
 * in the lower bit, the upper bit will be set if the two bits were
 * different, and clear if they were the same.
 */
#define my_will_wont_is_changing(opt) \
			((options[opt]+MY_STATE_WILL) & MY_WANT_STATE_WILL)

#define my_do_dont_is_changing(opt) \
			((options[opt]+MY_STATE_DO) & MY_WANT_STATE_DO)

/*
 * Make everything symetrical
 */

#define	HIS_STATE_WILL			MY_STATE_DO
#define	HIS_WANT_STATE_WILL		MY_WANT_STATE_DO
#define HIS_STATE_DO			MY_STATE_WILL
#define HIS_WANT_STATE_DO		MY_WANT_STATE_WILL

#define	his_state_is_do			my_state_is_will
#define	his_state_is_will		my_state_is_do
#define his_want_state_is_do		my_want_state_is_will
#define his_want_state_is_will		my_want_state_is_do

#define	his_state_is_dont		my_state_is_wont
#define	his_state_is_wont		my_state_is_dont
#define his_want_state_is_dont		my_want_state_is_wont
#define his_want_state_is_wont		my_want_state_is_dont

#define	set_his_state_do		set_my_state_will
#define	set_his_state_will		set_my_state_do
#define	set_his_want_state_do		set_my_want_state_will
#define	set_his_want_state_will		set_my_want_state_do

#define	set_his_state_dont		set_my_state_wont
#define	set_his_state_wont		set_my_state_dont
#define	set_his_want_state_dont		set_my_want_state_wont
#define	set_his_want_state_wont		set_my_want_state_dont

#define his_will_wont_is_changing	my_do_dont_is_changing
#define his_do_dont_is_changing		my_will_wont_is_changing

/* ******* */

void fatal (int f, char *msg);

/*
 * Linemode support states, in decreasing order of importance
 */
#define REAL_LINEMODE	0x04
#define KLUDGE_OK	0x03
#define	NO_AUTOKLUDGE	0x02
#define KLUDGE_LINEMODE	0x01
#define NO_LINEMODE	0x00

#include <arpa/telnet.h>

#define NETSLOP 64

#define ttloop(c) while (c) io_drain ()

/* External variables */
extern char	options[256];
extern char	do_dont_resp[256];
extern char	will_wont_resp[256];
extern int	linemode;	/* linemode on/off */
extern int	uselinemode;	/* what linemode to use (on/off) */
extern int	editmode;	/* edit modes in use */
extern int	useeditmode;	/* edit modes to use */
extern int	alwayslinemode;	/* command line option */
extern int	lmodetype;	/* Client support for linemode */
extern int	flowmode;	/* current flow control state */
extern int	restartany;	/* restart output on any character state */
extern int	diagnostic;	/* telnet diagnostic capabilities */
#if defined(AUTHENTICATION)
extern int	auth_level;
extern int      autologin;
#endif

extern slcfun	slctab[NSLC + 1];	/* slc mapping table */

extern char *terminaltype;
extern char *remote_hostname;
extern char *local_hostname;
extern char *login_invocation;
extern char *user_name;

extern int	pty, net;
extern int	SYNCHing;		/* we are in TELNET SYNCH mode */
extern struct telnetd_clocks clocks;
extern char line[];

extern char *xstrdup P((const char *));
extern int argcv_get P((const char *command, const char *delim,
			int *argc, char ***argv));

void io_setup P((void));
int net_has_data P((void));
int net_get_char P((int peek));
void set_neturg P((void));
int net_output_data P((const char *format,...));
int net_output_datalen P((const void *buf, size_t l));
int net_buffer_level P((void));
void io_drain P((void));

int stilloob P((int s));
void ptyflush P((void));
char * nextitem P((char *current));
void netclear P(());
void netflush P(());

int pty_buffer_is_full P((void));
void pty_output_byte P((int c));
void pty_output_datalen P((const void *data, size_t len));
int pty_buffer_level P(());

/* Debugging functions */
extern void printoption P((char *, int));
extern void printdata P((char *, char *, int));
extern void printsub P((int, unsigned char *, int));
extern void debug_output_datalen P((const char *data, size_t len));
extern void debug_output_data P((const char *fmt, ...));

/* TTY functions */
extern void init_termbuf P((void));
extern void set_termbuf P((void));
extern int spcset P((int func, cc_t *valp, cc_t **valpp));
extern void term_send_eof P((void));
extern int term_change_eof P((void));

extern void tty_binaryin P((int));
extern void tty_binaryout P((int));
extern int tty_flowmode P((void));
extern int tty_restartany P((void));
extern int tty_isbinaryin P((void));
extern int tty_isbinaryout P((void));
extern int tty_iscrnl P((void));
extern int tty_isecho P((void));
extern int tty_isediting P((void));
extern int tty_islitecho P((void));
extern int tty_isnewmap P((void));
extern int tty_israw P((void));
extern int tty_issofttab P((void));
extern int tty_istrapsig P((void));
extern int tty_linemode P((void));
extern void tty_rspeed P((int));
extern void tty_setecho P((int));
extern void tty_setedit P((int));
extern void tty_setlinemode P((int));
extern void tty_setlitecho P((int));
extern void tty_setsig P((int));
extern void tty_setsofttab P((int));
extern void tty_tspeed P((int));

extern char * expand_line P((const char *fmt));

/*  FIXME */
extern void _termstat P((void));
extern void add_slc P((char func, char flag, cc_t val));
extern void check_slc P((void));
extern void change_slc P((char func, char flag, cc_t val));

extern void cleanup P((int));
extern void clientstat P((int, int, int));
extern void copy_termbuf P(());
extern void deferslc P((void));
extern void defer_terminit P((void));
extern void do_opt_slc P((unsigned char *, int));
extern void dooption P((int));
extern void dontoption P((int));
extern void edithost P((char *, char *));
extern void fatal P((int, char *));
extern void fatalperror P((int, char *));
extern void get_slc_defaults P((void));
extern void localstat P((void));
extern void flowstat P((void));
extern void netclear P((void));

extern void send_do P((int, int));
extern void send_dont P((int, int));
extern void send_slc P((void));
extern void send_status P((void));
extern void send_will P((int, int));
extern void send_wont P((int, int));
extern void set_termbuf P((void));
extern void start_login P((char *, int, char *));
extern void start_slc P((int));
extern void start_slave P((char *, int, char *));

extern void suboption P((void));
extern void telrcv P((void));
extern void ttloop P((void));
  
extern int end_slc P((unsigned char **));
extern int spcset P((int, cc_t *, cc_t **));
extern int stilloob P((int));
extern int terminit P((void));
extern int termstat P((void));
extern void willoption P((int));
extern void wontoption P((int));

#ifdef	ENCRYPTION
extern void	(*encrypt_output) P((unsigned char *, int));
extern int	(*decrypt_input) P((int));
#endif	/* ENCRYPTION */





