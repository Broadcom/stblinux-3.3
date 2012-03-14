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

#include <telnetd.h>

#if defined(HAVE_TERMIOS_H)
typedef struct termios TERMDESC;
# define _term_getattr tcgetattr
# define _term_setattr(fd, tp) tcsetattr (fd, TCSANOW, tp)

#elif defined(HAVE_TERMIO_H)
typedef struct termio TERMDESC;
# define _term_getattr tcgetattr
# define _term_setattr(fd, tp) tcsetattr (fd, TCSANOW, tp)

#else

#define IOCTL_INTERFACE

typedef struct 
{
  struct sgttyb sg;
  struct tchars tc;
  struct ltchars ltc;
  int state;
  int lflags;
#define termdesc_eofc   tc.t_eofc
#define termdesc_erase  tc.sg.sg_erase
#define termdesc_kill   sg.sg_kill
#define termdesc_ip     tc.t_intrc
#define termdesc_abort  tc.t_quitc
#define termdesc_xon    tc.t_startc
#define termdesc_xoff   tc.t_stopc
#define termdesc_ao     ltc.t_flushc
#define termdesc_susp   ltc.t_suspc
#define termdesc_ew     ltc.t_werasc
#define termdesc_rp     ltc.t_rprntc
#define termdesc_lnext  ltc.t_lnextc
#define termdesc_forw1  tc.t_brkc
} TERMDESC;

# define cfsetospeed(tp, val) (tp)->sg.sg_ospeed = (val)
# define cfsetispeed(tp, val) (tp)->sg.sg_ispeed = (val)
# define cfgetospeed(tp)      (tp)->sg.sg_ospeed
# define cfgetispeed(tp)      (tp)->sg.sg_ispeed

int
_term_getattr (int fd, TERMDESC *tp)
{
  ioctl (fd, TIOCGETP, (char *)&tp->sg);
  ioctl (fd, TIOCGETC, (char *)&tp->tc);
  ioctl (fd, TIOCGLTC, (char *)&tp->ltc);
# ifdef	TIOCGSTATE
  ioctl (fd, TIOCGSTATE, (char *)&tp->state);
# endif
  return 0;
}
#endif

TERMDESC termbuf, termbuf2;

#ifdef IOCTL_INTERFACE

void
_term_setattr (int fd, TERMDESC *tp)
{
  ioctl (fd, TIOCSETN, (char *)&tp->sg);
  ioctl (fd, TIOCSETC, (char *)&tp->tc);
  ioctl (fd, TIOCSLTC, (char *)&tp->ltc);
  ioctl (fd, TIOCLSET, &tp->lflags);
}

void
term_send_eof ()
{
  /*nothing*/
}

int
term_change_eof ()
{
  return 0;
}

int
tty_linemode()
{
  return termbuf.state & TS_EXTPROC;
}

void
tty_setlinemode (int on)
{
#ifdef	TIOCEXT
  set_termbuf ();
  ioctl (pty, TIOCEXT, (char *)&on);
  init_termbuf ();
#else	/* !TIOCEXT */
# ifdef	EXTPROC
  if (on)
    termbuf.c_lflag |= EXTPROC;
  else
    termbuf.c_lflag &= ~EXTPROC;
# endif
#endif	/* TIOCEXT */
}

int
tty_isecho ()
{
  return termbuf.sg.sg_flags & ECHO;
}

int
tty_flowmode ()
{
  return ((termbuf.tc.t_startc) > 0 && (termbuf.tc.t_stopc) > 0) ? 1 : 0;
}

int
tty_restartany ()
{
# ifdef	DECCTQ
  return (termbuf.lflags & DECCTQ) ? 0 : 1;
# else
  return -1;
# endif
}

void
tty_setecho(int on)
{
  if (on)
    termbuf.sg.sg_flags |= ECHO|CRMOD;
  else
    termbuf.sg.sg_flags &= ~(ECHO|CRMOD);
}

int
tty_israw ()
{
  return termbuf.sg.sg_flags & RAW;
}

#if defined (AUTHENTICATION) && defined(NO_LOGIN_F) && defined(LOGIN_R)
int
tty_setraw(int on)
{
  if (on)
    termbuf.sg.sg_flags |= RAW;
  else
    termbuf.sg.sg_flags &= ~RAW;
}
#endif

void
tty_binaryin (int on)
{
  if (on)
    termbuf.lflags |= LPASS8;
  else
    termbuf.lflags &= ~LPASS8;
}

void
tty_binaryout (int on)
{
  if (on)
    termbuf.lflags |= LLITOUT;
  else
    termbuf.lflags &= ~LLITOUT;
}

int
tty_isbinaryin ()
{
  return termbuf.lflags & LPASS8;
}

int
tty_isbinaryout ()
{
  return termbuf.lflags & LLITOUT;
}

int
tty_isediting ()
{
  return !(termbuf.sg.sg_flags & (CBREAK|RAW));
}

int
tty_istrapsig()
{
  return !(termbuf.sg.sg_flags&RAW);
}

void
tty_setedit (int on)
{
  if (on)
    termbuf.sg.sg_flags &= ~CBREAK;
  else
    termbuf.sg.sg_flags |= CBREAK;
}

void
tty_setsig (int on)
{
}

int
tty_issofttab ()
{
  return termbuf.sg.sg_flags & XTABS;
}

void
tty_setsofttab (int on)
{
  if (on)
    termbuf.sg.sg_flags |= XTABS;
  else
    termbuf.sg.sg_flags &= ~XTABS;
}

int
tty_islitecho ()
{
  return !(termbuf.lflags & LCTLECH);
}

void
tty_setlitecho (int on)
{
  if (on)
    termbuf.lflags &= ~LCTLECH;
  else
    termbuf.lflags |= LCTLECH;
}

int
tty_iscrnl ()
{
  return termbuf.sg.sg_flags & CRMOD;
}

#else

# define termdesc_eofc    c_cc[VEOF]
# define termdesc_erase   c_cc[VERASE]
# define termdesc_kill    c_cc[VKILL]
# define termdesc_ip      c_cc[VINTR]
# define termdesc_abort   c_cc[VQUIT]
# ifdef	VSTART
#  define termdesc_xon    c_cc[VSTART]
# endif
# ifdef	VSTOP
#  define termdesc_xoff   c_cc[VSTOP]
# endif

# if !defined(VDISCARD) && defined(VFLUSHO)
#  define VDISCARD VFLUSHO
# endif
# ifdef	VDISCARD
#  define termdesc_ao     c_cc[VDISCARD]
# endif

# ifdef VSUSP
#  define termdesc_susp   c_cc[VSUSP]
# endif
# ifdef	VWERASE
#  define termdesc_ew     c_cc[VWERASE]
# endif
# ifdef	VREPRINT
#  define termdesc_rp     c_cc[VREPRINT]
# endif
# ifdef	VLNEXT
#  define termdesc_lnext  c_cc[VLNEXT]
# endif
# ifdef	VEOL
#  define termdesc_forw1  c_cc[VEOL]
# endif
# ifdef	VEOL2
#  define termdesc_forw2  c_cc[VEOL2]
# endif
# ifdef	VSTATUS
#  define termdesc_status c_cc[VSTATUS]
# endif

static cc_t oldeofc = '\004';

void
term_send_eof ()
{
#if VEOF == VMIN
  if (!tty_isediting())
    pty_output_byte (oldeofc);
#endif
}

int
term_change_eof ()
{
#if VEOF == VMIN
  if (!tty_isediting())
    return 1;
  if (slctab[SLC_EOF].sptr)
    oldeofc = *slctab[SLC_EOF].sptr;
  return 0;
#endif
}

int
tty_linemode()
{
#ifdef EXTPROC
  return (termbuf.c_lflag & EXTPROC);
#else
  return 0;		/* Can't ever set it either. */
#endif /* EXTPROC */
}

void
tty_setlinemode (int on)
{
#ifdef	TIOCEXT
  set_termbuf ();
  ioctl (pty, TIOCEXT, (char *)&on);
  init_termbuf ();
#else	/* !TIOCEXT */
# ifdef	EXTPROC
  if (on)
    termbuf.c_lflag |= EXTPROC;
  else
    termbuf.c_lflag &= ~EXTPROC;
# endif
#endif	/* TIOCEXT */
}

int
tty_isecho ()
{
  return termbuf.c_lflag & ECHO;
}

int
tty_flowmode ()
{
  return (termbuf.c_iflag & IXON) ? 1 : 0;
}

int
tty_restartany ()
{
  return (termbuf.c_iflag & IXANY) ? 1 : 0;
}

void
tty_setecho(int on)
{
  if (on)
    termbuf.c_lflag |= ECHO;
  else
    termbuf.c_lflag &= ~ECHO;
}

int
tty_israw ()
{
  return !(termbuf.c_lflag & ICANON);
}

#if defined (AUTHENTICATION) && defined(NO_LOGIN_F) && defined(LOGIN_R)
int
tty_setraw(int on)
{
  if (on)
    termbuf.c_lflag &= ~ICANON;
  else
    termbuf.c_lflag |= ICANON;
}
#endif

void
tty_binaryin (int on)
{
  if (on)
    termbuf.c_iflag &= ~ISTRIP;
  else 
    termbuf.c_iflag |= ISTRIP;
}

void
tty_binaryout (int on)
{
  if (on)
    {
      termbuf.c_cflag &= ~(CSIZE|PARENB);
      termbuf.c_cflag |= CS8;
      termbuf.c_oflag &= ~OPOST;
    }
  else
    {
      termbuf.c_cflag &= ~CSIZE;
      termbuf.c_cflag |= CS7|PARENB;
      termbuf.c_oflag |= OPOST;
    }
}

int
tty_isbinaryin ()
{
  return !(termbuf.c_iflag & ISTRIP);
}

int
tty_isbinaryout ()
{
  return !(termbuf.c_oflag&OPOST);
}

int
tty_isediting ()
{
  return termbuf.c_lflag & ICANON;
}

int
tty_istrapsig()
{
  return termbuf.c_lflag & ISIG;
}

void
tty_setedit (int on)
{
  if (on)
    termbuf.c_lflag |= ICANON;
  else
    termbuf.c_lflag &= ~ICANON;
}

void
tty_setsig (int on)
{
  if (on)
    termbuf.c_lflag |= ISIG;
  else
    termbuf.c_lflag &= ~ISIG;
}

int
tty_issofttab ()
{
# ifdef	OXTABS
  return termbuf.c_oflag & OXTABS;
# endif
# ifdef	TABDLY
  return (termbuf.c_oflag & TABDLY) == TAB3;
# endif
}

void
tty_setsofttab (int on)
{
  if (on)
    {
# ifdef	OXTABS
      termbuf.c_oflag |= OXTABS;
# endif
# ifdef	TABDLY
      termbuf.c_oflag &= ~TABDLY;
      termbuf.c_oflag |= TAB3;
# endif
    }
  else
    {
# ifdef	OXTABS
      termbuf.c_oflag &= ~OXTABS;
# endif
# ifdef	TABDLY
      termbuf.c_oflag &= ~TABDLY;
      termbuf.c_oflag |= TAB0;
# endif
    }
}

int
tty_islitecho ()
{
# ifdef	ECHOCTL
  return !(termbuf.c_lflag & ECHOCTL);
# endif
# ifdef	TCTLECH
  return !(termbuf.c_lflag & TCTLECH);
# endif
# if!defined(ECHOCTL) && !defined(TCTLECH)
  return 0;	/* assumes ctl chars are echoed '^x' */
#endif
}

void
tty_setlitecho (int on)
{
# ifdef	ECHOCTL
  if (on)
    termbuf.c_lflag &= ~ECHOCTL;
  else
    termbuf.c_lflag |= ECHOCTL;
# endif
# ifdef	TCTLECH
  if (on)
    termbuf.c_lflag &= ~TCTLECH;
  else
    termbuf.c_lflag |= TCTLECH;
# endif
}

int
tty_iscrnl ()
{
  return termbuf.c_iflag & ICRNL;
}

#endif

void
init_termbuf ()
{
  _term_getattr (pty, &termbuf);
  termbuf2 = termbuf;
}

#if defined(TIOCPKT_IOCTL)
/*FIXME: Hardly needed*/
void
copy_termbuf ()
{
  int len = 0;
  char *cp = (char*) &termbuf;
  
  while (pty_input_level () > 0)
    {
      if (len >= sizeof (termbuf))
	break;
      *cp++ = pty_get_char (0);
    }
  termbuf2 = termbuf;
}
#endif

void
set_termbuf ()
{
  if (memcmp (&termbuf, &termbuf2, sizeof (termbuf)))
    _term_setattr (pty, &termbuf);
}

/* spcset(func, valp, valpp)
 
   This function takes various special characters (func), and
   sets *valp to the current value of that character, and
   *valpp to point to where in the "termbuf" structure that
   value is kept.
 
   It returns the SLC_ level of support for this function. */

#define	setval(a, b)	*valp = termbuf.a ; \
			*valpp = &termbuf.a ; \
			return b;
#define	defval(a)       *valp = ((cc_t)a); \
                        *valpp = (cc_t *)0; \
			return SLC_DEFAULT;

int
spcset (int func, cc_t *valp, cc_t **valpp)
{
  switch (func)
    {
    case SLC_EOF:
      setval (termdesc_eofc, SLC_VARIABLE);
      
    case SLC_EC:
      setval (termdesc_erase, SLC_VARIABLE);
      
    case SLC_EL:
      setval (termdesc_kill, SLC_VARIABLE);
      
    case SLC_IP:
      setval (termdesc_ip, SLC_VARIABLE|SLC_FLUSHIN|SLC_FLUSHOUT);
      
    case SLC_ABORT:
      setval (termdesc_abort, SLC_VARIABLE|SLC_FLUSHIN|SLC_FLUSHOUT);

    case SLC_XON:
#ifdef termdesc_xon
      setval (termdesc_xon, SLC_VARIABLE);
#else
      defval (0x13);
#endif
      
    case SLC_XOFF:
#ifdef termdesc_xoff
      setval (termdesc_xoff, SLC_VARIABLE);
#else
      defval (0x11);
#endif
      
    case SLC_EW:
#ifdef termdesc_ew
      setval (termdesc_ew, SLC_VARIABLE);
#else
      defval (0);
#endif
      
    case SLC_RP:
#ifdef termdesc_rp
      setval (termdesc_rp, SLC_VARIABLE);
#else
      defval(0);
#endif
      
    case SLC_LNEXT:
#ifdef termdesc_lnext
      setval (termdesc_lnext, SLC_VARIABLE);
#else
      defval (0);
#endif
      
    case SLC_AO:
#ifdef termdesc_ao      
      setval (termdesc_ao, SLC_VARIABLE|SLC_FLUSHOUT);
#else
      defval (0);
#endif
      
    case SLC_SUSP:
#ifdef termdesc_susp
      setval (termdesc_susp, SLC_VARIABLE|SLC_FLUSHIN);
#else
      defval (0);
#endif
      
#ifdef termdesc_forw1
    case SLC_FORW1:
      setval (termdesc_forw1, SLC_VARIABLE);
#endif
      
#ifdef termdesc_forw2
    case SLC_FORW2:
      setval (termdesc_forw2, SLC_VARIABLE);
#endif
      
    case SLC_AYT:
#ifdef termdesc_status
      setval (termdesc_status, SLC_VARIABLE);
#else
      defval (0);
#endif

    case SLC_BRK:
    case SLC_SYNCH:
    case SLC_EOR:
      defval (0);

    default:
      *valp = 0;
      *valpp = 0;
      return SLC_NOSUPPORT;
    }
}

#if B4800 != 4800
#define	DECODE_BAUD
#endif

#ifdef	DECODE_BAUD

/*
 * A table of available terminal speeds
 */
struct termspeeds {
  int	speed;
  int	value;
} termspeeds[] = {
  { 0,     B0 },    { 50,    B50 },   { 75,    B75 },
  { 110,   B110 },  { 134,   B134 },  { 150,   B150 },
  { 200,   B200 },  { 300,   B300 },  { 600,   B600 },
  { 1200,  B1200 }, { 1800,  B1800 }, { 2400,  B2400 },
  { 4800,   B4800 },
#ifdef	B7200
  { 7200,  B7200 },
#endif
  { 9600,   B9600 },
#ifdef	B14400
  { 14400,  B14400 },
#endif
#ifdef	B19200
  { 19200,  B19200 },
#endif
#ifdef	B28800
  { 28800,  B28800 },
#endif
#ifdef	B38400
  { 38400,  B38400 },
#endif
#ifdef	B57600
  { 57600,  B57600 },
#endif
#ifdef	B115200
  { 115200, B115200 },
#endif
#ifdef	B230400
  { 230400, B230400 },
#endif
  { -1,     0 }
};
#endif	/* DECODE_BAUD */

void
tty_tspeed (int val)
{
#ifdef	DECODE_BAUD
  struct termspeeds *tp;

  for (tp = termspeeds; (tp->speed != -1) && (val > tp->speed); tp++)
    ;
  if (tp->speed == -1)	/* back up to last valid value */
    --tp;
  val = tp->value;
#endif	
  cfsetospeed (&termbuf, val);
}

void
tty_rspeed (int val)
{
#ifdef	DECODE_BAUD
  struct termspeeds *tp;

  for (tp = termspeeds; (tp->speed != -1) && (val > tp->speed); tp++)
    ;
  if (tp->speed == -1)	/* back up to last valid value */
    --tp;
  val = tp->value;
#endif
  cfsetispeed (&termbuf, val);
}


 

