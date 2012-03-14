/*
 * termcap.c	Replacement for the GNU Emacs termcap routines.
 *		These do more error checking, like preventing a loop
 *		with the tc= capability and buffer overflow.
 *		Also, these routines can stuff a whole lot more in
 *		one buffer because duplicate capabilities are eliminated.
 *
 * Version:	1.3 15-Apr-1996 MvS (miquels@cistron.nl)
 *
 * Changelog    1.0 ??-Sep-1994 MvS Initial version
 *		1.1 20-Oct-1994 MvS (Broken) version for libc 4.5.19
 *		1.2 13-Dec-1994 MvS Fixed '\' escapes, disabled the
 *		                    multiple char. capability names.
 *		1.3 15-Apr-1996 MvS Ignore entries starting with `.'
 *		                    Preprocess escaped entries
 *
 *		Copyright (C) Miquel van Smoorenburg 1994,1995,1996
 *		This code falls under the LGPL.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <termcap.h>

/* Maximum terminal type includes (tc=) */
#define MAX_TERMS 16

/* Escape sequences we know about. */
static char *escapes = "E\033r\rn\nb\bt\tf\f\\\\";

/* Pointer for tgetstr() et al */
static char *term_entry;
static int is_malloced;

/* Table with speeds for padding. */
static short speeds[] = {
  0, 50, 75, 110, 134, 150, -3, -6, -12, -18,
  -20, -24, -36, -48, -72, -96, -192, -384, -576, -1152
};

/* Some (undocumented?) global variables. */
speed_t ospeed;
int tputs_baud_rate;
char PC;
#if !defined(TGETENT_BUFSIZE)
#define	TGETENT_BUFSIZE	1536	/* XXX used to be 1024 */
#endif
int tgetent_bufsize = TGETENT_BUFSIZE;

/* We store a terminal description in a linked list. */
struct tc_ent {
  struct tc_ent *next;
  char cap[1];
};

/* Safe malloc. */
static void *xmalloc(int len)
{
  void *x;

  if ((x = malloc(len)) != NULL) return(x);
  write(2, "Virtual memory exhausted.\n", 26);
  exit(1);
}

/* Safe strdup() */
static char *strsave(char *s)
{
  char *x;

  x = xmalloc(strlen(s) + 1);
  strcpy(x, s);
  return(x);
}

/* Add a copy of a string to the end of a list */
static int add_to_list(char **list, char *s, int max_entries)
{
  int i;
  int done = 0;

  if (! list)
	return(0);

  for(i = 0; i < max_entries; i++)
	if (list[i] == NULL) {
		list[i] = strsave (s);
		list[i + 1] = NULL;
		done = 1;
		break;
	} else if (strcmp (s, list[i]) == 0) {
		done = 1;
		break;
	}
  return(done);
}

/*
 *	Try to shrink a capability.
 */
static void shrinkcap(char *r, char *s)
{
  char *start, *sp;
  int len, c, i;

  /* Translate escaped characters and hat-notation. */
  while((c = *s++)) {
	start = s - 1;
	if ((c == '\\') && (*s)) {

		/* Escaped character. */
		c = *s++;

		if (c >= '0' && c <= '7') {
			/* Octal number. */
			c -= '0';
			i = 0;
			while(*s >= '0' && *s <= '7' && ++i < 3) {
				c = (c * 8) + (*s - '0');
				s++;
			}
		} else {
			/* \r or \n or whatever. */
			for(sp = escapes; *sp; sp += 2)
				if (c == *sp) {
					c = *++sp;
					break;
				}
		}
	} else if ((c == '^') && (*s))
		/* Hat notation. */
		c = *s++ & 0x1f;

	/* See if we want to translate. */
	if ((c & 0x7f) > 31 && c != ':')
		*r++ = c;
	else {
		len = s - start;
		strncpy(r, start, len);
		r += len;
	}
  }
  *r++ = 0;
}

/* Build a linked list with capabilities. */
static void build_list(struct tc_ent **listp, char *buf, char **term_list)
{
  struct tc_ent *i, *last = NULL, *list = *listp;
  char *s, *sp, *bp;
  int len;

  /* Skip name field. */
  for(sp = buf; *sp && *sp != ':'; sp++)
	;
  if (*sp) *sp++ = 0;

  /* Extract capabilities one-by-one. */
  while(*sp) {
	/* Find end of field. */
	bp = sp;
	while(*sp && *sp != ':') {
		/* Allow escaped ':' */
		if (*sp == '\\' && sp[1] == ':') sp++;
		sp++;
	}
	if (*sp) *sp++ = 0;

	/* Check for empty field or comments. */
	while(*bp == ' ' || *bp == '\t' || *bp == '\n') bp++;
	if (*bp == 0 || *bp == ':' || *bp == '.') continue;

	/* Is this the "tc" capability? */
	if (strncmp(bp, "tc=", 3) == 0) {
		add_to_list(term_list, bp + 3, MAX_TERMS);
		continue;
	}

	/* Find name of capability. */
	if ((s = strchr(bp, '=')) != NULL)
		len = s - bp;
	else if ((s = strchr(bp, '@')) != NULL)
		len = s - bp;
	else if ((s = strchr(bp, '#')) != NULL)
		len = s - bp;
	else len = strlen(bp);
	if (len == 0) continue;

	/* See if the capability is already in the list. */
	if (list) {
		for(i = list; i; i = i->next) {
			last = i;
			if (strncmp(i->cap, bp, len) == 0) break;
		}
	}
	if (list != NULL && i != NULL) continue;

	/* Add capability to the list. */
	i = (struct tc_ent *)xmalloc(sizeof(struct tc_ent) + strlen(bp));
	if (i == NULL) break;
	shrinkcap(i->cap, bp);
	i->next = NULL;
	if (list == NULL)
		list = i;
	else
		last->next = i;
  }
  /* Done. */
  *listp = list;
}

/* Add OR change a capability (hardcoded for li# and co#) */
static void add_list(struct tc_ent **list, char *cap)
{
  struct tc_ent *prev, *new, *l;

  /* Walk through the list. */
  prev = NULL;
  for(l = *list; l; l = l->next) {
	if (strncmp(l->cap, cap, 3) == 0) {

		/* Found: modify in-place. */
		new = xmalloc(sizeof(struct tc_ent) + strlen(cap));
		strcpy(new->cap, cap);
		new->next = l->next;
		if (prev)
			prev->next = new;
		else
			*list = new;
		free(l);
		l = new;
		break;
	}
	prev = l;
  }
  if (l != NULL) return;

  /* Not found, add to the end of the list. */
  new = xmalloc(sizeof(struct tc_ent) + strlen(cap));
  strcpy(new->cap, cap);
  new->next = NULL;
  if (prev)
	prev->next = new;
  else
	*list = new;
}

/* Convert a number to ASCII */
static char *_itoa(int num, char *buf)
{
  char *sp = buf + 16;

  *--sp = 0;
  do {
	*--sp = (num % 10) + '0';
	num /= 10;
  } while(num);
  return(sp);
}

/* Adjust lines and columns by doing a TIOCGWINSZ */
static void adjust_lines_cols(struct tc_ent **l)
{
  struct winsize ws;
  char buf[16];
  char num[16];

  /* Get and check window size. */
  if (ioctl(0, TIOCGWINSZ, &ws) < 0 || !ws.ws_row || !ws.ws_col)
	return;

  /* Fill in li# and co# */
  strcpy(buf, "li#");
  strcpy(buf + 3, _itoa(ws.ws_row, num));
  add_list(l, buf);

  strcpy(buf, "co#");
  strcpy(buf + 3, _itoa(ws.ws_col, num));
  add_list(l, buf);
}

/* See if strings contains terminal name. */
static int tc_comp(char *line, char *term)
{
  char *sp, *bp;
  int found = 0, x;
  int len = strlen(term);

  bp = sp = line;
  x = *bp;
  while(x && x != ':' && x != '\n') {
	/* Find the end of this description. */
	while(*sp && *sp != ':' && *sp != '|' && *sp != '\n')
		sp++;
	if (len == (sp - bp) && strncmp(term, bp, len) == 0) {
		/* Found it! */
		found = 1;
		break;
	}
	x = *sp++;
	bp = sp;
  }
  return(found);
}


/* Load a specific terminal. */
static char *get_one_entry(FILE *tfp, char *term)
{
  char line[256];
  int status = 0;
  char *sp;
  char buf[4096];
  char *bufp = buf;

  if (term == NULL) return(NULL);

  /* Start at beginning. */
  rewind(tfp);

  /* Read line by line. */
  while(fgets(line, 256, tfp) != NULL) {
	if (line[0] == '#') continue;

	/* See if this is what we're looking for. */
	if (status == 0 && tc_comp(line, term) == 0) continue;
	status = 1;

	/* We are reading a description here. */
	for(sp = line; *sp == ' ' || *sp == '\t'; sp++)
		;

	/* Add the rest until nl to the buffer. */
	while(*sp && *sp != '\n') {
		if (*sp == '\\' && (*(sp + 1) == '\n')) break;
		*bufp++ = *sp++;
		if (bufp - buf > 4092) {
			/* Buffer full, quit. */
			*sp = '\n';
			break;
		}
	}
	if (*sp == '\n') break;
  }
  /* Save buffer to malloced area. */
  *bufp++ = 0;
  if (status == 0) return(NULL);
  if ((sp = xmalloc(bufp - buf)) == NULL) return(sp);
  memcpy(sp, buf, bufp - buf);
  return(sp);
}


/* Read terminal description. */
static char *tc_read(struct tc_ent **tcp, char *term)
{
  FILE *fp;
  char *sp, *tc;
  char *desc = NULL;
  char *tc_file = "/etc/termcap";
  struct tc_ent *l = NULL;
  int index;
  int tc_set = 0;
  char *term_list[MAX_TERMS + 1];

  *tcp = NULL;

  /* See if we have a TERMCAP environment variable. */
  if ((tc = getenv("TERMCAP")) != NULL) {
	if (*tc == '/')
	{
		tc_file = tc;
		tc_set=1;
	}
	else {
		/* check if TERMCAP is term */
		if (tc_comp(tc, term)) {
#if DEBUG
			printf("Using TERMCAP\n");
#endif
			/* Just read the TERMCAP variable. */
			sp = strsave(tc);
			build_list(&l, sp, NULL);
			*tcp = l;
			return(sp);
		}
	}
  }

#if DEBUG
  printf("Using file %s\n", tc_file);
#endif

  if(tc_set)
  {
  	setfsuid(getuid());
  	setfsgid(getgid());
  }
  /* Now read the termcap file. */
  fp = fopen(tc_file, "r");
  
  if(tc_set)
  {
  	setfsuid(geteuid());
  	setfsgid(getegid());
  }
  if (fp == NULL)
  	return(NULL);

  desc = term;
  term_list[0] = term;
  term_list[1] = NULL;
  for(index = 0; (index < MAX_TERMS) && term_list[index]; index++) {
#if DEBUG
	printf("LOOKUP: term %s\n", term_list[index]);
#endif
	sp = get_one_entry(fp, term_list[index]);
	if (sp == NULL) break;
	build_list(&l, sp, term_list);
	free (sp);
  }
  fclose(fp);

  for(index = 1; term_list[index] != NULL; index++)
    free (term_list[index]);

  /* Done. */
  *tcp = l;
#if DEBUG
  printf(">> tc_read done, desc = %s\n", desc);
#endif
  return(desc ? desc : "");
}


/* The tgetent function. */
int tgetent(void *buffer, const char *term)
{
  char *s;
  struct tc_ent *l, *i, *next;
  char *bp, *sp = (char *)buffer;
  int len, count, maxlen;

  /* Find the termcap entry. */
  s = tc_read(&l, (char *)term);

  /* Return -1 if we can't open the termcap file. */
  if (s == NULL) return(-1);

  /* Return 0 if the entry is not present. */
  if (l == NULL)
  {
    /* For compatibility with programs like `less' that want to
       put data in the termcap buffer themselves as a fallback.  */
    if (buffer)
    {
      /* Free old buffer if possible. */
      if (is_malloced && term_entry) free(term_entry);
      is_malloced = 0;
      term_entry = buffer;
    }
    return(0);
  }

  /* Adjust lines and columns. */
  adjust_lines_cols(&l);

  /* Free old buffer if possible. */
  if (is_malloced && term_entry) free(term_entry);

  /* Do we already have a buffer? */
  /* No we don't. I don't care if they pass us a few gigabytes
   * of storage. We're ignoring it. */
  /* if (sp) {
   *	maxlen = tgetent_bufsize - 1;
   *	is_malloced = 0;
   * } else { */
	/* Count how many bytes we need. */
	count = strlen(s) + 1;
	for(i = l; i; i = i->next)
		count += strlen(i->cap) + 1;
	count++;
	
	/* Malloc this amount. */
  	sp = xmalloc(count);
	maxlen = count + 32; /* Just a lot. */
	is_malloced = 1;
/*  } */

  /* Save buffer into static variable (yuk!) */
  term_entry = sp;

  /* First copy the description to the buffer. */
  count = 0;
  for(bp = s; *bp; bp++) {
	*sp++ = *bp;
	count++;
	if (count >= maxlen-1) {
		write(2, "tgetent: warning: termcap entry too long\n", 41);
		break;
	}
  }
  *sp++ = ':';
  count++;

  /* And now the capabilities. */
  for(i = l; i; i = next) {

	/* Is this a 'skip' capability? */
	len = strlen(i->cap);
	if (strchr(i->cap, '=') == NULL && i->cap[len-1] == '@') {
		next = i->next;
		free(i);
		continue;
	}

	/* Check for buffer overflow. */
	count += len + 1;
	if (count >= maxlen) {
		write(2, "tgetent: warning: termcap entry too long\n", 41);
		break;
	}

	/* Add capability to buffer. */
	for(bp = i->cap; *bp; bp++)
		*sp++ = *bp;
	*sp++ = ':';

	/* Free space. */
	next = i->next;
	free(i);
  }
  *sp = 0;

  return(1);
}

/* Generic "find capability" routine. */
static char *find_cap(char *bp, const char *cap, char sep)
{
#if MULTIPLE_CHAR_CAP /* Disabled because it breaks some programs */
  int len = strlen(cap);

  if (len == 2) {
#endif
	/* Normal case, do fast lookup. */
	while(*bp) {
		if (bp[0] == ':' &&
		    bp[1] == cap[0] &&
		    bp[2] == cap[1] &&
		    bp[3] == sep) return(bp + 4);
		bp++;
	}
	return(NULL);
#if MULTIPLE_CHAR_CAP
  }
  /* Longer string, use slow lookup. */
  while(*bp) {
	if (bp[0] == ':' && bp[len+1] == sep && strncmp(bp+1, cap, len) == 0)
		return(bp + len + 2);
	bp++;
  }
  return(NULL);
#endif
}

/* Find a number capability. */
int tgetnum(const char *cap)
{
  char *s;

  s = find_cap(term_entry, cap, '#');
  return(s ? atoi(s) : -1);
}

/* Find a boolean capability. */
int tgetflag(const char *cap)
{
  return(find_cap(term_entry, cap, ':') ? 1 : 0);
}

/* Find a string capability. */
char *tgetstr(const char *cap, char **bufp)
{
  char *s;
  char *sp, *r, *ret;
  int c, i;

  s = find_cap(term_entry, cap, '=');
  if (s == NULL) return(s);

  /* Yawn. Let's ignore bufp, too. */
  /* Where to put the result. */
  /* if (bufp == (char **)NULL) { */
	for(sp = s; *sp != ':' && *sp; sp++) {
		if (*sp =='\\' && sp[1] == ':') sp++;
	}
	ret = xmalloc(sp - s + 1);
  /* } else
   *	ret = *bufp;
   */
  r = ret;

  /* Translate escaped characters and hat-notation. */
  while((c = *s++) && c != ':') {
	if (c == '\\') {

		/* Escaped character. */
		c = *s++;

		if (c >= '0' && c <= '7') {
			/* Octal number. */
			c -= '0';
			i = 0;
			while(*s >= '0' && *s <= '7' && ++i < 3) {
				c = (c * 8) + (*s - '0');
				s++;
			}
		} else {
			/* \r or \n or whatever. */
			for(sp = escapes; *sp; sp += 2)
				if (c == *sp) {
					c = *++sp;
					break;
				}
		}
	} else if (c == '^')
		/* Hat notation. */
		c = *s++ & 0x1f;
	*r++ = c;
  }
  *r++ = 0;

  /* Do we need to update bufp? */
 /*  if (bufp) *bufp = r; */

  return(ret);
}

/* Output string with padding - stolen from GNU termcap.c :) */
void tputs(const char *str, int nlines, int (*outfun)(int))
{
  int padcount = 0;
  int speed, spd;

  /* Safety check. */
  if (!str) return;

  /* Try to get output speed. */
  spd = ospeed;
#ifdef B38400
  /* We can't usually trust the speed constants > 38400. */
  if (ospeed > B38400) spd = B38400;
#endif
  if (spd == 0)
	speed = tputs_baud_rate;
  else
	speed = speeds[spd];

  /* Read padding information. */
  while (*str >= '0' && *str <= '9') {
      padcount += *str++ - '0';
      padcount *= 10;
  }
  if (*str == '.') {
      str++;
      padcount += *str++ - '0';
  }
  if (*str == '*') {
      str++;
      padcount *= nlines;
  }

  /* Now output the capability string. */
  while (*str)
	(*outfun) (*str++);

  /* Do we need padding? */
  if (padcount == 0) return;

  /* padcount is now in units of tenths of msec.  */
  padcount *= speeds[ospeed];
  padcount += 500;
  padcount /= 1000;
  if (speeds[ospeed] < 0)
	padcount = -padcount;
  else {
	padcount += 50;
	padcount /= 100;
  }

  /* And output the pad character. */
  while (padcount-- > 0)
	(*outfun) (PC);
}


#ifdef TEST
/*ARGSUSED*/
int main(int argc, char **argv)
{
  char buf[TGETENT_BUFSIZE];
  char *s;
  char *ts;

  if (tgetent(buf, argv[1]) != 1) exit(1);

  printf("%s\n\n", term_entry);

  ts = tgetstr("Sf", NULL);
  if (ts == NULL) ts = "not found";
  for(s = ts; s && *s; s++) if (*s == '\033') *s = '*';

  s = tgetstr("cm", NULL);
  if (s && *s == '\033') *s = '?';

  printf("tgetflag(li) = %d\n", tgetflag("li"));
  printf("tgetflag(mi) = %d\n", tgetflag("mi"));
  printf("tgetstr(cm) = [%s]\n", s);
  printf("tgetstr(ks) = [%s]\n", tgetstr("ks", NULL));
  printf("tgetnum(li) = %d\n", tgetnum("li"));
  printf(">> arg res(ts, 5, 5) = %s\n", tgoto(ts, 5, 5));
  return(0);
}
#endif
