#ifndef _WHOIS_H
#define _WHOIS_H

/* 6bone referto: extension */
#define REFERTO_FORMAT	"%% referto: whois -h %255s -p %15s %1021[^\n\r]"

/* String sent to RIPE servers - ONLY FIVE CHARACTERS! */
/* Do *NOT* change it if you don't know what you are doing! */
#define IDSTRING "Md4.5"

/* system features */
#ifdef ENABLE_NLS
# ifndef NLS_CAT_NAME
#  define NLS_CAT_NAME   "whois"
# endif
# ifndef LOCALEDIR
#  define LOCALEDIR     "/usr/share/locale"
# endif
#endif

/* NLS stuff */
#ifdef ENABLE_NLS
# include <libintl.h>
# include <locale.h>
# define _(a) (gettext (a))
# ifdef gettext_noop
#  define N_(a) gettext_noop (a)
# else
#  define N_(a) (a)
# endif
#else
# define _(a) (a)
# define N_(a) a
#endif


/* prototypes */
const char *whichwhois(const char *);
const char *whereas(int, struct as_del []);
char *queryformat(const char *, const char *, const char *);
void do_query(const int, const char *);
const char *query_crsnic(const int, const char *);
int openconn(const char *, const char *);
void closeconn(const int);
void usage(void);
void sighandler(int);
unsigned long myinet_aton(const char *);
int domcmp(const char *, const char *);
int domfind(const char *, const char *[]);

void err_quit(const char *,...);
void err_sys(const char *,...);


/* flags for RIPE-like servers */
const char *ripeflags="acFlLmMrRSx";
const char *ripeflagsp="gisTtvq";

/* Configurable features */

/* 6bone referto: support */
#define EXT_6BONE

/* Always hide legal disclaimers */
#undef ALWAYS_HIDE_DISCL

/* Default server */
#define DEFAULTSERVER   "whois.internic.net"

#endif /* _WHOIS_H */
