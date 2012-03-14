#define	MAXWAIT		10		/* max seconds to wait for response */

#define	OPT_FLOOD	0x001
#define	OPT_INTERVAL	0x002
#define	OPT_NUMERIC	0x004
#define	OPT_QUIET	0x008
#define	OPT_RROUTE	0x010
#define	OPT_VERBOSE	0x020

#define PING_TIMING(s) (s >= PING_HEADER_LEN)
#define	PING_HEADER_LEN	sizeof (struct timeval)
#define	PING_DATALEN	(64 - PING_HEADER_LEN)	/* default data length */

struct ping_stat
{
  double tmin;	    /* minimum round trip time */
  double tmax;      /* maximum round trip time */
  double tsum;	    /* sum of all times, for doing average */
  double tsumsq;    /* sum of all times squared, for std. dev. */
};



extern unsigned options;
extern PING *ping;
extern int is_root;
extern unsigned long preload;
extern u_char *data_buffer;
extern size_t data_length;

extern int ping_run (PING *ping, int (*finish)());
extern int ping_finish (void);
extern void print_icmp_header (struct sockaddr_in *from,
			      struct ip *ip, icmphdr_t *icmp, int len);
