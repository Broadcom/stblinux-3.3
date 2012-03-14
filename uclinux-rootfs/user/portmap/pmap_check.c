 /*
  * pmap_check - additional portmap security.
  * 
  * Always reject non-local requests to update the portmapper tables.
  * 
  * Refuse to forward mount requests to the nfs mount daemon. Otherwise, the
  * requests would appear to come from the local system, and nfs export
  * restrictions could be bypassed.
  * 
  * Refuse to forward requests to the nfsd process.
  * 
  * Refuse to forward requests to NIS (YP) daemons; The only exception is the
  * YPPROC_DOMAIN_NONACK broadcast rpc call that is used to establish initial
  * contact with the NIS server.
  * 
  * Always allocate an unprivileged port when forwarding a request.
  * 
  * If compiled with -DCHECK_PORT, require that requests to register or
  * unregister a privileged port come from a privileged port. This makes it
  * more difficult to replace a critical service by a trojan. Also, require
  * that requests to set/unset the NFSD port come form a privileged port.
  * 
  * If compiled with -DHOSTS_ACCESS, reject requests from hosts that are not
  * authorized by the /etc/hosts.{allow,deny} files. The local system is
  * always treated as an authorized host. The access control tables are never
  * consulted for requests from the local system, and are always consulted
  * for requests from other hosts. Access control is based on IP addresses
  * only; attempts to map an address to a host name might cause the
  * portmapper to hang.
  * 
  * Author: Wietse Venema (wietse@wzv.win.tue.nl), dept. of Mathematics and
  * Computing Science, Eindhoven University of Technology, The Netherlands.
  */

#ifndef lint
static char sccsid[] = "@(#) pmap_check.c 1.8 96/07/07 10:49:10";
#endif
#include <unistd.h>
#include <string.h>
#include <rpc/rpc.h>
#include <rpc/pmap_prot.h>
#include <syslog.h>
#include <netdb.h>
#include <pwd.h>
#include <sys/types.h>
#include <sys/signal.h>
#ifdef SYSV40
#include <netinet/in.h>
#include <rpc/rpcent.h>
#endif

extern char *inet_ntoa();

#include "pmap_check.h"

/* Explicit #defines in case the include files are not available. */

#define NFSPROG		((u_long) 100003)
#define MOUNTPROG	((u_long) 100005)
#define	YPXPROG		((u_long) 100069)
#define YPPROG          ((u_long) 100004)
#define YPPROC_DOMAIN_NONACK ((u_long) 2)
#define MOUNTPROC_MNT	((u_long) 1)
#define NFS_PORT	2049

static void logit();
static void toggle_verboselog();
int     verboselog = 0;
int     allow_severity = LOG_INFO;
int     deny_severity = LOG_WARNING;

/* A handful of macros for "readability". */

/* coming from libwrap.a (tcp_wrappers) */
extern int hosts_ctl(char *daemon, char *name, char *addr, char *user);

#define reserved_port(p) (IPPORT_RESERVED/2 < (p) && (p) < IPPORT_RESERVED)

#define unreserved_port(p) (IPPORT_RESERVED <= (p) && (p) != NFS_PORT)

#define	legal_port(a,p) \
  (reserved_port(ntohs((a)->sin_port)) || unreserved_port(p))

#define log_bad_port(addr, proc, prog) \
  logit(deny_severity, addr, proc, prog, ": request from unprivileged port")

#define log_bad_host(addr, proc, prog) \
  logit(deny_severity, addr, proc, prog, ": request from unauthorized host")

#define log_bad_owner(addr, proc, prog) \
  logit(deny_severity, addr, proc, prog, ": request from non-local host")

#define	log_no_forward(addr, proc, prog) \
  logit(deny_severity, addr, proc, prog, ": request not forwarded")

#define log_client(addr, proc, prog) \
  logit(allow_severity, addr, proc, prog, "")

#ifdef HOSTS_ACCESS
static int
good_client(addr)
struct sockaddr_in *addr;
{
    struct hostent *hp;
    char **sp;
    char *tmpname;

    /* Check the IP address first. */
    if (hosts_ctl("portmap", "", inet_ntoa(addr->sin_addr), ""))
	return 1;

    /* Check the hostname. */
    hp = gethostbyaddr ((const char *) &(addr->sin_addr),
			sizeof (addr->sin_addr), AF_INET);

    if (!hp)
	return 0;

    /* must make sure the hostent is authorative. */
    tmpname = alloca (strlen (hp->h_name) + 1);
    strcpy (tmpname, hp->h_name);
    hp = gethostbyname(tmpname);
    if (hp) {
	/* now make sure the "addr->sin_addr" is on the list */
	for (sp = hp->h_addr_list ; *sp ; sp++) {
	    if (memcmp(*sp, &(addr->sin_addr), hp->h_length)==0)
		break;
	}
	if (!*sp)
	    /* it was a FAKE. */
	    return 0;
    }
    else
	   /* never heard of it. misconfigured DNS? */
	   return 0;

   /* Check the official name first. */
   if (hosts_ctl("portmap", "", hp->h_name, ""))
	return 1;

   /* Check aliases. */
   for (sp = hp->h_aliases; *sp ; sp++) {
	if (hosts_ctl("portmap", "", *sp, ""))
	    return 1;
   }

   /* No match */
   return 0;
}
#endif

/* check_startup - additional startup code */

void    check_startup()
{

    /*
     * Give up root privileges so that we can never allocate a privileged
     * port when forwarding an rpc request.
     *
     * Fix 8/3/00 Philipp Knirsch: First lookup our rpc user. If we find it,
     * switch to that uid, otherwise simply resue the old bin user and print
     * out a warning in syslog.
     */

    struct passwd *pwent;

    pwent = getpwnam("rpc");
    if (pwent == NULL) {
        syslog(LOG_WARNING, "user rpc not found, reverting to user bin");
        if (setuid(1) == -1) {
            syslog(LOG_ERR, "setuid(1) failed: %m");
            exit(1);
        }
    }
    else {
        if (setuid(pwent->pw_uid) == -1) {
            syslog(LOG_WARNING, "setuid() to rpc user failed: %m");
            if (setuid(1) == -1) {
                syslog(LOG_ERR, "setuid(1) failed: %m");
                exit(1);
            }
        }
    }

    (void) signal(SIGINT, toggle_verboselog);
}

/* check_default - additional checks for NULL, DUMP, GETPORT and unknown */

int
check_default(addr, proc, prog)
struct sockaddr_in *addr;
u_long  proc;
u_long  prog;
{
#ifdef HOSTS_ACCESS
    if (!(from_local(addr) || good_client(addr))) {
	log_bad_host(addr, proc, prog);
	return (FALSE);
    }
#endif
    if (verboselog)
	log_client(addr, proc, prog);
    return (TRUE);
}

/* check_privileged_port - additional checks for privileged-port updates */
int
check_privileged_port(addr, proc, prog, port)
struct sockaddr_in *addr;
u_long  proc;
u_long  prog;
u_long  port;
{
#ifdef CHECK_PORT
    if (!legal_port(addr, port)) {
	log_bad_port(addr, proc, prog);
	return (FALSE);
    }
#endif
    return (TRUE);
}

/* check_setunset - additional checks for update requests */

#ifdef LOOPBACK_SETUNSET
int
check_setunset(xprt, ludp_xprt, ltcp_xprt, proc, prog, port)
SVCXPRT *xprt;
SVCXPRT *ludp_xprt;
SVCXPRT *ltcp_xprt;
u_long  proc;
u_long  prog;
u_long  port;
{
    struct sockaddr_in *addr = svc_getcaller(xprt);

    if (xprt != ludp_xprt && xprt != ltcp_xprt) {
#ifdef HOSTS_ACCESS
	(void) good_client(addr);		/* because of side effects */
#endif
	log_bad_owner(addr, proc, prog);
	return (FALSE);
    }
    if (port && !check_privileged_port(addr, proc, prog, port))
	return (FALSE);
    if (verboselog)
	log_client(addr, proc, prog);
    return (TRUE);
}

#else
int
check_setunset(addr, proc, prog, port)
struct sockaddr_in *addr;
u_long  proc;
u_long  prog;
u_long  port;
{
    if (!from_local(addr)) {
#ifdef HOSTS_ACCESS
	(void) good_client(addr);		/* because of side effects */
#endif
	log_bad_owner(addr, proc, prog);
	return (FALSE);
    }
    if (port && !check_privileged_port(addr, proc, prog, port))
	return (FALSE);
    if (verboselog)
	log_client(addr, proc, prog);
    return (TRUE);
}

#endif

/* check_callit - additional checks for forwarded requests */
int
check_callit(addr, proc, prog, aproc)
struct sockaddr_in *addr;
u_long  proc;
u_long  prog;
u_long  aproc;
{
#ifdef HOSTS_ACCESS
    if (!(from_local(addr) || good_client(addr))) {
	log_bad_host(addr, proc, prog);
	return (FALSE);
    }
#endif
    if (prog == PMAPPROG || prog == NFSPROG || prog == YPXPROG ||
	(prog == MOUNTPROG && aproc == MOUNTPROC_MNT) ||
	(prog == YPPROG && aproc != YPPROC_DOMAIN_NONACK)) {
	log_no_forward(addr, proc, prog);
	return (FALSE);
    }
    if (verboselog)
	log_client(addr, proc, prog);
    return (TRUE);
}

/* toggle_verboselog - toggle verbose logging flag */

static void toggle_verboselog(sig)
int     sig;
{
    (void) signal(sig, toggle_verboselog);
    verboselog = !verboselog;
}

/* logit - report events of interest via the syslog daemon */

static void logit(severity, addr, procnum, prognum, text)
int     severity;
struct sockaddr_in *addr;
u_long  procnum;
u_long  prognum;
char   *text;
{
    char   *procname;
    char    procbuf[4 * sizeof(u_long)];
    char   *progname;
    char    progbuf[4 * sizeof(u_long)];
    struct rpcent *rpc;
    struct proc_map {
	u_long  code;
	char   *proc;
    };
    struct proc_map *procp;
    static struct proc_map procmap[] = {
	{ PMAPPROC_CALLIT, "callit"},
	{ PMAPPROC_DUMP, "dump"},
	{ PMAPPROC_GETPORT, "getport"},
	{ PMAPPROC_NULL, "null"},
	{ PMAPPROC_SET, "set"},
	{ PMAPPROC_UNSET, "unset"},
	{ 0, 0},
    };

    /*
     * Fork off a process or the portmap daemon might hang while
     * getrpcbynumber() or syslog() does its thing.
     */

    if (fork() == 0) {

	/* Try to map program number to name. */

	if (prognum == 0) {
	    progname = "";
	} else if ((rpc = getrpcbynumber((int) prognum))) {
	    progname = rpc->r_name;
	} else {
	    sprintf(progname = progbuf, "%lu", prognum);
	}

	/* Try to map procedure number to name. */

	for (procp = procmap; procp->proc && procp->code != procnum; procp++)
	     /* void */ ;
	if ((procname = procp->proc) == 0)
	    sprintf(procname = procbuf, "%lu", (u_long) procnum);

	/* Write syslog record. */

	syslog(severity, "connect from %s to %s(%s)%s",
	       inet_ntoa(addr->sin_addr), procname, progname, text);
	exit(0);
    }
}
