 /*
  * pmap_set - set portmapper table from data produced by pmap_dump
  * 
  * Author: Wietse Venema (wietse@wzv.win.tue.nl), dept. of Mathematics and
  * Computing Science, Eindhoven University of Technology, The Netherlands.
  */

#ifndef lint
static char sccsid[] = "@(#) pmap_set.c 1.2 96/07/06 23:06:23";
#endif

#include <stdio.h>
#include <sys/types.h>
#ifdef SYSV40
#include <netinet/in.h>
#endif
#include <rpc/rpc.h>
#include <rpc/pmap_clnt.h>

int parse_line(char *buf, u_long *, u_long *, int *, unsigned *); 

int
main(argc, argv)
int     argc;
char  **argv;
{
    char    buf[BUFSIZ];
    u_long  prog;
    u_long  vers;
    int     prot;
    unsigned port;

    while (fgets(buf, sizeof(buf), stdin)) {
	if (parse_line(buf, &prog, &vers, &prot, &port) == 0) {
	    fprintf(stderr, "%s: malformed line: %s", argv[0], buf);
	    return (1);
	}
	if (pmap_set(prog, vers, prot, (unsigned short) port) == 0)
	    fprintf(stderr, "not registered: %s", buf);
    }
    return (0);
}

/* parse_line - convert line to numbers */
int
parse_line(buf, prog, vers, prot, port)
char   *buf;
u_long *prog;
u_long *vers;
int    *prot;
unsigned *port;
{
    char    proto_name[BUFSIZ];

    if (sscanf(buf, "%lu %lu %s %u", prog, vers, proto_name, port) != 4) {
	return (0);
    }
    if (strcmp(proto_name, "tcp") == 0) {
	*prot = IPPROTO_TCP;
	return (1);
    }
    if (strcmp(proto_name, "udp") == 0) {
	*prot = IPPROTO_UDP;
	return (1);
    }
    if (sscanf(proto_name, "%d", prot) == 1) {
	return (1);
    }
    return (0);
}
