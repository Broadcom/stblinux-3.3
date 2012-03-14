/*
 * radius.c - RADIUS authentication plugin for pppd
 *
 * (C) Copyright 2001, Lineo Inc. (www.lineo.com)
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include <netdb.h>
#include <ctype.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <pppd.h>
#include <fsm.h>
#include <ipcp.h>
#include <magic.h>

#include "radius.h"
#include "librad.h"

/* State information */
static bool plugin_loaded = 0;
static int accountstart = 0;
static char sessionid[16];
static char radius_reply_message[AUTH_STRING_LEN+1] = "";
static int radius_class_length = -1;
static char radius_class[256];

/* Hooks */
static int (*prev_pap_check_hook) __P((void));
static int (*prev_pap_auth_hook) __P((char *user, char *passwd, char **msgp,
		struct wordlist **paddrs, struct wordlist **popts));
static void (*prev_ip_up_hook) __P((void));
static void (*prev_ip_down_hook) __P((void));

/* Options */
static bool use_radius = 0;
static bool use_account = 0;
static u_long radius_server = -1;
static int radius_auth_port = PW_AUTH_UDP_PORT;
static char radius_secret[MAXSECRETLEN] = "";    /* key to encrypt packets */
static u_long nas_ip_address = -1;
static char nas_identifier[AUTH_STRING_LEN+1] = "";
static u_long nas_port_number = -1;
static u_long nas_port_type = -1;

static int radius_get_server(char**);
static int radius_nas_ip_address(char**);

static option_t radius_options[] =
{
	{ "radius", o_bool, &use_radius,
	  STR("Enable RADIUS authentication"), 1 },
	{ "radius-accounting", o_bool, &use_account,
	  STR("Enable RADIUS accounting"), 1 },
	{ "radius-server", o_special, radius_get_server,
	  STR("RADIUS server IP address and optional authentication port") },
	{ "radius-secret", o_string, radius_secret,
	  STR("Key used to encrypt RADIUS packets"),
	  OPT_STATIC, NULL, MAXSECRETLEN },
	{ "radius-nas-ip-address", o_special, radius_nas_ip_address,
	  STR("NAS IP address for RADIUS") },
	{ "radius-nas-identifier", o_string, nas_identifier,
	  STR("NAS identifier for RADIUS"), OPT_STATIC, NULL, AUTH_STRING_LEN },
	{ "radius-port-number", o_int, &nas_port_number,
	  STR("Port number for RADIUS") },
	{ "radius-port-type", o_int, &nas_port_type,
	  STR("Port type for RADIUS") },
	{ NULL }
};

static int
radius_get_server(char **argv)
{
	char *p, *endp;
	struct servent *servp;
	struct hostent *hostp;
	struct in_addr addr;

	/* Determine the port */
	p = strchr(*argv, ':');
	if (p != NULL) {
		radius_auth_port = strtoul(p+1, &endp, 10);
		if (*endp) {
			option_error("invalid RADIUS server port '%s'", p+1);
			return 0;
		}
	}
	if (radius_auth_port == 0) {
		servp = getservbyname("radacct", "udp");
		if (servp != NULL) {
			radius_auth_port = ntohs(servp->s_port);
		} else {
			radius_auth_port = PW_AUTH_UDP_PORT;
		}
	}

	/* Remove port if present */
	if (p != NULL)
		*p = 0;
	/* Determine the server IP address */
	if (inet_aton(*argv, &addr) == 0) {
		hostp = gethostbyname(*argv);
		if (hostp == NULL) {
			option_error("invalid RADIUS server '%s'", *argv);
			return 0;
		}
		memcpy((char*)&addr, hostp->h_addr, sizeof(addr));
	}
	if (p != NULL)
		*p = ':';

	radius_server = ntohl(addr.s_addr);
	return 1;
}

static int
radius_nas_ip_address(char **argv)
{
	struct in_addr addr;

	if (inet_aton(*argv, &addr) == 0) {
		option_error("invalid RADIUS NAS IP address '%s'", *argv);
		return 0;
	}

	nas_ip_address = ntohl(addr.s_addr);
	return 1;
}

static int
radius_check(void)
{
	int ret;
	char *tty;

	if (prev_pap_check_hook) {
		ret = prev_pap_check_hook();
		if (ret >= 0) {
			return ret;
		}
	}

	if (!use_radius)
		return -1;

	if (radius_server == -1)
		return 0;

	/* Determine reasonable defaults for unspecified options */
	tty = devnam;
	if (strncmp(tty, "/dev/", 5) == 0)
		tty += 5;
	if (nas_port_type == -1) {
		if (strncmp(tty, "ttyp", 4) == 0)
			nas_port_type = PW_NAS_PORT_VIRTUAL;
		else
			nas_port_type = PW_NAS_PORT_ASYNC;
	}
	while (isalpha(*tty))
		tty++;
	if (nas_port_number == -1 && isdigit(*tty))
		nas_port_number = atoi(tty);

	return 1;
}


/* Authenticate/authorize */
static int
radius_auth(char *t_user, char *t_passwd, char **t_msgp,
		struct wordlist **t_paddrs, struct wordlist **t_popts)
{
	int ret;
	struct radius_attrib *attriblist;
	struct radius_attrib *recvattriblist;
	struct radius_attrib *attrib;
	char *addrstr;
	int addrlen;
	struct wordlist *addr;
    
	if (prev_pap_auth_hook) {
		ret = prev_pap_auth_hook(t_user, t_passwd, t_msgp, t_paddrs, t_popts);
		if (ret >= 0) {
			return ret;
		}
	}
    
	if (!use_radius)
		return -1;

	if (radius_server == -1) {
		*t_msgp = "RADIUS server not found";
		return 0;
	}

	attriblist = NULL;
	recvattriblist = NULL;

	*t_msgp = "Out of memory for RADIUS attribute";
	if (!radius_add_attrib(
			&attriblist, PW_USER_NAME, 0, t_user, strlen(t_user))) {
		radius_free_attrib(attriblist);
		return 0;
	}

	if (!radius_add_attrib(
			&attriblist, PW_PASSWORD, 0, t_passwd, strlen(t_passwd))) {
		radius_free_attrib(attriblist);
		return 0;
	}

	if (nas_ip_address != -1) {
		if (!radius_add_attrib(
				&attriblist, PW_NAS_IP_ADDRESS, nas_ip_address, NULL, 0)) {
			radius_free_attrib(attriblist);
			return 0;
		}
	}
	else if (nas_identifier[0]) {
		if (!radius_add_attrib(
				&attriblist, PW_NAS_IDENTIFIER,
				0, nas_identifier, strlen(nas_identifier))) {
			radius_free_attrib(attriblist);
			return 0;
		}
	}

	if (nas_port_number != -1) {
		if (!radius_add_attrib(
				&attriblist, PW_NAS_PORT_ID, nas_port_number, NULL, 0)) {
			radius_free_attrib(attriblist);
			return 0;
		}
	}

	if (nas_port_type != -1) {
		if (!radius_add_attrib(
				&attriblist, PW_NAS_PORT_TYPE, nas_port_type, NULL, 0)) {
			radius_free_attrib(attriblist);
			return 0;
		}
	}

	if (!radius_add_attrib(
			&attriblist, PW_SERVICE_TYPE, PW_FRAMED_USER, NULL, 0)) {
		radius_free_attrib(attriblist);
		return 0;
	}

	if (!radius_add_attrib(&attriblist, PW_FRAMED_PROTOCOL, PW_PPP, NULL, 0)) {
		radius_free_attrib(attriblist);
		return 0;
	}

	ret = radius_send_access_request(
			radius_server, radius_auth_port, radius_secret,
			attriblist, &recvattriblist);
	if (ret < 0) {
		*t_msgp = "RADIUS server failed";
		ret = 0;
	}
	else if (ret == PW_AUTHENTICATION_ACK) {
		ret = 1; /* Default to success unless an attribute makes us fail */
		*t_msgp = "RADIUS authentication accepted.";
		*t_paddrs = NULL;
		for (attrib=recvattriblist; ret && attrib!=NULL; attrib=attrib->next) {
			switch (attrib->type) {
			case PW_SERVICE_TYPE:
				if (ntohl(attrib->u.value) != PW_FRAMED_USER) {
					*t_msgp = "RADIUS service type is not framed";
					ret = 0;
				}
				break;

			case PW_FRAMED_PROTOCOL:
				if (ntohl(attrib->u.value) != PW_PPP) {
					*t_msgp = "RADIUS framed protocol is not PPP";
					ret = 0;
				}
				break;

			case PW_FRAMED_IP_ADDRESS:
				if (ntohl(attrib->u.value) != 0xfffffffe
						&& ntohl(attrib->u.value) != 0xffffffff) {
					addrstr = inet_ntoa(attrib->u.addr);
					addrlen = strlen(addrstr);
					addr = (struct wordlist*)malloc(sizeof(struct wordlist)
							+ addrlen + 1);
					if (addr == NULL) {
						*t_msgp = "Out of memory for RADIUS address";
						ret = 0;
					}
					else {
						addr->word = (char*)(addr+1);
						strncpy(addr->word, addrstr, addrlen);
						addr->word[addrlen] = '\0';
			
						addr->next = *t_paddrs;
						*t_paddrs = addr;
					}
				}
				break;

			case PW_FRAMED_IP_NETMASK:
				if (attrib->u.value && attrib->u.value != 0xffffffff) {
					netmask = attrib->u.value;
				}
				break;

			case PW_FRAMED_COMPRESSION:
				if (ntohl(attrib->u.value) == PW_NONE) {
					ipcp_wantoptions[0].neg_vj = 0;
					ipcp_allowoptions[0].neg_vj = 0;
				}
				else if (ntohl(attrib->u.value) == PW_VAN_JACOBSEN_TCP_IP) {
					ipcp_wantoptions[0].neg_vj = 1;
					ipcp_allowoptions[0].neg_vj = 1;
				}
				break;

			case PW_REPLY_MESSAGE:
				strncpy(radius_reply_message,attrib->u.string,AUTH_STRING_LEN);
				radius_reply_message[AUTH_STRING_LEN] = 0;
				*t_msgp = radius_reply_message;
				break;

			case PW_FRAMED_ROUTE:
				/* XXX: store route for adding/removing in ip-up/ip-down */
				break;

			case PW_IDLE_TIMEOUT:
				if (attrib->u.value != 0) {
					idle_time_limit = ntohl(attrib->u.value);
				}
				break;

			case PW_SESSION_TIMEOUT:
				if (attrib->u.value != 0) {
					maxconnect = ntohl(attrib->u.value);
				}
				break;

			case PW_CLASS:
				radius_class_length = attrib->length - 2;
				memcpy(radius_class, attrib->u.string, radius_class_length);
				break;
			}
		}

		if (ret && !*t_paddrs) {
			/* Allow any address */
			addr = (struct wordlist*)malloc(sizeof(struct wordlist) + 2);
			if (addr == NULL) {
				*t_msgp = "Out of memory for RADIUS address";
				ret = 0;
			}
			else {
				addr->word = (char*)(addr+1);
				addr->word[0] = '*';
				addr->word[1] = '\0';
				addr->next = NULL;
				*t_paddrs = addr;
			}
		}
	}
	else if (ret == PW_AUTHENTICATION_REJECT) {
		*t_msgp = "RADIUS authentication rejected.";
		for (attrib=recvattriblist; attrib!=NULL; attrib=attrib->next) {
			if (attrib->type == PW_REPLY_MESSAGE) {
				strncpy(radius_reply_message,attrib->u.string,AUTH_STRING_LEN);
				radius_reply_message[AUTH_STRING_LEN] = 0;
				*t_msgp = radius_reply_message;
			}
		}
		ret = 0;
	}
	else if (ret == PW_ACCESS_CHALLENGE) {
		*t_msgp = "RADIUS server sent unexpected CHAP challenge.";
		ret = 0;
	}

	radius_free_attrib(attriblist);
	radius_free_attrib(recvattriblist);

	return ret;
}

static int
radius_common_account_attrib(struct radius_attrib **attriblist)
{
	if (!radius_add_attrib(
			attriblist, PW_USER_NAME, 0, peer_authname, strlen(peer_authname)))
		return 0;

	/* Although the RFC states that one of these two MUST be present,
	 * the cistron radiusd uses the source address of the packet if
	 * the PW_NAS_IP_ADDRESS is not specified. */
	if (nas_ip_address != -1) {
		if (!radius_add_attrib(
				attriblist, PW_NAS_IP_ADDRESS, nas_ip_address, NULL, 0))
			return 0;
	}
	else if (nas_identifier[0]) {
		if (!radius_add_attrib(
				attriblist, PW_NAS_IDENTIFIER,
				0, nas_identifier, strlen(nas_identifier)))
			return 0;
	}

	if (nas_port_number != -1) {
		if (!radius_add_attrib(
				attriblist, PW_NAS_PORT_ID, nas_port_number, NULL, 0))
			return 0;
	}
	
	if (nas_port_type != -1) {
		if (!radius_add_attrib(
				attriblist, PW_NAS_PORT_TYPE, nas_port_type, NULL, 0))
			return 0;
	}

	if (!radius_add_attrib(
			attriblist, PW_SERVICE_TYPE, PW_FRAMED_USER, NULL, 0))
		return 0;

	if (!radius_add_attrib(attriblist, PW_FRAMED_PROTOCOL, PW_PPP, NULL, 0))
		return 0;

	if (!radius_add_attrib(
			attriblist, PW_FRAMED_IP_ADDRESS,
			ipcp_hisoptions->hisaddr, NULL, 0))
		return 0;

	if (!radius_add_attrib(
			attriblist, PW_FRAMED_COMPRESSION,
			ipcp_gotoptions[0].neg_vj ? PW_VAN_JACOBSEN_TCP_IP : PW_NONE,
			NULL, 0))
		return 0;

	if (radius_class_length >= 0) {
		if (!radius_add_attrib(attriblist, PW_CLASS,
				0, radius_class, radius_class_length))
			return 0;
	}

	return 1;
}

static void
radius_ip_up(void)
{
	struct radius_attrib *attriblist, *recvattriblist;
	int ret;

	if (prev_ip_up_hook) {
		prev_ip_up_hook();
	}

	if (use_account) {
		if (radius_server == -1)
			return;

		attriblist = NULL;
		recvattriblist = NULL;
	
		if (!radius_add_attrib(
				&attriblist, PW_ACCT_STATUS_TYPE, PW_STATUS_START, NULL, 0)) {
			radius_free_attrib(attriblist);
			return;
		}

		sprintf(sessionid, "%x", radius_sessionid());
		if (!radius_add_attrib(
				&attriblist, PW_ACCT_SESSION_ID,
				0, sessionid, strlen(sessionid))) {
			radius_free_attrib(attriblist);
			return;
		}

		if (!radius_common_account_attrib(&attriblist)) {
			radius_free_attrib(attriblist);
			return;
		}

		ret = radius_send_account_request(
				radius_server, radius_auth_port+1, radius_secret,
				attriblist, &recvattriblist);

		radius_free_attrib(attriblist);
		radius_free_attrib(recvattriblist);

		if (ret >= 0) {
			accountstart = 1;
		}
	}
}

static void
radius_ip_down(void)
{
	struct radius_attrib *attriblist, *recvattriblist;

	if (prev_ip_down_hook) {
		prev_ip_down_hook();
	}

	/* Put in the accountstart check here since this hook
	 * also gets called if an IP address could not be
	 * negotiated. */
	if (use_account && accountstart) {
		accountstart = 0;

		if (radius_server == -1)
			return;

		attriblist = NULL;
		recvattriblist = NULL;
	
		if (!radius_add_attrib(
				&attriblist, PW_ACCT_STATUS_TYPE, PW_STATUS_STOP, NULL, 0)) {
			radius_free_attrib(attriblist);
			return;
		}

		if (!radius_add_attrib(
				&attriblist, PW_ACCT_SESSION_ID,
				0, sessionid, strlen(sessionid))) {
			radius_free_attrib(attriblist);
			return;
		}

		if (!radius_common_account_attrib(&attriblist)) {
			radius_free_attrib(attriblist);
			return;
		}

		if (link_stats_valid) {
			if (!radius_add_attrib(&attriblist, PW_ACCT_INPUT_OCTETS,
					link_stats.bytes_in, NULL, 0)) {
				radius_free_attrib(attriblist);
				return;
			}

			if (!radius_add_attrib(&attriblist, PW_ACCT_OUTPUT_OCTETS,
					link_stats.bytes_out, NULL, 0)) {
				radius_free_attrib(attriblist);
				return;
			}

			if (!radius_add_attrib(
					&attriblist, PW_ACCT_SESSION_TIME,
					link_connect_time, NULL, 0)) {
				radius_free_attrib(attriblist);
				return;
			}

		}

		radius_send_account_request(
				radius_server, radius_auth_port+1, radius_secret,
				attriblist, &recvattriblist);

		radius_free_attrib(attriblist);
		radius_free_attrib(recvattriblist);
	}
}

void
#ifdef EMBED
radius_plugin_init(void)
#else
	 plugin_init(void)
#endif
{
	if (!plugin_loaded) {
		plugin_loaded = 1;

		magic_init();
	
		/* install pppd hooks */
		add_options(radius_options);
	
		prev_pap_check_hook = pap_check_hook;
		pap_check_hook = radius_check;
	
		prev_pap_auth_hook = pap_auth_hook;
		pap_auth_hook = radius_auth;
	
		prev_ip_up_hook = ip_up_hook;
		ip_up_hook = radius_ip_up;
	
		prev_ip_down_hook = ip_down_hook;
		ip_down_hook = radius_ip_down;
	}
}
