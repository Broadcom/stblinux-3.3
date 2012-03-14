/*
 * librad.c - RADIUS protocol library.
 *
 * (C) Copyright 2001, Lineo Inc. (www.lineo.com)
 */

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <md5.h>
#include <magic.h>

#include "radius.h"
#include "librad.h"


#define RESEND_TIMEOUT  3
#define RESEND_COUNT    10
#define RADIUS_ID_FILE "/var/log/radius.id"
#define RADIUS_SESSIONID_FILE "/var/log/radius.sessionid"
#define RADIUS_ENCRYPT_PASSWORD_LEN(l) \
	(((l) + AUTH_VECTOR_LEN - 1) & ~(AUTH_VECTOR_LEN-1))


static u_char
radius_id(void)
{
	int fd, n;
	u_char id;

	fd = open(RADIUS_ID_FILE, O_RDWR|O_CREAT, 0644);
	if (fd < 0) {
		syslog(LOG_ERR, "%s: %m", RADIUS_ID_FILE);
		return -1;
	}
	if (flock(fd, LOCK_EX) != 0) {
		syslog(LOG_ERR, "failed to lock %s", RADIUS_ID_FILE);
	}

	n = read(fd, &id, 1);
	if (n < 1) {
		id = magic();
	} else {
		id++;
	}
	lseek(fd, 0L, SEEK_SET);
	write(fd, &id, 1);
    
	flock(fd, LOCK_UN);
	close(fd);

	return id;
}

u_int
radius_sessionid(void)
{
	int fd, n;
	u_char sessionid;

	fd = open(RADIUS_SESSIONID_FILE, O_RDWR|O_CREAT, 0644);
	if (fd < 0) {
		syslog(LOG_ERR, "%s: %m", RADIUS_SESSIONID_FILE);
		return -1;
	}
	if (flock(fd, LOCK_EX) != 0) {
		syslog(LOG_ERR, "failed to lock %s", RADIUS_SESSIONID_FILE);
	}

	n = read(fd, &sessionid, sizeof(sessionid));
	if (n < sizeof(sessionid)) {
		sessionid = magic();
	} else {
		sessionid++;
	}
	lseek(fd, 0L, SEEK_SET);
	write(fd, &sessionid, sizeof(sessionid));

	flock(fd, LOCK_UN);
	close(fd);

	return sessionid;
}

static void
radius_random_vector(u_char *vector, int length)
{
	u_int i;

	while (length > 0) {
		i = magic();
		memcpy(vector, &i, (length<sizeof(i)) ? length : sizeof(i));
		vector += sizeof(i);
		length -= sizeof(i);
	}
}

static int
radius_encrypt_password(
		char *secret, u_char *vector,
		u_char *buffer,	u_char *password, int length)
{
	int secret_len;
	int i;
	MD5_CTX context;

	secret_len = strlen(secret);
	for (;;) {
		MD5Init(&context);
		MD5Update(&context, secret, secret_len);
		MD5Update(&context, vector, AUTH_VECTOR_LEN);
		MD5Final(buffer, &context);
		for (i=0; i<AUTH_VECTOR_LEN && length>0; i++, length--) {
			buffer[i] ^= password[i];
		}
		if (length <= 0) {
			break;
		}
		vector = buffer;
		buffer += AUTH_VECTOR_LEN;
		password += AUTH_VECTOR_LEN;
	}

	return 1;
}

static void
radius_calc_vector(
		char *secret, u_char *buf, int len,
		u_char *vector_in, u_char *vector_out)
{
	int header_len;
	int secret_len;
	MD5_CTX context;

	header_len = ((AUTH_HDR*)buf)->vector - buf;
	secret_len = strlen(secret);
	MD5Init(&context);
	MD5Update(&context, buf, header_len);
	MD5Update(&context, vector_in, AUTH_VECTOR_LEN);
	MD5Update(&context, buf + header_len + AUTH_VECTOR_LEN,
			len - header_len - AUTH_VECTOR_LEN);
	MD5Update(&context, secret, secret_len);
	MD5Final(vector_out, &context);
}

/* If string is NULL, a 'value' attrib is added,
 * otherwise a 'string' attrib is added. */
int
radius_add_attrib(
		struct radius_attrib **list, u_char type,
		u_int value, char *string, u_int length)
{
	struct radius_attrib *attrib, **p;

	attrib = (struct radius_attrib*)malloc(sizeof(*attrib));
	if (attrib == NULL) {
		return 0;
	}

	attrib->type = type;
	attrib->next = NULL;
	if (string != NULL) {
		attrib->length = length;
		if (attrib->length > AUTH_STRING_LEN) {
			attrib->length = AUTH_STRING_LEN;
		}
		strncpy(attrib->u.string, string, attrib->length);
	}
	else {
		attrib->length = 4;
		attrib->u.value = htonl(value);
	}
	attrib->length += 2; /* type/length fields */

	for (p = list; *p != NULL; p = &((*p)->next));
	*p = attrib;

	return 1;
}

void
radius_free_attrib(struct radius_attrib *list)
{
	struct radius_attrib *p;

	while (list != NULL) {
		p = list->next;
		free(list);
		list = p;
	}
}

static int
radius_get_attrib(struct radius_attrib **list, u_char *buf, int len)
{
	struct radius_attrib *from, *to, **next;

	*list = NULL;
	next = list;
	from = (struct radius_attrib*)buf;
	while (len >= 2) {
		if (from->length < 2
				|| from->length > len
				|| from->length > sizeof(*to)) {
			radius_free_attrib(*list);
			*list = NULL;
			return 0;
		}

		to = (struct radius_attrib*)malloc(sizeof(*to));
		if (to == NULL) {
			radius_free_attrib(*list);
			*list = NULL;
			return 0;
		}

		memcpy(to, from, from->length);
		to->next = NULL;
		*next = to;
		next = &to->next;

		len -= from->length;
		from = (struct radius_attrib*)(((u_char*)from) + from->length);
	}
    
	return 1;
}

/* Send with timeouts/retries */
static int
radius_send(
		u_long host, int port, char *secret,
		u_char *sendbuf, int sendlen, u_char *recvbuf, int maxrecvlen)
{
	int s;
	struct sockaddr_in salocal, saremote, safrom;
	fd_set set;
	struct timeval timeout;
	int ret, recvlen, fromlen, resend, sendcount;
	u_char vector[AUTH_VECTOR_LEN];
	AUTH_HDR *sendheader, *recvheader;

	s = socket(PF_INET, SOCK_DGRAM, 0);
	if (s < 0) {
		syslog(LOG_ERR, "socket: %m");
		return -1;
	}

	memset(&salocal, 0, sizeof(salocal));
	salocal.sin_family = AF_INET;
	salocal.sin_addr.s_addr = htonl(INADDR_ANY);
	salocal.sin_port = 0;

	if (bind(s, (struct sockaddr*)&salocal, sizeof(salocal)) < 0) {
		syslog(LOG_ERR, "bind: %m");
		close(s);
		return -1;
	}

	memset(&saremote, 0, sizeof(saremote));
	saremote.sin_family = AF_INET;
	saremote.sin_addr.s_addr = htonl(host);
	saremote.sin_port = htons(port);

	sendheader = (AUTH_HDR*)sendbuf;
	recvheader = (AUTH_HDR*)recvbuf;

	resend = 1;
	sendcount = 0;
	while (sendcount < 10) {
		if (resend) {
			resend = 0;
			sendcount++;

			if (sendto(s, sendbuf, sendlen, 0,
					(struct sockaddr*)&saremote, sizeof(saremote)) < 0) {
				syslog(LOG_ERR, "sendto: %m");
				close(s);
				return -1;
			}
		}

		FD_ZERO(&set);
		FD_SET(s, &set);
		timeout.tv_sec = RESEND_TIMEOUT;
		timeout.tv_usec = 0;
		ret = select(s+1, &set, NULL, NULL, &timeout);
		if (ret < 0) {
			syslog(LOG_ERR, "select: %m");
			close(s);
			return -1;
		}
		if (ret == 0) {
			/* Timed out so resend */
			resend = 1;
			if (sendcount > 3) {
				syslog(LOG_ERR, "RADIUS server %s not responding",
						inet_ntoa(saremote.sin_addr));
			}
		}
		else if (FD_ISSET(s, &set)) {
			fromlen = sizeof(safrom);
			recvlen = recvfrom(s, recvbuf, maxrecvlen, 0,
					(struct sockaddr*)&safrom, &fromlen);
			if (recvlen < 0) {
				syslog(LOG_ERR, "recvfrom: %m");
				close(s);
				return -1;
			}

			if (safrom.sin_addr.s_addr != saremote.sin_addr.s_addr) {
				syslog(LOG_ERR, "Received unexpected packet from RADIUS server %s", inet_ntoa(safrom.sin_addr));
				continue;
			}

			if (recvlen == 0) {
				syslog(LOG_ERR, "Received 0 bytes from RADIUS server %s",
						inet_ntoa(safrom.sin_addr));
				continue;
			}

			if (recvlen < ntohs(recvheader->length)) {
				syslog(LOG_ERR, "Received packet with invalid length from RADIUS server %s", inet_ntoa(safrom.sin_addr));
				continue;
			}
			recvlen = ntohs(recvheader->length);

			if ((sendheader->code == PW_AUTHENTICATION_REQUEST
					&& recvheader->code != PW_AUTHENTICATION_ACK
					&& recvheader->code != PW_AUTHENTICATION_REJECT
					&& recvheader->code != PW_ACCESS_CHALLENGE)
					|| (sendheader->code == PW_ACCOUNTING_REQUEST
							&& recvheader->code != PW_ACCOUNTING_RESPONSE)) {
				syslog(LOG_ERR, "Received unexpected packet with code %d from RADIUS server %s", recvheader->code, inet_ntoa(safrom.sin_addr));
				continue;
			}

			if (sendheader->id != recvheader->id) {
				syslog(LOG_ERR, "Received packet with mismatched id from RADIUS server %s", inet_ntoa(safrom.sin_addr));
				continue;
			}

			radius_calc_vector(secret, recvbuf, recvlen,
					((AUTH_HDR*)sendbuf)->vector, vector);
			if (memcmp(((AUTH_HDR*)recvbuf)->vector, vector,
					AUTH_VECTOR_LEN) != 0) {
				syslog(LOG_ERR, "Received packet with invalid authenticator from RADIUS server %s", inet_ntoa(safrom.sin_addr));
				continue;
			}

			close(s);
			return recvlen;
		}
	}

	close(s);
	syslog(LOG_ERR, "Maximum retries reached for RADIUS server %s",
			inet_ntoa(saremote.sin_addr));
	return -1;
}

int
radius_send_access_request(
		u_long host, int port, char *secret,
		struct radius_attrib *attriblist,struct radius_attrib **recvattriblist)
{
	struct radius_attrib *attrib;
	struct radius_attrib *sendattrib;
	int attriblen, sendlen, recvlen;
	u_char *sendbuf, *p;
	u_char recvbuf[1024];
	AUTH_HDR *header;

	attriblen = 0;
	for (attrib = attriblist; attrib != NULL; attrib = attrib->next) {
		if (attrib->type == PW_PASSWORD) {
			attriblen += 2 + RADIUS_ENCRYPT_PASSWORD_LEN(attrib->length);
		}
		else {
			attriblen += attrib->length;
		}
	}

	sendlen = AUTH_HDR_LEN + attriblen;
	sendbuf = (u_char*)malloc(sendlen);
	if (sendbuf == NULL) {
		return -1;
	}

	header = (AUTH_HDR*)sendbuf;
	header->code = PW_AUTHENTICATION_REQUEST;
	header->id = radius_id();
	header->length = htons(sendlen);
	radius_random_vector(header->vector, sizeof(header->vector));

	p = sendbuf + AUTH_HDR_LEN;
	for (attrib = attriblist; attrib != NULL; attrib = attrib->next) {
		if (attrib->type == PW_PASSWORD) {
			sendattrib = (struct radius_attrib*)p;
			sendattrib->type = attrib->type;
			sendattrib->length = 2+RADIUS_ENCRYPT_PASSWORD_LEN(attrib->length);
			if (!radius_encrypt_password(secret, header->vector,
					sendattrib->u.string, attrib->u.string, attrib->length)) {
				free(sendbuf);
				return -1;
			}
			p += sendattrib->length;
		}
		else {
			memcpy(p, attrib, attrib->length);
			p += attrib->length;
		}
	}

	recvlen = radius_send(host, port, secret, sendbuf, sendlen,
			recvbuf, sizeof(recvbuf));
	free(sendbuf);
	if (recvlen <= 0) {
		return -1;
	}

	if (!radius_get_attrib(recvattriblist,
			recvbuf + AUTH_HDR_LEN, recvlen - AUTH_HDR_LEN)) {
		return -1;
	}

	return ((AUTH_HDR*)recvbuf)->code;
}

int
radius_send_account_request(
		u_long host, int port, char *secret,
		struct radius_attrib *attriblist,struct radius_attrib **recvattriblist)
{
	struct radius_attrib *attrib;
	int attriblen, sendlen, recvlen;
	u_char *sendbuf, *p;
	u_char recvbuf[1024];
	AUTH_HDR *header;

	attriblen = 0;
	for (attrib = attriblist; attrib != NULL; attrib = attrib->next) {
		attriblen += attrib->length;
	}

	sendlen = AUTH_HDR_LEN + attriblen;
	sendbuf = (u_char*)malloc(sendlen);
	if (sendbuf == NULL) {
		return -1;
	}

	header = (AUTH_HDR*)sendbuf;
	header->code = PW_ACCOUNTING_REQUEST;
	header->id = radius_id();
	header->length = htons(sendlen);
	memset(header->vector, 0, AUTH_VECTOR_LEN);

	p = sendbuf + AUTH_HDR_LEN;
	for (attrib = attriblist; attrib != NULL; attrib = attrib->next) {
		memcpy(p, attrib, attrib->length);
		p += attrib->length;
	}

	radius_calc_vector(secret, sendbuf, sendlen,
			((AUTH_HDR*)sendbuf)->vector, ((AUTH_HDR*)sendbuf)->vector);

	recvlen = radius_send(host, port, secret, sendbuf, sendlen,
			recvbuf, sizeof(recvbuf));
	free(sendbuf);
	if (recvlen <= 0) {
		return -1;
	}

	if (!radius_get_attrib(recvattriblist,
			recvbuf + AUTH_HDR_LEN, recvlen - AUTH_HDR_LEN)) {
		return -1;
	}

	return ((AUTH_HDR*)recvbuf)->code;
}
