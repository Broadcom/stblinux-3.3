/*
 *  sha1test.c
 *
 *  Description:
 *      This file will exercise the SHA-1 code performing the three
 *      tests documented in FIPS PUB 180-1 plus one which calls
 *      SHA1Input with an exact multiple of 512 bits, plus a few
 *      error test checks.
 *
 *  Portability Issues:
 *      None.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "sha1.h"

#define MSGBUF_SIZE 65536

int main(int argc, char ** argv)
{
    SHA1Context sha;
    int i, err, bytes_read;
    uint8_t Message_Digest[20];
	uint8_t *MsgBuf;
	FILE * f;

	if(argc < 2) {
		fprintf(stderr, "usage: sha1 filename\n");
		return -1;
	}
	MsgBuf = (uint8_t *)malloc(MSGBUF_SIZE);
	if(!MsgBuf) {
		fprintf(stderr, "can not alloc memory\n");
		return -1;
	}
	f = fopen(argv[1], "r");
	if(!f) {
		fprintf(stderr, "can not open file %s\n", argv[1]);
		free(MsgBuf);
		return -1;
	}
	err = SHA1Reset(&sha);
	if(err) {
            fprintf(stderr, "SHA1Reset Error %d.\n", err );
			goto error;
	}
	do {
		bytes_read = fread(MsgBuf, 1, MSGBUF_SIZE, f);
		err = SHA1Input(&sha, MsgBuf, bytes_read);
		if (err)
        {
            fprintf(stderr, "SHA1Input Error %d.\n", err );
            break;    /* out of for i loop */
        }
	}while(bytes_read == MSGBUF_SIZE);
	
	err = SHA1Result(&sha, Message_Digest);
	if (err)
	{
		fprintf(stderr,"SHA1Result Error %d, could not compute message digest.\n",err );
	}else {
		printf("\t");
		for(i = 0; i < 20 ; ++i)
			printf("%02x", Message_Digest[i]);
		printf("\n");
	}

error:
	fclose(f);
	free(MsgBuf);
    return err;
}

