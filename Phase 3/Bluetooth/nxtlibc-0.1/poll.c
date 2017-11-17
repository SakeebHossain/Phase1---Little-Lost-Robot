/*
 * nxtlibc, the lego mindstorms nxt bluetooth api.
 * Copyright (C) 2007 Don Neumann
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "nxtlibc.h"

extern int dodebug;

/*
 * 
 * I have no idea what these do, hence they have not been tested
 * and I don't know if they work
 *
 */

/* one of POLL_BUFFER OR HS_BUFFER */
int poll_command_length(unsigned char bufnum) {

	unsigned char cm[3];
	char *rbuf;
	int rlen, cmret;
	int rret = 0;

	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0xA1;
	cm[2] = (unsigned char) bufnum;

	if (dodebug) {
		printf("poll_command_length %d:\n", bufnum);
	}

	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 3);
	if (cmret) {
		fprintf(stderr, "Failed to poll command length buffer\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (dodebug) {
		printf("poll_command_length: got extra lengths of %d\n", (int)(rlen - 3));
	}

	memcpy(&rret, &cm[4], (int)(rlen - 3));

	free(rbuf);
	return(rret);
}

int poll_command(unsigned char bufnum, unsigned char length, char **buf, int *len) {

	unsigned char cm[4] = { 0x01, 0xA2, bufnum, length };
	char *rbuf;
	int rlen, cmret;
	unsigned char lod;

	if (dodebug) {
		printf("poll_command %d %d:\n", bufnum, length);
	}
	cmret = _do_cmd((char *)&cm, 4, (char **)&rbuf, (int *)&rlen, 3);
	if (cmret) {
		fprintf(stderr, "Failed to poll command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	lod = (unsigned char) rbuf[4]; /* length of data */
	if (dodebug) {
		printf("got lod %d\n", lod);
	}
	*len = (int) lod;
	if (dodebug) {
		printf("got len %d\n", (int)*len);
	}

	*buf = (char *) malloc((int)*len);
	memcpy(*buf, &rbuf[5], (int)*len);

	free(rbuf);
	return((int)*len);
}
