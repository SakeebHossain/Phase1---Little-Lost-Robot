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

/* still need to use _close_command */


/* return the pointer to the linear memory segment */
int open_read_linear(char *name) {

	unsigned char cm[22];
	char *rbuf;
	int *rlen, cmret;
	int memseg;

	if (dodebug) {
		printf("open_read_linear:\n");
	}
	memset(&cm, 0, 22);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x8A;
	memcpy(&cm[2], name, strlen(name));
	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run open_read_linear\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(&memseg, &rbuf[3], 4);

	if (dodebug) {
		printf("open_read_linear: memory segment %d\n", memseg);
	}

	free(rbuf);
	return(memseg); /* memory segment */
}

unsigned char _open_write_linear(char *name, int fsize) {

	unsigned char cm[26], handle;
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_open_write_linear: %s %d\n", name, fsize);
	}
	memset(&cm, 0, 26);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x89;
	memcpy(&cm[2], name, strlen(name));
	memcpy(&cm[22], &fsize, 4);
	cmret = _do_cmd((char *)&cm, 26, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run open_write_linear\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	handle = (unsigned char) rbuf[3];
	free(rbuf);
	return((unsigned char) handle); /* the handle */
}

unsigned char _open_write_data(char *name, int fsize) {

	unsigned char cm[26], handle;
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_open_write_data: %s %d\n", name, fsize);
	}

	memset(&cm, 0, 26);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x8B;
	memcpy(&cm[2], name, strlen(name));
	memcpy(&cm[22], &fsize, 4);
	cmret = _do_cmd((char *)&cm, 26, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run open_write_data\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	handle = (unsigned char) rbuf[3];
	free(rbuf);
	return((unsigned char) handle); /* the handle */
}

unsigned char _open_append_data(char *name, int fsize) {

	unsigned char cm[22], handle;
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_open_append_data: %s %d\n", name, fsize);
	}
	memset(&cm, 0, 26);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x8B;
	memcpy(&cm[2], name, strlen(name));
	/*memcpy(&cm[22], &fsize, 4);*/
	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run open_append_data\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	handle = (unsigned char) rbuf[3];
	free(rbuf);
	return((unsigned char) handle); /* the handle */
}

