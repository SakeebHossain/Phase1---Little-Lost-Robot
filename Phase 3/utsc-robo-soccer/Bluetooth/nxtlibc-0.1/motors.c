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

int _set_output_state(unsigned char port, signed char power, unsigned char mode, unsigned char regulation, signed char ratio, unsigned char runstate, unsigned long tacholimit) {

	char *rbuf;
	unsigned char cm[13];
	int *rlen, cmret;

	cm[0] = (unsigned char) 0x00;
	cm[1] = (unsigned char) 0x04;
	cm[2] = (unsigned char) port;
	cm[3] = (signed char) power;
	cm[4] = (unsigned char) mode;
	cm[5] = (unsigned char) regulation;
	cm[6] = (signed char) ratio;
	cm[7] = (unsigned char) runstate;
	memcpy(&cm[8], &tacholimit, 5);

	if (dodebug) {
		printf("_set_output_state:\n");
	}

	cmret = _do_cmd((char *)&cm, 13, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _set_output_state command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	free(rbuf);
	return(0);
}

int _get_output_state(unsigned char port, signed char *power, unsigned char *mode, unsigned char *regulation, signed char *ratio, unsigned char *runstate, unsigned long *tacholimit, int *tachocount, int *blocktachocount, int *rotationcount) {

	char *rbuf;
	unsigned char cm[3] = { 0x0, 0x06, port };
	int *rlen, cmret;
	unsigned long t1 = 0;
	int t2;

	if (dodebug) { printf("_set_output_state:\n"); }
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _set_output_state command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	*power = (signed char) rbuf[4];
	*mode = (unsigned char) rbuf[5];
	*regulation = (unsigned char) rbuf[6];
	*ratio = (signed char) rbuf[7];
	*runstate = (unsigned char) rbuf[8];

	memcpy(&t1, &rbuf[9], 5);
	*tacholimit = t1;
	memcpy(&t2, &rbuf[13], 4);
	*tachocount = t2;
	memcpy(&t2, &rbuf[17], 4);
	*blocktachocount = t2;
	memcpy(&t2, &rbuf[21], 4);
	*rotationcount = t2;

	free(rbuf);
	return(0);
}

/* relative is boolean
 * true: position relative to last movement
 * false: absolute position
 */

int reset_motor_position(unsigned char port, unsigned char relative) {

	char *rbuf;
	unsigned char cm[4] = { 0x0, 0x0A, port, relative };
	int *rlen, cmret;

	if (dodebug) {
		printf("_set_output_state:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _set_output_state command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}
