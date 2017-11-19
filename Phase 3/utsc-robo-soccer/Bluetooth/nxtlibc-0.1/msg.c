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
 *
 * **NOTE** - There were pervasive pointer/integer misuse problems
 *            in this code. I've removed them as far as making sure
 *            data types and casting are correct and consistent,
 *            but the functionality is in question since these 
 *            problems were present in the code we have been using
 *            for a couple years. This library should be rewritten
 *            from scratch, with proper documentation of what 
 *            functions expect and return, and proper typing
 *            enforced via data definitions, not ad-hoc casting to
 *            make the compiler shut up. FJE - Jul. 2015
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
 * these routines are untested, so I Don't know if they work
 */

int message_write(unsigned char inbox, unsigned char size, unsigned char *data) {

	unsigned char cm[256];
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("message_write:\n");
	}

	cm[0] = (unsigned char) 0x0;
	cm[1] = (unsigned char) 0x09;
	cm[2] = (unsigned char) inbox;
	cm[3] = (unsigned char) size;
	memcpy(&cm[4], data, (int)size);

	if (dodebug) {
		printf("message_write:\n");
	}
	cmret = _do_cmd((char *)&cm, 4+(int)size, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run message_write command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	free(rbuf);
	return(0);
}

int message_read(unsigned char rinbox, unsigned char linbox, unsigned char remove, unsigned char *inbox, unsigned char *size, unsigned char **data) {

	unsigned char cm[256];
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("message_read:\n");
	}

	cm[0] = (unsigned char) 0x0;
	cm[1] = (unsigned char) 0x13;
	cm[2] = (unsigned char) rinbox;
	cm[3] = (unsigned char) linbox;
	cm[4] = (unsigned char) remove; /* remove? non-zero clears message from inbox */

	if (dodebug) {
		printf("message_read:\n");
	}

	cmret = _do_cmd((char *)&cm, 5, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run message_read command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	*inbox = (unsigned char) cm[3];
	*size = (unsigned char) cm[4];
	*data = (unsigned char *) malloc((int)*size);
	memcpy(*data, &cm[5], (int)*size);

	free(rbuf);
	return(0);
}
