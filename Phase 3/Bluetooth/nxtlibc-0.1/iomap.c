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
struct fstruct *modulefirst = NULL, *modulenext = NULL;

int _close_module_handle(unsigned char handle) {
	unsigned char cm[3] = { 0x01, 0x92, (unsigned char) handle };
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_close_module_handle:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to close module handle\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	/* do I really need more here? byte 2 is status, byte 3 is handle */
	if ((unsigned char)rbuf[3] != (unsigned char)handle) {
		fprintf(stderr, "close_module_handle weirdnes, request close on %d but closed %d\n", handle, (unsigned char)rbuf[3]);
	}

	free(rbuf);
	return(0);
}

unsigned char _request_first_module(char *mask) {

	unsigned char cm[22], name[20];
	unsigned char handle;
	int moduleid, modulesize;
	short int iomapsize;
	char *rbuf, *modname;
	int *rlen, cmret;
	struct fstruct *fs = (struct fstruct *) malloc(sizeof(struct fstruct));

	if (mask == NULL) {
		modname = (char *) strdup("*.*");
	}
	else {
		modname = mask;
	}

	if (dodebug) {
		printf("request_first_module:\n");
	}

	memset(&cm, 0, 22);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x90;
	memcpy(&cm[2], modname, strlen(modname));

	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);

	if (cmret) {
		fprintf(stderr, "Failed to run request_first_module\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	handle = (unsigned char) rbuf[3];
	memset(&name, 0, 20);
	memcpy(&name, &rbuf[4], 20);
	memcpy(&moduleid, &rbuf[24], 4);
	memcpy(&modulesize, &rbuf[28], 4);
	memcpy(&iomapsize, &rbuf[32], 2);

	fs->filename = (char *) strdup(name);
	fs->module = moduleid;
	fs->moduleid = (unsigned char) rbuf[26];
	if (modulefirst != NULL) {
		/* then we should clear it */
	}
	else {
		modulefirst = fs;
		modulefirst->next = NULL;
		modulenext = modulefirst;
	}

	if (dodebug) {
		printf("name %s moduleid %d modulesize %d iomapsize %d\n", name, moduleid, modulesize, iomapsize);
	}

	free(rbuf);
	return((unsigned char)handle);
}

int _request_next_module(unsigned char handle) {

	char *rbuf;
	int *rlen, cmret;
	unsigned char cm[3] = { 0x01, 0x91, (unsigned char) handle };
	unsigned int size;
	unsigned char name[20];
	int moduleid, modulesize;
	short int iomapsize;
	struct fstruct *fs;

	if (dodebug) {
		printf("_request_next_module:\n");
	}

	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x91;
	cm[2] = (unsigned char) handle;
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);

	if (cmret) {
		fprintf(stderr, "Failed to run _request_next_module\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memset(&name, 0, 19);
	memcpy(&name, &rbuf[4], 20);
	memcpy(&moduleid, &rbuf[24], 4);
	memcpy(&modulesize, &rbuf[28], 4);
	memcpy(&iomapsize, &rbuf[32], 2);

	fs = (struct fstruct *) malloc(sizeof(struct fstruct));
	fs->filename = strdup(name);
	fs->module = moduleid;
	fs->moduleid = (unsigned char) rbuf[26];
	fs->next = NULL;
	modulenext->next = fs;
	modulenext = fs;

	if (dodebug) {
		printf("name %s moduleid %d modulesize %d iomapsize %d\n", name, moduleid, modulesize, iomapsize);
	}

	free(rbuf);
	return(0);
}

/*
 * The following wildcards will be accepted within the file mgt system
 * Filename.Extension
 * *.[File type name]
 * Filename.*
 * *.*
 */
void list_modules(char *mask) {

	unsigned char handle;

	if (dodebug) {
		printf("list_modules:\n");
	}
	if (modulefirst) return;
	handle = _request_first_module(mask);
	while (_request_next_module(handle) != -1) {
		// EMPTY
	}
	_close_module_handle(handle);
}

int get_module_id(int module) {

	struct fstruct *fs;

	if (dodebug) { printf("get_module_id:\n"); }
	if (!modulefirst) {
		list_modules(NULL);
	}

	fs = modulefirst;
	while (fs != NULL) {
		if (fs->moduleid == module) return(fs->module);
		fs = fs->next;
	}

	return(-1);
}

int read_io_map(int moduleid, unsigned short int index, unsigned short int len, unsigned char **data) {

	char *rbuf;
	int *rlen, cmret, i;
	unsigned char cm[10];
	unsigned short int nlen;

	if (dodebug) {
		printf("read_io_map:\n");
	}

	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x94;
	memcpy(&cm[2], &moduleid, 4);
	memcpy(&cm[6], &index, 2);
	memcpy(&cm[8], &len, 2);

	cmret = _do_cmd((char *)&cm, 10, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run read_io_map\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(&nlen, &rbuf[7], 2);
	*data = (unsigned char *) malloc((int)nlen);
	memcpy(*data, &rbuf[9], (int)nlen);

	if (dodebug) {
		printf("nlen %d\n", nlen);
	}

	free(rbuf);
	return((int)nlen);
}

int write_io_map(int moduleid, unsigned short int index, unsigned short int len, unsigned char *data) {

	char *rbuf;
	int *rlen, cmret;
	unsigned char cm[1000];
	unsigned short int nlen;

	if (dodebug) {
		printf("write_io_map:\n");
	}

	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x95;
	memcpy(&cm[2], &moduleid, 4);
	memcpy(&cm[6], &index, 2);
	memcpy(&cm[8], &len, 2);
	memcpy(&cm[10], data, len);

	cmret = _do_cmd((char *)&cm, 10 + len, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run write_io_map\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	free(rbuf);
	return(0);
}
