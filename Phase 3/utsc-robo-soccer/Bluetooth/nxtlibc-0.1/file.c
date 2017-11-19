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

struct fstruct *filefirst = NULL, *filenext = NULL;

int _close_command(unsigned char handle)
{
	unsigned char cm[3] = { 0x01, 0x84, (unsigned char) handle };
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_close_command:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to close handle\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

/* do I really need more here? byte 2 is status, byte 3 is handle */
	if ((unsigned char)rbuf[3] != (unsigned char)handle) {
		fprintf(stderr, "close_command weirdnes, request close on %d but closed %d\n", handle, (unsigned char)rbuf[3]);
	}

	free(rbuf);
	return(0);
}

unsigned char _find_first(char *mask)
{
	unsigned char cm[22], name[19];
	unsigned char handle;
	unsigned int size;
	char *rbuf;
	int *rlen, cmret;
	struct fstruct *fs = (struct fstruct *) malloc(sizeof(struct fstruct));

	if (dodebug) {
		printf("list_files:\n");
	}

	memset(&cm, 0, 22);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x86;
	if (mask) {
		memcpy(&cm[2], mask, strlen(mask));
	}
	else {
		memcpy(&cm[2], "*.*", 3);
	}

	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _find_first\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	handle = (unsigned char) rbuf[3];
	memcpy(&size, &rbuf[24], 4);
	memset(&name, 0, 19);
	memcpy(&name, &rbuf[4], 19);

	fs->filename = (char *) strdup(name);
	fs->size = size;

	if (filefirst != NULL) {
	/* then we should clear it */
	}
	else {
	filefirst = fs;
	filefirst->next = NULL;
	filenext = filefirst;
	}

	if (dodebug) {
		printf("handle %d size %d name %s\n", (unsigned char)handle, (unsigned int)size, name);
	}

	free(rbuf);
	return((unsigned char)handle);
}

int _find_next(unsigned char handle)
{
	char *rbuf;
	int *rlen, cmret;
	unsigned char cm[3] = { 0x01, 0x87, (unsigned char) handle };
	unsigned int size;
	unsigned char name[19];
	struct fstruct *fs;

	if (dodebug) {
		printf("_find_next:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _find_next\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(&size, &rbuf[24], 4);
	memset(&name, 0, 19);
	memcpy(&name, &rbuf[4], 19);

	fs = (struct fstruct *) malloc(sizeof(struct fstruct));
	fs->filename = strdup(name);
	fs->size = size;
	fs->next = NULL;
	filenext->next = fs;
	filenext = fs;

	if (dodebug) {
		printf("handle %d size %d name %s\n", (unsigned char)handle, (unsigned int)size, (char *)name);
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
int list_files(char *mask) {

	unsigned char handle;

	if (dodebug) {
		printf("list_files:\n");
	}

	handle = _find_first(mask);
	if (handle == -1) return(-1);
	while (_find_next(handle) != -1) {
		// EMPTY
	}
	_close_command(handle);
	return(0);
}

/* return the handle */
unsigned char _open_read(char *name, int *fsize) {

	unsigned char cm[22], handle;
	char *rbuf;
	int *rlen, cmret;
	unsigned int fs;

	if (dodebug) {
		printf("_open_read:\n");
	}
	memset(&cm, 0, 22);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x80;
	memcpy(&cm[2], name, strlen(name));
	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run find first\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(&fs, &rbuf[4], 4);
	*fsize = fs;
	if (dodebug) {
		printf("_open_read: file size %d hadle %d\n", *fsize, (unsigned char)rbuf[3]);
	}
	handle = (unsigned char) rbuf[3];
	free(rbuf);
	return((unsigned char) handle); /* the handle */
}

int _read_command(unsigned char handle, char *buf, short int len) {

	unsigned char cm[5];
	char *rbuf;
	int *rlen, cmret;
	short int cread;

	if (dodebug) {
		printf("_read_command:\n");
	}

	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x82;
	cm[2] = (unsigned char) handle;
	memcpy(&cm[3], &len, 2);
	cmret = _do_cmd((char *)&cm, 5, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run find first\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(&cread, &rbuf[4], 2);
	if ((short int) cread != (short int)len) {
		fprintf(stderr, "Failed to read proper lengths, %d %d\n", cread, len);
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(buf, &rbuf[6], (short int)len);
	free(rbuf);
	if (dodebug) {
		printf("read_command copied %d bytes\n", (short int)len);
	}
	return((int)len);
}

int read_file(char *name, char **buf, int *len) {

	int fsize;
	unsigned char handle;

	if (dodebug) {
		printf("read_file:\n");
	}
	handle = _open_read(name, (int *)&fsize);
	if (handle == -1) return(-1);
	if (dodebug) {
		printf("opened %s size %d for reading\n", name, (int)fsize);
	}

	*buf = (char *) malloc((int)fsize);
	*len = (int) fsize;

	while ((int)fsize > 0) {
		char mybuf[56], myret;
		int pos;

		if (dodebug) {
			printf("fsize to read %d\n", (int)fsize);
		}
		myret = _read_command(handle, (char *)&mybuf, fsize >= 56 ? (short int)56 : (short int)fsize);
		if (dodebug) {
			printf("myret is %d pos is %d\n", myret, pos);
		}
		memcpy(*buf + pos, &mybuf, (int)fsize >= 56 ? 56 : (int)fsize);
		pos += (int)fsize >= 56 ? 56 : (int)fsize;
		fsize -= (int)14;
	}

	_close_command(handle);
	return(0);
}


unsigned char _open_write(char *name, int fsize) {

	unsigned char cm[26], handle;
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_open_write: %s %d\n", name, fsize);
	}

	memset(&cm, 0, 26);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x81;
	memcpy(&cm[2], name, strlen(name));
	memcpy(&cm[22], &fsize, 4);
	cmret = _do_cmd((char *)&cm, 26, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run open_write\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	handle = (unsigned char) rbuf[3];
	free(rbuf);
	return((unsigned char) handle); /* the handle */
}

int _write_command(unsigned char handle, char *buf, int len) {

	unsigned char cm[1024];
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_write_command:\n");
	}

	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x83;
	cm[2] = (unsigned char) handle;
	memcpy(&cm[3], buf, len);
	cmret = _do_cmd((char *)&cm, 3+len, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run write command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}
	free(rbuf);
	return(0);
}


/*
 * 0 is normal, 1 is linear, 2 is write data, 3 is append data
 * these last 3 are used for various filetypes of which I am unsure
 */
int write_file(int type, char *name, char *buf, int len) {

	unsigned char handle;
	int myret;
	int cp = 0, cp2 = 0;

	if (dodebug) {
		printf("write_file:\n");
	}
	if (type == FILE_NORMAL) {
		handle = _open_write(name, len);
	}
	else if (type == FILE_LINEAR) {
		handle = _open_write_linear(name, len);
	}
	else if (type == FILE_DATA) {
		handle = _open_write_data(name, len);
	}
	else if (type == FILE_APPEND_DATA) {
		handle = _open_append_data(name, len);
	}

	if (handle == -1) return(-1);

	if (dodebug) {
		printf("opened %s size %d for writing\n", name, len);
	}

	for (cp = 0; cp < len; cp+= 56) {
		cp2 = (len - cp > 56) ? 56 : (len - cp - 1);
		myret = _write_command(handle, &buf[cp], cp2);
	}

	_close_command(handle);
	return(0);
}

int delete_file(char *name) {

	unsigned char cm[22];
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("delete_command %s:\n", name);
	}
	memset(&cm[0], 0, 22);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x85;
	memcpy(&cm[2], name, strlen(name));
	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to delete %s\n", name);
		if (rbuf) free(rbuf);
		return(-1);
	}
	/* we can  compare with the return the filename deleted, not necessary */
	free(rbuf);
	return(0);
}
