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

int nxt_bluetooth_initialize(char *addr) {

	int ret;
	ret = _do_connect(addr);
	return(ret);
}

void nxt_bluetooth_done() {

	_do_close();
}

int get_battery_level() {

	char *rbuf;
	int rlen, cmret;
	short int slen;
	unsigned char cm[2] = { 0x0, 0x0b };

	if (dodebug) {
		printf("get_battery_level:\n");
	}

	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	memcpy(&slen, (char **)&rbuf[3], 2);
	if (cmret) {
		fprintf(stderr, "Failed to get battery level\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (dodebug) {
		printf("battery %d\n", (int)slen);
	}

	free(rbuf);
	return((int)slen);
}

char *get_firmware_version() {

	unsigned char cm[2] = { 0x01, 0x88 };
	char *rbuf, *bret;
	int rlen, cmret;

	if (dodebug) {
		printf("get_firmware_version:\n");
	}
	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to get firmware version");
		if (rbuf) free(rbuf);
		return(NULL);
	}

	if (dodebug) {
		printf("got rlen %d\n", (int)rlen);
	}

	bret = (char *) malloc(40);
	memset(bret, 0, 40);
	sprintf(bret, "Protocol: %u.%u Firmware: %u.%u", (unsigned char)rbuf[4], (unsigned char)rbuf[3], (unsigned char)rbuf[6], (unsigned char)rbuf[5]);

	free(rbuf);
	return(bret);
}

int _get_device_info(char **name, int *bsignal, int *flash) {

	unsigned char cm[2] = { 0x01, 0x9b };
	char *rbuf;
	int rlen, cmret;
	char nxtname[14], nxtaddr[6];
	int bs, fl;

	if (dodebug) {
		printf("get_device_info:\n");
	}
	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to get device info\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (dodebug) {
		printf("got rlen %d\n", (int)rlen);
	}

	debug_buf(&rbuf[3], 4);

	memset(&nxtname, 0, 14);
	memset(&nxtaddr, 0, 6);
	memcpy(&nxtname, &rbuf[3], 14);
	memcpy(&nxtaddr, &rbuf[18], 6);
	memcpy(&bs, &rbuf[25], 4);
	memcpy(&fl, &rbuf[29], 4);

	if (dodebug) {
		printf("nxtname %s\n", nxtname);
		printf("nxtaddr %s\n", nxtaddr);
		printf("bs %d\n", bs);
		printf("flash %d\n", fl);
	}

	*name = malloc(strlen(nxtname)+1);
	memset(*name, 0, strlen(nxtname)+1);
	memcpy(*name, &nxtname, strlen(nxtname));
	*bsignal = bs;
	*flash = fl;

	free(rbuf);
	return(0);
}

char *get_brick_name() {

	char *name;
	int *x1, *x2;

	if (dodebug) {
		printf("get_brick_name:\n");
	}

	_get_device_info((char **)&name, (int *) &x1, (int *)&x2);
	if (dodebug) {
		printf("got name %s\n", name);
	}
	return(name);
}

int get_free_flash() {

	char *n;
	int x1, flash;
	int ret;

	if (dodebug) {
		printf("get_free_flash:\n");
	}
	ret = _get_device_info((char **)&n, (int *)&x1, (int *)&flash);
	if (n) free(n);
	if (ret == -1) return(-1);
	return((int)flash);
}

/* this always seems to be zero for some reason */
int get_bluetooth_signal_strength() {

	char *n;
	int bs, x2;
	int ret;

	if (dodebug) {
		printf("get_bluetooth_signal_strength:\n");
	}
	ret = _get_device_info((char **)&n, (int *)&bs, (int *)&x2);
	if (n) free(n);
	if (ret == -1) return(-1);
	return((int)bs);
}

int start_program(char *name) {

	char *rbuf;
	int rlen, cmret;
	unsigned char cm[22];

	if (dodebug) { printf("start_program: %s\n", name); }
	memset(&cm, 0, 22);
	cm[0] = (unsigned char) 0x0;
	cm[1] = (unsigned char) 0x0;
	memcpy(&cm[2], name, strlen(name));

	cmret = _do_cmd((char *)&cm, 22, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to execute program %s\n", name);
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}

int stop_program() {

	unsigned char cm[2] = { 0x00, 0x01 };
	char *rbuf;
	int rlen, cmret;

	if (dodebug) {
		printf("stop_program:\n");
	}

	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to stop running program\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}

int play_sound_file(int boolean, char *name) {

	unsigned char cm[23];
	char *rbuf;
	int rlen, cmret;

	if (dodebug) {
		printf("play_sound_file:\n");
	}
	memset(&cm, 0, 23);
	cm[0] = (unsigned char) 0x0;
	cm[1] = (unsigned char) 0x02;
	cm[2] = (unsigned char) boolean; /* loop indefinitely? */
	memcpy(&cm[3], name, strlen(name));
	cmret = _do_cmd((char *)&cm, 23, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to play sound file\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}

int stop_sound_playback() {

	unsigned char cm[2] = { 0x00, 0x0C };
	char *rbuf;
	int rlen, cmret;

	if (dodebug) {
		printf("stop_sound_playback:\n");
	}
	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to stop sound playback\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}

/*
 * HZ range 200 - 14000
 * Duration uword ms
 */
int play_tone(short int hz, short int duration) {

	unsigned char cm[6] = { 0x00, 0x03 };
	char *rbuf;
	int rlen, cmret;

	if (dodebug) {
		printf("play_tone:\n");
	}

	memcpy(&cm[2], &hz, 2);
	memcpy(&cm[4], &duration, 2);

	cmret = _do_cmd((char *)&cm, 6, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to play tone at hz %d duration %d\n", hz, duration);
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}

int keep_alive() {

	unsigned char cm[2] = { 0x00, 0x0D };
	char *rbuf;
	int rlen, cmret;
	unsigned int sleep = 0;

	if (dodebug) {
		printf("keep_alive:\n");
	}

	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to stop sound playback\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	memcpy(&sleep, &rbuf[3], 4);
	if (dodebug) {
		printf("keep_alive: %d\n", (unsigned int)sleep);
	}

	free(rbuf);
	return((int)sleep);
}

char *get_current_program_name() {

	unsigned char cm[2] = { 0x00, 0x11 };
	char *rbuf;
	int rlen, cmret;
	unsigned char fname[19];
	char *fret = NULL;

	if (dodebug) {
		printf("get_current_program_name:\n");
	}
	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to get current program name\n");
		if (rbuf) free(rbuf);
		return(NULL);
	}

	memset(&fname, 0, 19);
	memcpy(&fname, &rbuf[3], 19);
	fret = (char *) strdup(fname);

	free(rbuf);
	return(fret);
}

int set_brick_name(char *name) {

	unsigned char cm[18];
	char *rbuf;
	int rlen, cmret;

	if (dodebug) {
		printf("set_brick_name %s:\n", name);
	}

	memset(&cm[0], 0, 18);
	cm[0] = (unsigned char) 0x01;
	cm[1] = (unsigned char) 0x98;
	memcpy(&cm[2], name, strlen(name));

	cmret = _do_cmd((char *)&cm, 18, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to set the brick name\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}

/* I have no idea what this does */
int delete_user_flash() {

	unsigned char cm[2] = { 0x01, 0xA0 };
	char *rbuf;
	int rlen, cmret;
	unsigned char ret;

	if (dodebug) {
		printf("delete_user_flash:\n");
	}
	cmret = _do_cmd((char *)&cm, 2, (char **)&rbuf, (int *)&rlen, 2);
	ret = (unsigned char) rbuf[0];
	free(rbuf);
	if (cmret) {
		fprintf(stderr, "Failed to delete user flash\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	if (rbuf) free(rbuf);
	return(0);
}
