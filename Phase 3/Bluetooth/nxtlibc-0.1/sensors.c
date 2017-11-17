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

int touch_sensor = 255;
int sound_sensor = 255;
int light_sensor = 255;
int ultrasonic_sensor = 255;
int lastsensor = 255;
int lasttype = 255;


int _set_input_mode(unsigned char port, unsigned char type, unsigned char mode) {

	unsigned char cm[5] = { 0x0, 0x05, port, type, mode };
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_set_input_mode:\n");
	}
	cmret = _do_cmd((char *)&cm, 5, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _set_output_state command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	free(rbuf);
	return(0);
}

int _get_input_values(unsigned char port, unsigned char *valid, unsigned char *calibrated, unsigned char *sensortype, unsigned char *sensormode, unsigned short int *rawad, unsigned short int *normalad, signed short int *scaled, signed short int *calibratedval) {

	unsigned char cm[3] = { 0x0, 0x07, port };
	char *rbuf;
	int *rlen, cmret;
	unsigned short int t1;
	signed short int t2;

	if (dodebug) {
		printf("_get_input_values:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _set_output_state command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	*valid = (unsigned char) rbuf[4];
	*calibrated = (unsigned char) rbuf[5];
	*sensortype = (unsigned char) rbuf[6];
	*sensormode = (unsigned char) rbuf[7];

	memcpy(&t1, &rbuf[8], 2);
	*rawad = (unsigned short int) t1;
	if (dodebug) {
		printf("rawad %d\n", (unsigned short int) t1);
	}
	memcpy(&t1, &rbuf[10], 2);
	*normalad = (unsigned short int) t1;
	if (dodebug) {
		printf("normalad %d\n", (unsigned short int) t1);
	}
	memcpy(&t2, &rbuf[12], 2);
	*scaled = (signed short int) t2;
	if (dodebug) {
		printf("scaled %d\n", (signed short int) t2);
	}
	memcpy(&t2, &rbuf[14], 2);
	*calibratedval = (signed short int) t2;
	if (dodebug) {
		printf("calibratedval %d\n", (signed short int) t2);
	}

	free(rbuf);
	return(0);
}

void set_touch_sensor(unsigned char port) {
	touch_sensor = port;
}

void set_sound_sensor(unsigned char port) {
	sound_sensor = port;
}

void set_light_sensor(unsigned char port) {
	light_sensor = port;
}

void set_ultrasonic_sensor(unsigned char port) {
	ultrasonic_sensor = port;
}

int reset_input_scaled_value(unsigned char port) {

	unsigned char cm[3] = { 0x0, 0x08, port };
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("reset_input_scaled_value:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
	        fprintf(stderr, "Failed to run _set_output_state command\n");
	        if (rbuf) free(rbuf);
	        return(-1);
	}

	free(rbuf);
	return(0);
}


int get_touch_value() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_touch_value:\n");
	}

	if (touch_sensor == 255) {
		fprintf(stderr, "you have not set the button sensor\n");
		return(-1);
	}

	if (lastsensor != touch_sensor) {
		_set_input_mode(touch_sensor, 1, BOOLEANMODE);
		lastsensor = touch_sensor;
	}

	ret = _get_input_values(touch_sensor, (unsigned char *) &valid, (unsigned char *)&calibrated, (unsigned char *) &sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)scaled);
}

int get_sound_db_pct() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_sound_db_pct:\n");
	}

	if (sound_sensor == 255) {
		fprintf(stderr, "you have not set the sound sensor\n");
		return(-1);
	}

	if ((lastsensor != sound_sensor) && (lasttype != 1)) {
		_set_input_mode(sound_sensor, 0x7, PCTFULLSCALEMODE);
		lastsensor = sound_sensor;
		lasttype = 1;
	}

	ret = _get_input_values(sound_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)scaled);
}

/* returns 0 - 1023 */
int get_sound_db_scale() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_sound_db_scale:\n");
	}

	if (sound_sensor == 255) {
		fprintf(stderr, "you have not set the sound sensor\n");
		return(-1);
	}

	if ((lastsensor != sound_sensor) && (lasttype != 1)) {
		_set_input_mode(sound_sensor, 0x7, PCTFULLSCALEMODE);
		lastsensor = sound_sensor;
		lasttype = 1;
	}

	ret = _get_input_values(sound_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)calibratedval);
}

/* 1-100 */
int get_sound_dba_pct() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_sound_dba_pct:\n");
	}

	if (sound_sensor == 255) {
		fprintf(stderr, "you have not set the sound sensor\n");
		return(-1);
	}

	if ((lastsensor != sound_sensor) && (lasttype != 2)) {
		_set_input_mode(sound_sensor, 0x8, PCTFULLSCALEMODE);
		lastsensor = sound_sensor;
		lasttype = 2;
	}

	ret = _get_input_values(sound_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *) &sensortype, (unsigned char *) &sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)scaled);
}

int get_sound_dba_scale() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_sound_dba_scale:\n");
	}

	if (sound_sensor == 255) {
		fprintf(stderr, "you have not set the sound sensor\n");
		return(-1);
	}

	if ((lastsensor != sound_sensor) && (lasttype != 2)) {
		_set_input_mode(sound_sensor, 0x8, PCTFULLSCALEMODE);
		lastsensor = sound_sensor;
		lasttype = 2;
	}

	ret = _get_input_values(sound_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *) &sensortype, (unsigned char *) &sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)calibratedval);
}

/* light_active with floodlight enabled
 * light_inactive with floodlight disabled
 * percentage reading
 * 0 - 1023 reading ?
 * passive with light off
 * active with light on
 * TRY: reflection
 */

int get_light_active_pct() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_light_active_pct:\n");
	}

	if (light_sensor == 255) {
		fprintf(stderr, "you have not set the light sensor\n");
		return(-1);
	}

	if ((lastsensor != light_sensor) && (lasttype != 3)) {
		_set_input_mode(light_sensor, 0x5, PCTFULLSCALEMODE);
		lastsensor = light_sensor;
		lasttype = 3;
	}

	ret = _get_input_values(light_sensor, (unsigned char *) &valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)scaled);
}

int get_light_active_scale() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_light_active_scale:\n");
	}

	if (light_sensor == 255) {
		fprintf(stderr, "you have not set the light sensor\n");
		return(-1);
	}

	if ((lastsensor != light_sensor) && (lasttype != 3)) {
		_set_input_mode(light_sensor, 0x5, PCTFULLSCALEMODE);
		lastsensor = light_sensor;
		lasttype = 3;
	}

	ret = _get_input_values(light_sensor, (unsigned char *) &valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)calibratedval);
}

int get_light_ambient_pct() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_light_sensor_inactive:\n");
	}

	if (light_sensor == 255) {
		fprintf(stderr, "you have not set the light sensor\n");
		return(-1);
	}

	if ((lastsensor != light_sensor) && (lasttype != 4)) {
		_set_input_mode(light_sensor, 0x6, PCTFULLSCALEMODE);
		lastsensor = light_sensor;
		lasttype = 3;
	}

	ret = _get_input_values(light_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)scaled);
}

int get_light_ambient_scale() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_light_sensor_inactive:\n");
	}

	if (light_sensor == 255) {
		fprintf(stderr, "you have not set the light sensor\n");
		return(-1);
	}

	if ((lastsensor != light_sensor) && (lasttype != 4)) {
		_set_input_mode(light_sensor, 0x6, PCTFULLSCALEMODE);
		lastsensor = light_sensor;
		lasttype = 3;
	}

	ret = _get_input_values(light_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return((int)calibratedval);
}

/* the source code mentions with REFLECTION
 * something about MENU_SENSOR_LIGHT_OLD
 * and this function doesn't work, but it seems to work
 * on the nxt menu
 */
int get_light_reflection() {

	unsigned char valid, calibrated, sensortype, sensormode;
	unsigned short int rawad, normalad;
	signed short int scaled, calibratedval;
	int ret;

	if (dodebug) {
		printf("get_light_reflection:\n");
	}

	if (light_sensor == 255) {
		fprintf(stderr, "you have not set the light sensor\n");
		return(-1);
	}

	if ((lastsensor != light_sensor) && (lasttype != 5)) {
		printf("in here\n");
		_set_input_mode(light_sensor, 0x3, RAWMODE);
		lastsensor = light_sensor;
		lasttype = 3;
	}

	ret = _get_input_values(light_sensor, (unsigned char *)&valid, (unsigned char *)&calibrated, (unsigned char *)&sensortype, (unsigned char *)&sensormode, (unsigned short int *) &rawad, (unsigned short int *) &normalad, (signed short int *)&scaled, (signed short int* )&calibratedval);

	if (ret == -1) {
		return(-1);
	}

	if (dodebug) {
		printf("valid %d ", (unsigned char) valid);
		printf("calibrated %d ", (unsigned char) calibrated);
		printf("sensortype %d ", (unsigned char) sensortype);
		printf("sensormode %d ", (unsigned char) sensormode);
		printf("rawad %d ", (unsigned short int) rawad);
		printf("normalizedad %d ", (unsigned short int) normalad);
		printf("scaled %d ", (signed short int) scaled);
		printf("calibrated val %d\n", (signed short int) calibratedval);
	}

	return(0);
}

/** lets try angle somewhere in here */
