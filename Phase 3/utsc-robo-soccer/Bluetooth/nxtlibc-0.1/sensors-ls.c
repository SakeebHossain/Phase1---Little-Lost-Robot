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
extern int ultrasonic_sensor;

int _ls_get_status(unsigned char port) {

	unsigned char cm[3] = { 0x0, 0x0E, port };
	char *rbuf;
	int *rlen, cmret;
	unsigned char x;
	int x2;

	if (dodebug) {
		printf("_ls_get_status:\n");
	}

	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _ls_get_status command\n");
		if (rbuf) free(rbuf);
		return(-1);
        }

	x = (unsigned char) rbuf[3];
	x2 = x;
	free(rbuf);
	return(x2);
}

int _ls_write(unsigned char port, unsigned char txlen, unsigned char rxlen, char *txdata) {

	unsigned char cm[30];
	char *rbuf;
	int *rlen, cmret;

	if (dodebug) {
		printf("_ls_write:\n");
	}

	cm[0] = (unsigned char) 0x0;
	cm[1] = (unsigned char) 0x0F;
	cm[2] = (unsigned char) port;
	cm[3] = (unsigned char) txlen;
	cm[4] = (unsigned char) rxlen;
	memcpy(&cm[5], txdata, (int)txlen);

	cmret = _do_cmd((char *)&cm, 5+(int)txlen, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _ls_write command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	free(rbuf);
	return(0);
}


int _ls_read(unsigned char port, int *len, char **buf) {

	unsigned char cm[3] = { 0x0, 0x10, port };
	char *rbuf;
	int *rlen, cmret;
	unsigned char mylen;

	if (dodebug) {
		printf("_ls_read:\n");
	}
	cmret = _do_cmd((char *)&cm, 3, (char **)&rbuf, (int *)&rlen, 2);
	if (cmret) {
		fprintf(stderr, "Failed to run _ls_write command\n");
		if (rbuf) free(rbuf);
		return(-1);
	}

	mylen = (unsigned char) rbuf[3];
	*len = mylen;
	if (dodebug) {
		printf("ls_read mylen %d len %d\n", mylen, *len);
	}
	*buf = malloc(mylen);
	memcpy(*buf, &rbuf[4], mylen);

	free(rbuf);
	return(0);
}


int _ls_request_response(unsigned char txlen, unsigned char rxlen, char *cmd, int cmdlen,  char **mbuf, int *mlen) {

	unsigned char cm[50];
	int ret;
	char *buf;
	int len;

	if (dodebug) {
		printf("_ls_request_response:\n");
	}

	memcpy(&cm[0], cmd, cmdlen);
	ret = _ls_write(ultrasonic_sensor, txlen, rxlen, (char *)&cm);
	if (ret == -1) {
		fprintf(stderr, "_ls_request_response failed\n");
		return(-1);
	}

	ret = 0;
	while (ret != rxlen) {
		ret = _ls_get_status(ultrasonic_sensor);
	}

	ret = _ls_read(ultrasonic_sensor, &len, (char **)&buf);

	if (dodebug) {
		printf("gumu len %d: ", len);
	}
	*mlen = len;
	*mbuf = (char *) malloc(len);
	memcpy(*mbuf, buf, len);

	if (dodebug) {
		for (ret = 0; ret < len; ret++) {
			printf("%02x ", (unsigned char )buf[ret]);
		}
		printf("\n");
	}

	return(0);
}

int initialize_ultrasonic_sensor() {

	int ret;

	if (dodebug) {
		printf("initialize_ultrasonic_sensor:\n");
	}

	if (ultrasonic_sensor == 255) {
		fprintf(stderr, "haven't set the ultrasonic port\n");
		return(-1);
	}

	ret = _set_input_mode(ultrasonic_sensor, LOWSPEED_9V, RAWMODE);
	return(ret);
}

/* FIX, what the heck does this return? it's different every time */
int get_ultrasonic_measurement_units() {

	unsigned char cm[2] = { 0x02, 0x14 };
	char *buf;
	int len, ret;

	if (dodebug) {
		printf("get_ultrasonic_measurement_units:\n");
	}

	ret = _ls_request_response(2, 7, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_measurement_units failed\n");
		return(ret);
	}

	if (dodebug) {
		printf("2nd len %d: ", len);
		for (ret = 0; ret < len; ret++) {
			printf("%02x ", (unsigned char )buf[ret]);
		}
		printf("\n");
	}

	if (buf) free(buf);
	return(0);
}

/*
 * Returns the distance reading from the NXT from register $byte.
 * $byte should be a value 0-7 indicating the measurement register
 * in the ultrasound sensor. In continuous measurement mode,
 * measurements are stored in register 0 only, however in one-shot mode,
 * each time one-shot is called a value will be stored in a new register.
 */

int get_ultrasonic_measurement_byte(unsigned char byte) {

	unsigned char cm[2] = { 0x02, 0x42 + byte };
	char *buf;
	int len, ret;
	unsigned char bla;

	if (dodebug) {
		printf("get_ultrasonic_measurement_byte:\n");
	}

	ret = _ls_request_response(2, 1, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_measurement_byte failed\n");
		return(ret);
	}
	bla = (unsigned char) buf[0];

	if (dodebug) {
		for (ret = 0; ret < len; ret++) {
			printf("%02x ", (unsigned char )buf[ret]);
		}
		printf("\n");
	}

	if (buf) free(buf);
	return((int)bla);
}

/* is this milliseconds? */
/* Returns time period betweeen ultrasonic measurements */
int get_ultrasonic_continuous_measurement_interval() {

	unsigned char cm[2] = { 0x02, 0x40 };
	char *buf;
	int len, ret;
	unsigned char bla;

	if (dodebug) {
		printf("get_ultrasonic_continuous_measurement_interval:\n");
	}

	ret = _ls_request_response(2, 1, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_continuous_measurement_interval failed\n");
		return(ret);
	}

	bla = (unsigned char) buf[0];
	if (buf) free(buf);
	return((int) bla);
}

/* returns whether the sensor is in one-off mode or continuous measmurement
 * mode (the default)
 */
int get_ultrasonic_read_command_state() {

	unsigned char cm[2] = { 0x02, 0x41 };
	char *buf;
	int *len, ret;
	unsigned char bla;

	if (dodebug) {
		printf("get_ultrasonic_read_command_state:\n");
	}

	ret = _ls_request_response(2, 1, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_read_command_state failed\n");
		return(ret);
	}

	bla = (unsigned char) buf[0];
	if (buf) free(buf);
	return((int)bla);
}

/* returns the calibrated zero-distance value for the sensor */
int get_ultrasonic_actual_zero() {

	unsigned char cm[2] = { 0x02, 0x50 };
	char *buf;
	int *len, ret;
	unsigned char bla;

	if (dodebug) {
		printf("get_ultrasonic_actual_zero:\n");
	}

	ret = _ls_request_response(2, 1, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_actual_zero failed\n");
		return(ret);
	}

	bla = (unsigned char) buf[0];
	if (buf) free(buf);
	return((int)bla);
}

/* returns the scale factor used to compute distances */
int get_ultrasonic_actual_scale_factor() {

	unsigned char cm[2] = { 0x02, 0x51 };
	char *buf;
	int *len, ret;
	unsigned char bla;

	if (dodebug) {
		printf("get_ultrasonic_actual_scale_factor:\n");
	}

	ret = _ls_request_response(2, 1, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_actual_scale_factor failed\n");
		return(ret);
	}

	bla = (unsigned char) buf[0];
	if (buf) free(buf);
	return((int)bla);
}

/* returns the scale divisor used to compute distances */
int get_ultrasonic_actual_scale_divisor() {

	unsigned char cm[2] = { 0x02, 0x52 };
	char *buf;
	int *len, ret;
	unsigned char bla;

	if (dodebug) {
		printf("get_ultrasonic_actual_scale_divisor:\n");
	}

	ret = _ls_request_response(2, 1, (char *)&cm, 2, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "get_ultrasonic_actual_scale_divisor failed\n");
		return(ret);
	}

	bla = (unsigned char) buf[0];
	if (buf) free(buf);
	return((int)bla);
}

/* puts sensor in single shot mode, it will only store a value in a register
 * once each time this function is called 
 */
int set_ultrasonic_single_shot() {

	unsigned char cm[3] = { 0x02, 0x41, 0x01 };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_single_shot:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_single_shot failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* puts the sensor in continuous measurement mode */
int set_ultrasonic_continuous_measurement() {

	unsigned char cm[3] = { 0x02, 0x41, 0x02 };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_continuous_measurement:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_continuous_measurement failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* ta in this mode the US sensor will detect only other ultrasonic sensors in
 * the vicinity
 */
int set_ultrasonic_event_capture_mode() {

	unsigned char cm[3] = { 0x02, 0x41, 0x03 };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_event_capture_mode:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_event_capture_mode failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* I don't know what this is, maybe it's a warm reset? */
int ultrasonic_request_warm_reset() {

	unsigned char cm[3] = { 0x02, 0x41, 0x04 };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("ultrasonic_request_warm_reset:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_warm_reset failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}


/*
 * sets the sampling interval for the range sensor
 */
int set_ultrasonic_continuous_measurement_interval(unsigned char interval) {

	unsigned char cm[3] = { 0x02, 0x40, interval };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_continuous_measurement_interval:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_continuous_measurement_interval failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* sets the calibrated zero value for the sensor */
int set_ultrasonic_actual_zero(unsigned char zero) {

	unsigned char cm[3] = { 0x02, 0x50, zero };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_actual_zero:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_actual_zero failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* sets the scale factor used in computing range */
int set_ultrasonic_actual_scale_factor(unsigned char scale) {

	unsigned char cm[3] = { 0x02, 0x51, scale };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_actual_scale_factor:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_actual_scale_factor failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* sets the scale divisor used in computing range */
int set_ultrasonic_actual_scale_divisor(unsigned char scale) {

	unsigned char cm[3] = { 0x02, 0x52, scale };
	char *buf;
	int *len, ret;

	if (dodebug) {
		printf("set_ultrasonic_actual_scale_divisor:\n");
	}

	ret = _ls_request_response(3, 0, (char *)&cm, 3, (char **)&buf, (int *)&len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_actual_scale_divisor failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}

/* turn off the ultrasonic sensor */
int set_ultrasonic_off() {

	unsigned char cm[3] = { 0x02, 0x41, 0x0 };
	char *buf;
	int len, ret;

	if (dodebug) {
		printf("set_ultrasonic_off:\n");
	}

	ret = _ls_request_response(3, 0, (unsigned char *)&cm, 3, (char **)&buf, &len);
	if (ret == -1) {
		if (buf) free(buf);
		fprintf(stderr, "set_ultrasonic_off failed\n");
		return(ret);
	}

	if (buf) free(buf);
	return(0);
}
