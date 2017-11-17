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

#include <math.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "nxtlibc.h"

extern int dodebug;
int screenhandle = 0;

int clear_screen() {
	unsigned char clr[200];

	if (dodebug) {
		printf("clear_screen:\n");
	}

	if (!screenhandle) {
		screenhandle = get_module_id(MODULE_DISPLAY);
		if (screenhandle == -1) return(-1);
	}

	memset(&clr, 0, 200);
	write_io_map(screenhandle, 0, 100, (unsigned char *)&clr); /*will clear everthing but line and works */
	/* write_io_map(screenhandle, 110, 10, &clr); full screen but cant draw */

	write_io_map(screenhandle, 200, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 300, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 400, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 500, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 600, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 700, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 800, 100, (unsigned char *)&clr);
	write_io_map(screenhandle, 900, 100, (unsigned char *)&clr);

	return(0);
}

int draw_pixel(unsigned char x, unsigned char y) {

	unsigned char val;
	unsigned short int pos;
	int len;
	unsigned char *d;

	if (dodebug) {
		printf("draw_pixel:\n");
	}

	if (!screenhandle) {
		screenhandle = get_module_id(MODULE_DISPLAY);
		if (screenhandle == -1) return(-1);
	}

	pos = (((y/8) * DISPLAY_WIDTH) + x) + 119;
	if (dodebug) {
		printf("pos %d\n", pos);
	}
	len = read_io_map(screenhandle, pos, 1, &d);
	if (len != 1) {
		return(-1);
	}

	val = d[0];
	val |= (1 << (y % 8));
	write_io_map(screenhandle, pos, 1, (unsigned char *)&val);
	free(d);
	return(0);
}

int clear_pixel(unsigned char x, unsigned char y) {

	unsigned char *val;
	unsigned short int pos;
	int len,v;

	if (dodebug) {
		printf("clear_pixel:\n");
	}

	if (!screenhandle) {
		screenhandle = get_module_id(MODULE_DISPLAY);
		if (screenhandle == -1) return(-1);
	}

	pos = (((y/8) * DISPLAY_WIDTH) + x) + 119;
	if (dodebug) {
		printf("pos %d\n", pos);
	}
	len = read_io_map(screenhandle, pos, 1, &val);
	if (len != 1) {
		return(-1);
	}
	(*val) &= ~(1 << (y % 8));
	write_io_map(screenhandle, pos, 1, val);
	return(0);
}

/*
 * http://www.cs.unc.edu/~mcmillan/comp136/Lecture6/Lines.html
 */

int draw_line(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1) {

	int dx = x1 - x0;
	int dy = y1 - y0;

	if (dodebug) {
		printf("draw_line:\n");
	}

	if (!screenhandle) {
		screenhandle = get_module_id(MODULE_DISPLAY);
		if (screenhandle == -1) return(-1);
	}

	draw_pixel(x0, y0);
	if (dx != 0) {
		float m = (float) dy / (float) dx;
		float b = y0 - m * x0;
		dx = (x1 > x0) ? 1 : -1;
		while (x0 != x1) {
			x0 += (unsigned char) dx;
			y0 = (unsigned char) round(m*x0+b);
			if (dodebug) { printf("drawing %u %u\n", x0, y0); }
			draw_pixel(x0, y0);
		}
	}
}
