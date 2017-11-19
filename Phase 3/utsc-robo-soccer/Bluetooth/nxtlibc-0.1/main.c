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

/*
 * http://www.nabble.com/Bluetooth-Direct-and-System-Commands-t2288117.html
 * http://mynxt.matthiaspaulscholz.eu/tools/index.html
 * http://news.lugnet.com/robotics/nxt/nxthacking/?n=14
 *
 *
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
extern struct fstruct *filefirst;
extern struct fstruct *modulefirst;
typedef unsigned long ULONG;

#define   SCREEN_BITS                   ((ULONG)0xE0000000)  // Executed as 1.
#define   STEPICON_BITS                 ((ULONG)0x1F000000)  // Executed as 2.
#define   BITMAP_BITS                   ((ULONG)0x00F00000)  // Executed as 3.
#define   MENUICON_BITS                 ((ULONG)0x000E0000)  // Executed as 4.
#define   STATUSICON_BITS               ((ULONG)0x0001E000)  // Executed as 5.
#define   SPECIAL_BITS                  ((ULONG)0x00001F00)  // Executed as 6.
#define   TEXTLINE_BITS                 ((ULONG)0x000000FF)  // Executed as 7.


void check_bits(unsigned char *buf, int len) {

	int i;
	ULONG val;

	for (i = 0; i < len; i+=4) {
		memcpy(&val, &buf[i], sizeof(ULONG));
		printf("pos %d val %ld: ", i, val);
		if (SCREEN_BITS & val) printf("screen bit ");
		if (STEPICON_BITS & val) printf("stepicon bit ");
		if (BITMAP_BITS & val) printf("bitmap bit ");
		if (MENUICON_BITS & val) printf("menuicon bit ");
		if (STATUSICON_BITS & val) printf("statusicon bit ");
		if (SPECIAL_BITS & val) printf("special bit ");
		if (TEXTLINE_BITS & val) printf("textline bit ");
		printf("\n");
	}
}


main(int argc, char *argv[]) {

	int ret, fd;
	char *bla, foo[10], foo2[200];
	unsigned char **fla;
	int *blanum;
	int x1, x2, x3, x4, x5, x6, x7, x8, i;
	struct fstruct *fs;

	ret = nxt_bluetooth_initialize("00:16:53:03:96:6B");
	if (ret) {
		printf("Failed to initialize bluetooth\n");
		exit(1);
	}
/*
ret = get_battery_level();
printf("got battery level %d\n", ret);
get_device_info();
start_program("foo");
stop_program();
play_sound_file(0, "! Click.rso");
stop_sound_playback();
play_tone(400, 1000);
keep_alive();
get_current_program_name();

read_file("RPGReader.sys", (char **)&bla, (int *)&blanum);
printf("blanum is %d\n", blanum);

fd = open("rp", O_CREAT | O_WRONLY);
write(fd, bla, blanum);
close(fd);
while (1) {
set_touch_sensor(0);
get_touch_value();
sleep(1);
}
*/
/*
set_ultrasonic_sensor(3);
initialize_ultrasonic_sensor();
set_ultrasonic_continuous_measurement();
get_ultrasonic_measurement_units();
x5 = get_ultrasonic_actual_scale_factor();
x6 = get_ultrasonic_actual_scale_divisor();
printf("x5 %d x6 %d\n", x5, x6);
*/
/*
while (1) {
	x1 = get_ultrasonic_measurement_byte(0);
	printf("x1 is %d\n", x1);
	sleep(1);
	}
	*/
/*
get_ultrasonic_measurement_units();
x2 = get_ultrasonic_continuous_measurement_interval();
x3 = get_ultrasonic_read_command_state();
x4 = get_ultrasonic_actual_zero();
printf("byte %d interval %d state %d zero %d\n", x1, x2, x3, x4);
printf("factor %d divisor %d\n", x5, x6);
*/

	list_files(NULL);
	fs = filefirst;
	while (fs != NULL) {
		printf("%s %d\n", fs->filename, fs->size);
		fs = fs->next;
	}
/*
for (x1 = 0; x1 < 64; x1++) {
	printf("-- reading %d of 100\n", x1 *  64);
	read_io_map(655361, x1*64, 100);
	}
	*/
/*sprintf(foo, "1234567890");*/

/*
clear_screen();

draw_pixel(1, 1);
draw_pixel(2, 2);
draw_pixel(3, 3);
draw_pixel(4, 4);
draw_pixel(40, 50);
draw_pixel(40, 63);
draw_pixel(90, 20);
draw_pixel(99, 20);
draw_pixel(80, 20);

draw_pixel(19, 19);
draw_pixel(20, 20);
draw_pixel(21, 21);
draw_pixel(22, 22);
draw_pixel(23, 23);

draw_pixel(96, 60);
draw_pixel(97, 61);
draw_pixel(98, 62);
draw_pixel(99, 63);

draw_line(9,12, 43, 62);
*/



/*
x2 = read_io_map(655361, 0, 108, (unsigned char **)&fla);
printf("read %d\n", x2);
check_bits(fla, x2);
free(fla);
x2 = read_io_map(655361, 109, 10, (unsigned char **)&fla);
printf("read %d\n", x2);
check_bits(fla, x2);
free(fla);
*/


/*
bla = get_brick_name();
ret = get_bluetooth_signal_strength();
fd = get_free_flash();
printf("brick name %s bs signal strength %d free flash %d\n", bla, ret, fd);
*/
/*
list_modules(NULL);
fs = modulefirst;
while (fs != NULL) {
	printf("%s %d %d\n", fs->filename, fs->moduleid, fs->module);
	fs = fs->next;
	}
*/

/*
x1 = get_module_id(MODULE_DISPLAY);
printf("got x1 %d\n", x1);
*/
	bla = get_firmware_version();
	printf("Return is >%s<\n", bla);

/*
set_touch_sensor(NXT_SENSOR1);
while (1) {
	x1 = get_touch_value();
	printf("x1 %d\n", x1);
	sleep(1);
	}
*/
/*
set_sound_sensor(NXT_SENSOR2);
while (1) {
	x1 = get_sound_dba_pct();
	x2 = get_sound_dba_scale();
	printf("x1 %d x2 %d\n", x1, x2);
	sleep(1);
	}
*/
/*
set_light_sensor(NXT_SENSOR3);
while (1) {
	x1 = get_light_active_pct();
	x2 = get_light_active_scale();
	printf("x1 %d x2 %d\n", x1, x2);
	sleep(1);
	}
*/
/*
set_light_sensor(NXT_SENSOR3);
while (1) {
	x1 = get_light_ambient_pct();
	x2 = get_light_ambient_scale();
	printf("x1 %d x2 %d\n", x1, x2);
	sleep(1);
	}
*/
/*
set_light_sensor(NXT_SENSOR3);
while (1) {
	x1 = get_light_reflection();
	printf("x1 %d\n", x1, x2);
	sleep(1);
	}
	*/

	nxt_bluetooth_done();
	return(0);
}
