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

#ifndef _NXTLIBC_H
#define _NXTLIBC_H

struct fstruct {
	char *filename;
	unsigned int size;
	int moduleid;
	int module;
	struct fstruct *next;
	};

/*  convert integer values to bytes for power settings
	assume to be not greater than 100(max power setting for motors) */
#define INT_TO_HEX(x) ( (x & 0xF0) + (x & 0x0F) )

#define NXT_RET		0x0  /* direct command, reply required */
#define NXT_SYS		0x1  /* system command, reply required */
#define NXT_REPLY	0x2  /* reply command */
#define NXT_NORET	0x80 /* Direct command, reply not required */
#define NXT_SYSOP	0x81 /* System command, reply not required */

#define POLL_BUFFER	0x0 /* poll buffer */
#define HS_BUFFER	0x1 /* high speed buffer */

/* bytes for motor selection */
#define OUT_A 	0x00
#define OUT_B 	0x01
#define OUT_C 	0x02
#define OUT_AB	0x03
#define OUT_AC	0x04
#define OUT_BC	0x05
#define OUT_ALL 0xFF

#define NXT_SENSOR1	0x0
#define NXT_SENSOR2	0x1
#define NXT_SENSOR3	0x2
#define NXT_SENSOR4	0x3

/** motor stuff for set_output_state **/

#define MODE_MOTORON	0x01 /* turn on specified motor */
#define MODE_BRAKE	0x02 /* use run/brake instead of run/float in PWM */
#define MODE_REGULATED	0x04 /* turns on the regulation */

#define REGULATION_MODE_IDLE		0x0 /* no regulation will be enabled */
#define REGULATION_MODE_MOTOR_SPEED	0x01 /* power control will be enabled on 						specified output */
#define REGULATION_MODE_MOTOR_SYNC	0x02 /* synchronization will be enabled 						(needs enabled on two output */

#define MOTOR_RUN_STATE_IDLE		0x0  /* output will be idle */
#define MOTOR_RUN_STATE_RAMPUP 		0x10 /* output will ramp up */
#define MOTOR_RUN_STATE_RUNNING		0x20 /* output will be running */
#define MOTOR_RUN_STATE_RAMPDOWN	0x40 /* output will ramp down */

/* for set_input_mode */
/* sensor type */
#define NO_SENSOR	0x0
#define SWITCH		0x1
#define TEMPERATURE	0x2
#define REFLECTION	0x3
#define ANGLE		0x4
#define LIGHT_ACTIVE	0x5
#define LIGHT_INACTIVE	0x6
#define SOUND_DB	0x7
#define SOUND_DBA	0x8
#define CUSTOM		0x9
#define LOWSPEED	0xA
#define LOWSPEED_9V	0xB
#define NO_OF_SENSOR_TYPES 0xC

/* sensor mode */
#define RAWMODE			0x0  /* 0 - 1023 */
#define BOOLEANMODE		0x20 /* 0 - 1 */
#define TRANSITIONCNTMODE	0x40 /* 0 - 65535 */
#define PERIODCOUNTERMODE	0x60 /* 0 - 65535 */
#define PCTFULLSCALEMODE	0x80 /* 0 - 100 */
#define CELSIUSMODE		0xA0 /* -200, 700 (readings in 10th of a deg)*/
#define FAHRENHEITMODE		0XC0 /* -400, 1580 (ditto) */
#define ANGLESTEPSMODE		0xE0 /* 0 - 65535 */
#define SLOPEMASK		0x1F
#define MODEMASK		0xE0

/* module debugging */
#define MODULE_CMD	0x1
#define MODULE_OUTPUT	0x2
#define MODULE_INPUT	0x3
#define MODULE_BUTTON	0x4
#define MODULE_COMM	0x5
#define MODULE_IOCTRL	0x6
#define MODULE_LED	0x7
#define MODULE_SOUND	0x8
#define MODULE_LOADER	0x9
#define MODULE_DISPLAY	0xA
#define MODULE_LOWSPEED	0xB
#define MODULE_UI	0xC

#define DISPLAY_WIDTH	100
#define DISPLAY_HEIGHT	64

#define	FILE_NORMAL		0
#define FILE_LINEAR		1
#define FILE_DATA		2
#define FILE_APPEND_DATA	3


/* bluetooth.c */
extern int _do_cmd(char *, int, char **, int *, int);
extern int _do_connect(char *);
extern void _do_close();
extern char *_nxt_error(unsigned char);
extern void debug_buf(char *, int);

/* file.c */
extern int _close_command(unsigned char);
extern unsigned char _find_first(char *);
extern int _find_next(unsigned char);
extern int list_files(char *);
extern unsigned char _open_read(char *, int *);
extern int _read_command(unsigned char, char *, short int);
extern int read_file(char *, char **, int *);
extern unsigned char _open_write(char *, int);
extern int _write_command(unsigned char, char *, int);
extern int write_file(int, char *, char *, int);
extern int delete_file(char *);


/* nxt.c */
extern int nxt_bluetooth_initialize(char *);
extern int get_battery_level();
extern char *get_firmware_version();
extern int _get_device_info(char **, int *, int *);
extern int get_bluetooth_signal_strength();
extern int start_program(char *);
extern int stop_program();
extern int play_sound_file(int, char *);
extern int stop_sound_playback();
extern int play_tone(short int, short int);
extern int keep_alive();
extern int set_brick_name(char *);
extern int delete_user_flash();
extern void nxt_bluetooth_done();

/* iomap.c */
extern int _close_module_handle(unsigned char);
extern unsigned char _request_first_module(char *);
extern int _request_next_module(unsigned char);
extern void list_modules(char *);
extern int get_module_id(int);
extern int read_io_map(int, unsigned short int, unsigned short int, unsigned char **);
extern int write_io_map(int, unsigned short int, unsigned short int, unsigned char *);

/* motors.c */
extern int _set_output_state(unsigned char, signed char, unsigned char, unsigned char, signed char, unsigned char, unsigned long);
extern int _get_output_state(unsigned char, signed char *, unsigned char *, unsigned char *, signed char *, unsigned char *, unsigned long *, int *, int *, int *);
extern int reset_motor_position(unsigned char,  unsigned char);

/* motorControl.c*/
extern int NXT_OnFwd(unsigned char, int);
extern int NXT_Off(unsigned char);
extern int NXT_PivotTurn(int);
extern int NXT_SyncTurn(int,int);

/* poll.c */
extern int poll_command_length(unsigned char);
extern int poll_command(unsigned char, unsigned char, char **, int *);

/* sensors.c */
extern int _set_input_mode(unsigned char, unsigned char, unsigned char);
extern int _get_input_values(unsigned char, unsigned char *, unsigned char *, unsigned char *, unsigned char *, unsigned short int *, unsigned short int *, signed short int *, signed short int *);
extern int reset_input_scaled_value(unsigned char);
extern void set_touch_sensor(unsigned char);
extern void set_sound_sensor(unsigned char);
extern void set_light_sensor(unsigned char);
extern void set_ultrasonic_sensor(unsigned char);
extern int get_touch_value();
extern int get_sound_db_pct();
extern int get_sound_db_scale();
extern int get_sound_dba_pct();
extern int get_sound_dba_scale();
extern int get_light_active_pct();
extern int get_light_active_scale();
extern int get_light_ambient_pct();
extern int get_light_ambient_scale();
extern int get_light_reflection();

/* sensors-ls.c */
extern int _ls_get_status(unsigned char);
extern int _ls_write(unsigned char, unsigned char, unsigned char, char *);
extern int _ls_read(unsigned char, int *, char **);
extern int _ls_request_response(unsigned char, unsigned char, char *, int, char **, int *);
extern int initialize_ultrasonic_sensor();
extern int get_ultrasonic_measurement_units();
extern int get_ultrasonic_measurement_byte(unsigned char byte);
extern int get_ultrasonic_continuous_measurement_interval();
extern int get_ultrasonic_read_command_state();
extern int get_ultrasonic_actual_zero();
extern int get_ultrasonic_actual_scale_factor();
extern int get_ultrasonic_actual_scale_divisor();
extern int set_ultrasonic_single_shot();
extern int set_ultrasonic_continuous_measurement();
extern int set_ultrasonic_event_capture_mode();
extern int ultrasonic_request_warm_reset();
extern int set_ultrasonic_continuous_measurement_interval(unsigned char);
extern int set_ultrasonic_actual_zero(unsigned char);
extern int set_ultrasonic_actual_scale_factor(unsigned char);
extern int set_ultrasonic_actual_scale_divisor(unsigned char);
extern int set_ultrasonic_off();

/* screen.c */
extern int clear_screen();
extern int draw_pixel(unsigned char, unsigned char);
extern int clear_pixel(unsigned char, unsigned char);
extern int draw_line(unsigned char, unsigned char, unsigned char, unsigned char);

/* file-linear.c */
extern int open_read_linear(char *);
extern unsigned char _open_write_linear(char *, int);
extern unsigned char _open_write_data(char *, int);
extern unsigned char _open_append_data(char *, int);

/* msg.c */
extern int message_write(unsigned char, unsigned char, unsigned char *);
extern int message_read(unsigned char, unsigned char, unsigned char, unsigned char *, unsigned char *, unsigned char **);

#endif
