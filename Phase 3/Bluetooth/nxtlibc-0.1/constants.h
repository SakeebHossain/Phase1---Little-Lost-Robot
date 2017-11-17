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

#define NXT_RET		0x0  /* direct command, reply required */
#define NXT_SYS		0x1  /* system command, reply required */
#define NXT_REPLY	0x2  /* reply command */
#define NXT_NORET	0x80 /* Direct command, reply not required */
#define NXT_SYSOP	0x81 /* System command, reply not required */

#define POLL_BUFFER	0x0 /* poll buffer */
#define HS_BUFFER	0x1 /* high speed buffer */

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
#define MOTOR_RUN_STATE_RAMPDOWN	0x20 /* output will ramp down */

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
