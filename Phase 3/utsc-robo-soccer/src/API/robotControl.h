/*****************************************************************
 CSC C85 - Fall 2013 - UTSC RoboSoccer Robot Control API

 This file contains the function headers and constant definitions
 that form part of the robot control API.

 You *MUST NOT* modify this file or the way the API works, but
 you have to be familiar with the available controls so you know
 what to use within your AI core.

 For a detailed description of the API, please review the
 README.TXT within the API's directory.

 Robot control API by David Szeto - Summer 2013
******************************************************************/
#ifndef _ROBOT_CONTROL_H
#define _ROBOT_CONTROL_H

#include <nxtlibc/nxtlibc.h>

//The base variants of all functions drive at max speed. variants of functions with _speed just mean that you can control the speed instead of flooring it.
//TO DO: Decide what to do with sensors

#define MAX_SPEED 100
//default speed to drive motors at
#ifndef DEFAULT_SPEED
    #define DEFAULT_SPEED 100
#endif
//set DIRECTION to -1 if motors are reversed
#ifndef DIRECTION
	#define DIRECTION 1
#endif
//set DIRECTION_KICKER to -1 if kicker is reversed
#ifndef DIRECTION_KICKER
	#define DIRECTION_KICKER 1
#endif
//POWER_FACTOR_RIGHT/LEFT multiplies all inputs to the right/left motor by this value. Used to account for differences in motor power.
#ifndef POWER_FACTOR_RIGHT
	#define POWER_FACTOR_RIGHT 1.0
#endif
#ifndef POWER_FACTOR_LEFT
	#define POWER_FACTOR_LEFT 1.0
#endif

//general driving
int drive(); //drive forwards
int drive_speed (int speed); //drive forwards (positive speed) or backwards (negative speed)
int reverse(); //drive backwards at default speed
int reverse_speed(int speed);
int all_stop(); //stop the motors (ignoring the kicker, of course)

//functions for turning left
int pivot_left(); //pivot left on the spot
int pivot_left_speed(int speed);
int turn_left(); //turn left by keeping left track stationary, but advancing right
int turn_left_speed(int speed);
int turn_left_reverse(); //turn left by keeping right track stationary, but reversing left
int turn_left_reverse_speed(int speed);

//functions for turning right
int pivot_right(); //pivot right on the spot
int pivot_right_speed(int speed);
int turn_right(); //turn right by keeping right track stationary, but advancing left
int turn_right_speed(int speed);
int turn_right_reverse(); //turn right by keeping left track stationary, but reversing right
int turn_right_reverse_speed(int speed);

//kick
int kick(); //drive kicking arm to deliver a kick
int kick_speed(int speed);
int stop_kicker(); //stop kicking arm
int retract(); //retract kicking arm
int retract_speed(int speed);


int drive_custom (int left_speed, int right_speed); //Directly control the driving motors with numerical speeds. Use sparingly; this API is supposed to be higher-level so the AI shouldn't be worrying about this kind of adjustment

#endif
