/*****************************************************************
 CSC C85 - Fall 2013 - UTSC RoboSoccer Robot Control API

 This file contains the implementation of the robot control API

 You *MUST NOT* modify this file or the way the API works, but
 you have to be familiar with the available controls so you know
 what to use within your AI core.

 For a detailed description of the API, please review the
 README.TXT within the API's directory.

 Robot control API by David Szeto - Summer 2013
******************************************************************/
#include "robotControl.h"
#include <stdio.h>

#define round(x) ((int)(x < 0 ? x - 0.5 : x + 0.5)) //used for rounding a double/float X into an int

//The base variants of all functions drive at DEFAULT_SPEED. Variants of functions with _speed just mean that you can control the speed instead of flooring it.
//TO DO: Decide what to do with sensors

/*speed range is [-100, 100]
*/

//general driving
int drive() {
	return drive_speed(DEFAULT_SPEED);
}
int drive_speed (int speed){
	return drive_custom(speed, speed);
}
int reverse(){
	return reverse_speed(DEFAULT_SPEED);
}
int reverse_speed(int speed){
	return drive_speed(speed * (-1));
}
int all_stop(){
	return drive_speed(0);
}

//functions for turning left
int pivot_left(){
	return pivot_left_speed(DEFAULT_SPEED);
}
int pivot_left_speed(int speed){
	return drive_custom(-speed, speed);
}
int turn_left(){
	return turn_left_speed(DEFAULT_SPEED);
}
int turn_left_speed(int speed){
	return drive_custom(0, speed);
}
int turn_left_reverse(){
	return turn_left_reverse_speed(DEFAULT_SPEED);
}
int turn_left_reverse_speed(int speed){
	return drive_custom(-speed, 0);
}

//functions for turning right
int pivot_right(){
	return pivot_right_speed(DEFAULT_SPEED);
}
int pivot_right_speed(int speed){
	return drive_custom(speed, -speed);
}
int turn_right(){
	return turn_right_speed(DEFAULT_SPEED);
}
int turn_right_speed(int speed){
	return drive_custom(speed, 0);
}
int turn_right_reverse(){
	return turn_right_reverse_speed(DEFAULT_SPEED);
}
int turn_right_reverse_speed(int speed){
	return drive_custom(0, -speed);
}

//kick
int kick(){
	return kick_speed(MAX_SPEED);
}
int kick_speed(int speed){
	return NXT_OnFwd(OUT_A, speed * DIRECTION_KICKER);
}
int stop_kicker(){
	return kick_speed(0);
}
int retract(){
	return retract_speed(MAX_SPEED);
}
int retract_speed(int speed){
	return kick_speed(-speed);
}

//Directly control the motors with numerical speeds. Use sparingly!
int drive_custom (int left_speed, int right_speed){
	if (NXT_OnFwd(OUT_B, round(right_speed * DIRECTION * POWER_FACTOR_RIGHT)) < 0){
		fprintf(stderr, "_set_output_state Failed\n");
		return -1;
	}
	if (NXT_OnFwd(OUT_C, round(left_speed * DIRECTION * POWER_FACTOR_LEFT)) < 0){
		fprintf(stderr, "_set_output_state Failed\n");
		return -1;
	}
	return 1;
}
