#include "nxtlibc.h"

#include <stdio.h>

/* 	Turns on the specified motors with specified power
   	Supported ports are:
   		OUT_A
   		OUT_B
   		OUT_C
   		OUT_BC
  	Power range is [-100,100]
  	Negative power makes motors go in reverse.
*/
int NXT_OnFwd(unsigned char port, int power){

	int ret = 0;
	if ( (port == OUT_A) || (port == OUT_B) || (port == OUT_C) ){
		ret += _set_output_state(port,INT_TO_HEX(power), 0x07,0x00,0x00,0x20,0x00);
	}
	else if (port == OUT_BC){
		ret += _set_output_state(OUT_B,INT_TO_HEX(power), 0x07,0x00,0x00,0x20,0x00);
		ret += _set_output_state(OUT_C,INT_TO_HEX(power), 0x07,0x00,0x00,0x20,0x00);
	}

	if ( ret < 0){
		fprintf(stderr, "_set_output_state Failed\n");
		return -1;
	}
	return ret;
}

/*	Turns off the specified motors with braking*/
int NXT_Off(unsigned char port){
	int ret = 0;
	if ( (port == OUT_A) || (port == OUT_B) || (port == OUT_C) ){
		ret += _set_output_state(port,0x00,0x02,0x00,0x00,0x00,0x00);
	}
	else if (port == OUT_BC){
		ret += _set_output_state(OUT_B,0x00,0x02,0x00,0x00,0x00,0x00);
		ret += _set_output_state(OUT_C,0x00,0x02,0x00,0x00,0x00,0x00);
	}

	if ( ret < 0){
		fprintf(stderr, "_set_output_state Failed\n");
		return -1;
	}
	return ret;
}

/* 	Turns on the spot
	Because motors on the actual robot may be upside down, this
	handles both left and right turns.
	By default with motors being right side up, positive power does
	a right turn and negative power does a left turn
*/
int NXT_PivotTurn(int power){

	int ret = 0;

	ret += _set_output_state(OUT_B,INT_TO_HEX(-power),0x07,0x00,0x00,0x20,0x00);
	ret += _set_output_state(OUT_C,INT_TO_HEX(power),0x07,0x00,0x00,0x20,0x00);

	if ( ret < 0){
		fprintf(stderr, "_set_output_state Failed\n");
		return -1;
	}
	return ret;
}
