/************************************************************************************
 CSC C85 - UTSC RoboSoccer
 
 Main - this sets up communications with the NXT, creates an AI data structure
        with the appropriate mode, and starts the image processing/AI main loop

        You only need to modify this file to set up your NXT's hex ID. Other than
        that you should *NOT* change anything here.

 roboSoccer.c - by Per Parker, Summer 2013
************************************************************************************/

#include "imagecapture/imageCapture.h"
#include <stdio.h>
#include <GL/glut.h>
#include <nxtlibc/nxtlibc.h>
#include "roboAI.h"

//just uncomment your bot's hex key to compile for your bot, and comment the other ones out.
#ifndef HEXKEY
	#define HEXKEY "00:16:53:0B:A1:4D"	// <--- SET UP YOUR NXT's HEX ID here
#endif

int main(int argc, char **argv)
{

  if (argc<4||(atoi(argv[2])>1||atoi(argv[2])<0)||(atoi(argv[3])>2||atoi(argv[3])<0))
  {
   fprintf(stderr,"roboSoccer: Incorrect number of parameters.\n");
   fprintf(stderr,"USAGE: roboSoccer video_device own_colour mode\n");
   fprintf(stderr,"  video_device - path to camera (typically /dev/video0 or /dev/video1)\n");
   fprintf(stderr,"  own_colour - colour of the NXT bot controlled by this program, 0 = GREEN, 1 = BLUE\n");
   fprintf(stderr,"  mode - AI mode: 0 = SOCCER, 1 = PENALTY, 2 = CHASE\n");
   exit(0);
  }

  //Connect to device
  //TODO read from config
  nxt_bluetooth_initialize(HEXKEY);

  // Start GLUT
  glutInit(&argc, argv);

  // Launch imageCapture
  if (imageCaptureStartup(argv[1], 1280, 720, atoi(argv[2]), atoi(argv[3]))) {
    fprintf(stderr, "Couldn't start image capture, terminating...\n");
    exit(0);
  }

  return 0;
}
