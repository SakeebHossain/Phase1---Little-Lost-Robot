/***********************************************************
             CSC C85 – Fall 2011

      Inverted Pendulum control exercise.

     Header file - read the .cpp file for more details.

     Written Jul. 26, 2011 by F. Estrada

***********************************************************/

#ifndef __InvertedPendulum_header

#define __InvertedPendulum_header

// OpenGL libraries
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

// C libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

// ***********  FUNCTION HEADER DECLARATIONS ****************

// --------------- OpenGL Stuff ------------------
// Initialization functions
void initGlut(char* winName);
void GL_Settings_Init();

// Callbacks for handling events in glut
void WindowReshape(int w, int h);
void WindowDisplay(void);

// Functions to help draw the object
void drawSquare(float size);
void drawAxisLines(void);

// -------------- Inverted Pendulum stuff ---------

void simulationEngine(void);
void ApplyHorizontalForce(void);

#endif

