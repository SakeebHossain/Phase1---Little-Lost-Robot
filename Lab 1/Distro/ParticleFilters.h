/*
  Particle filters implementation for a simple robot.

  Header file. Note you may need to modify sections of
  this file. Read the comments in ParticleFilters.c
  and complete all required work.

  Written by F.J.E. for CSC C85, May 2012
*/

#ifndef __ParticleFilters_header
#define __ParticleFilters_header

#include<stdio.h>
#include<stdlib.h>
#include<math.h>

// OpenGL libraries for image display
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "ParticleUtils.h"

// Particle Filter functions

// Initilization and setup
int main(int argc, char *argv[]);		
// Particle initialization
void initParticles(void);			
// Compute likelihood for a particle
void computeLikelihood(struct particle *p, struct particle *rob, double noise_sigma);
// Particle resampling
struct particle *resample(void);		
// Main loop
void ParticleFilterLoop(void);

// OpenGL functions - you DO NOT need to modify or read these
void initGlut(char* winName);
void WindowReshape(int w, int h);
void kbHandler(unsigned char key, int x, int y);

#endif
