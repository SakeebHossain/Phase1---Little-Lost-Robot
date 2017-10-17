/*
  Particle filters implementation for a simple robot.

  Your goal here is to implement a simple particle filter
  algorithm for robot localization.

  A map file in .ppm format is read from disk, the map
  contains empty spaces and walls.

  A simple robot is randomly placed on this map, and
  your task is to write a particle filter algorithm to
  allow the robot to determine its location with
  high certainty.

  You must complete all sections marked

  TO DO:

  NOTE: 2 keyboard controls are provided:

  q -> quit (exit program)
  r -> reset particle set during simulation

  Written by F.J.E. for CSC C85, May 2012. Updated Aug. 15, 2014

  This robot model inspired by Sebastian Thrun's
  model in CS373.
*/

#include "ParticleFilters.h"

/**********************************************************
 GLOBAL DATA
**********************************************************/
unsigned char *map;		// Input map
unsigned char *map_b;		// Temporary frame
struct particle *robot;		// Robot
struct particle *list;		// Particle list
int sx,sy;			// Size of the map image
char name[1024];		// Name of the map
int n_particles;		// Number of particles
int windowID;               	// Glut window ID (for display)
int Win[2];                 	// window (x,y) size
int RESETflag;			// RESET particles

/**********************************************************
 PROGRAM CODE
**********************************************************/
int main(int argc, char *argv[])
{
 /*
   Main function. Usage for this program:

   ParticleFilters map_name n_particles

   Where:
    map_name is the name of a .ppm file containing the map. The map
             should be BLACK on empty (free) space, and coloured
             wherever there are obstacles or walls. Anything not
             black is an obstacle.

    n_particles is the number of particles to simulate in [100, 50000]

   Main loads the map image, initializes a robot at a random location
    in the map, and sets up the OpenGL stuff before entering the
    filtering loop.
 */

 if (argc!=3)
 {
  fprintf(stderr,"Wrong number of parameters. Usage: ParticleFilters map_name n_particles.\n");
  exit(0);
 }

 strcpy(&name[0],argv[1]);
 n_particles=atoi(argv[2]);

 if (n_particles<100||n_particles>50000)
 {
  fprintf(stderr,"Number of particles must be in [100, 50000]\n");
  exit(0);
 }

 fprintf(stderr,"Reading input map\n");
 map=readPPMimage(name,&sx, &sy);
 if (map==NULL)
 {
  fprintf(stderr,"Unable to open input map, or not a .ppm file\n");
  exit(0);
 }

 // Allocate memory for the temporary frame
 fprintf(stderr,"Allocating temp. frame\n");
 map_b=(unsigned char *)calloc(sx*sy*3,sizeof(unsigned char));
 if (map_b==NULL)
 {
  fprintf(stderr,"Out of memory allocating image data\n");
  free(map);
  exit(0);
 }

 srand48((long)time(NULL));		// Initialize random generator from timer
 fprintf(stderr,"Init robot...\n");
 robot=initRobot(map,sx,sy);
 if (robot==NULL)
 {
  fprintf(stderr,"Unable to initialize robot.\n");
  free(map);
  free(map_b);
  exit(0);
 }
 sonar_measurement(robot,map,sx,sy);	// Initial measurements...

 // Initialize particles at random locations
 fprintf(stderr,"Init particles...\n");
 list=NULL;
 initParticles();

 // Done, set up OpenGL and call particle filter loop
 fprintf(stderr,"Entering main loop...\n");
 Win[0]=800;
 Win[1]=800;
 glutInit(&argc, argv);
 initGlut(argv[0]);
 glutMainLoop();

 // This point is NEVER reached... memory leaks must be resolved by OpenGL main loop
 exit(0);

}

//Creates a randomly generated particle with probability prob to be added to particle list head
struct particle *addParticle(struct particle *head, double prob) {
 struct particle *current, *new_p = (struct particle*) malloc(sizeof(struct particle));
  new_p = (struct particle*) malloc(sizeof(struct particle));
  new_p->x = (double) (rand() / double(RAND_MAX)) * (sx - 1);
  new_p->y = (double) (rand() / double(RAND_MAX)) * (sy - 1);
  while(hit(new_p, map, sx, sy)) {
   new_p->x = (double) (rand() / double(RAND_MAX)) * (sx - 1);
   new_p->y = (double) (rand() / double(RAND_MAX)) * (sy - 1);
  }
  new_p->theta = (double) (rand() / double(RAND_MAX)) * 359;
  new_p->prob = prob;
  ground_truth(new_p, map, sx, sy);
  new_p->next = NULL;
  if(head == NULL) {
   head = new_p;
  }
  else {
   current = head;
   while(current->next != NULL) {
    current = current->next;
   }
   current->next = new_p;
  }
  return head;
}

void initParticles(void)
{
 /*
   This function creates and returns a linked list of particles
   initialized with random locations (not over obstacles or walls)
   and random orientations.

   There is a utility function to help you find whether a particle
   is on top of a wall.

   Use the global pointer 'list' to keep track of the *HEAD* of the
   linked list.

   Probabilities should be uniform for the initial set.
 */

 /***************************************************************
 // TO DO: Complete this function to generate an initially random
 //        list of particles.
 ***************************************************************/
 for(int i = 0; i < n_particles; i++) {
  list = addParticle(list, (1.0 / (double) n_particles));
 }
}

//Selects a particle from the list that has probability in range of range
struct particle *selectParticle(struct particle *list, double range) {
 struct particle *current;
 double running_prob;
 current = list;
 running_prob = current->prob;
 while(current->next != NULL && running_prob < range) {
  current = current->next;
  running_prob += current->prob;
 }
 return current;
}

//Add a particle to the list head, given another particle
//Creates a new particle from next to add to head
struct particle *addParticle(struct particle *head, struct particle *next) {
 struct particle *current, *new_p = (struct particle*) malloc(sizeof(struct particle));
 (*new_p).x = (*next).x;
 (*new_p).y = (*next).y;
 (*new_p).theta = (*next).theta;
 (*new_p).prob = (*next).prob;
 ground_truth(new_p, map, sx, sy);
 new_p->next = NULL;
 if(head == NULL) {
  head = new_p;
 }
 else {
  current = head;
  while(current->next != NULL) {
   current = current->next;
  }
  current->next = new_p;
 }
 return head;
}

void computeLikelihood(struct particle *p, struct particle *rob, double noise_sigma)
{
 /*
   This function computes the likelihood of a particle p given the sensor
   readings from robot 'robot'.

   Both particles and robot have a measurements array 'measureD' with 16
   entries corresponding to 16 sonar slices.

   The likelihood of a particle depends on how closely its measureD
   values match the values the robot is 'observing' around it. Of
   course, the measureD values for a particle come from the input
   map directly, and are not produced by a simulated sonar.

   Assume each sonar-slice measurement is independent, and if

   error_i = (p->measureD[i])-(sonar->measureD[i])

   is the error for slice i, the probability of observing such an
   error is given by a Gaussian distribution with sigma=20 (the
   noise standard deviation hardcoded into the robot's sonar).

   You may want to check your numbers are not all going to zero...

   This function updates the likelihood for the particle in
   p->prob
 */

 /****************************************************************
 // TO DO: Complete this function to calculate the particle's
 //        likelihood given the robot's measurements
 ****************************************************************/
 // get sonor reading
 // initiate sum = 0
 // iterate through measureD, find gaussian error, and add to sum
 // prob = prob * sum
 double sum = 0;
 for (int i = 0; i < 16; i++) {
     double error_i = abs((p->measureD[i])-(rob->measureD[i]));
     sum += log(GaussEval(error_i, noise_sigma));
 }

 p->prob = exp(sum);

}

void ParticleFilterLoop(void)
{
 /*
    Main loop of the particle filter
 */

  // OpenGL variables. Do not remove
  unsigned char *tmp;
  GLuint texture;
  static int first_frame=1;
  double max;
  struct particle *p,*pmax;
  char line[1024];
  // Add any local variables you need right below.

  if (!first_frame)
  {
   // Step 1 - Move all particles a given distance forward (this will be in
   //          whatever direction the particle is currently looking).
   //          To avoid at this point the problem of 'navigating' the
   //          environment, any particle whose motion would result
   //          in hitting a wall should be bounced off into a random
   //          direction.
   //          Once the particle has been moved, we call ground_truth(p)
   //          for the particle. This tells us what we would
   //          expect to sense if the robot were actually at the particle's
   //          location.
   //
   //          Don't forget to move the robot the same distance!
   //

   /******************************************************************
   // TO DO: Complete Step 1 and test it
   //        You should see a moving robot and sonar figure with
   //        a set of moving particles.
   ******************************************************************/
   int turn = 0;
   double move_dist = 4;
   move(robot, move_dist);
   if(hit(robot, map, sx, sy)) {
    if(robot->theta >= 180) {
     robot->theta -= 180;
    }
    else {
     robot->theta += 180;
    }
    turn = 1;
   }
   p = list;
   while(p != NULL) {
    move(p, move_dist);
    if(hit(p, map, sx, sy)) {
     if(p->theta >= 180) {
      p->theta -= 180;
     }
     else {
      p->theta += 180;
     }
    }
    ground_truth(p, map, sx, sy);
    p = p->next;
   }
   // Step 2 - The robot makes a measurement - use the sonar
   sonar_measurement(robot,map,sx,sy);

   // Step 3 - Compute the likelihood for particles based on the sensor
   //          measurement. See 'computeLikelihood()' and call it for
   //          each particle. Once you have a likelihood for every
   //          particle, turn it into a probability by ensuring that
   //          the sum of the likelihoods for all particles is 1.

   /*******************************************************************
   // TO DO: Complete Step 3 and test it
   //        You should see the brightness of particles change
   //        depending on how well they agree with the robot's
   //        sonar measurements. If all goes well, particles
   //        that agree with the robot's position/direction
   //        should be brightest.
   *******************************************************************/
   p = list;
   double sigma = 20, sum_prob = 0;
   while(p != NULL) {
    computeLikelihood(p, robot, sigma);
    sum_prob += p->prob;
    p = p->next;
   }

   p = list;
   while(p != NULL) {
    p->prob /= sum_prob;
    p = p->next;
   }
   // Step 4 - Resample particle set based on the probabilities. The goal
   //          of this is to obtain a particle set that better reflect our
   //          current belief on the location and direction of motion
   //          for the robot. Particles that have higher probability will
   //          be selected more often than those with lower probability.
   //
   //          To do this: Create a separate (new) list of particles,
   //                      for each of 'n_particles' new particles,
   //                      randomly choose a particle from  the current
   //                      set with probability given by the particle
   //                      probabilities computed in Step 3.
   //                      Initialize the new particles (x,y,theta)
   //                      from the selected particle.
   //                      Note that particles in the current set that
   //                      have high probability may end up being
   //                      copied multiple times.
   //
   //                      Once you have a new list of particles, replace
   //                      the current list with the new one. Be sure
   //                      to release the memory for the current list
   //                      before you lose that pointer!
   //

   /*******************************************************************
   // TO DO: Complete and test Step 4
   //        You should see most particles disappear except for
   //        those that agree with the robot's measurements.
   //        Hopefully the largest cluster will be on and around
   //        the robot's actual location/direction.
   *******************************************************************/

   p = list;
   double minimum = 1; 
   while(p != NULL) {
    if (p->prob < minimum) {
        minimum = p->prob;
    } 
    p = p->next;
   }

   p = NULL;
   struct particle *best;
   sum_prob = 0;

   for(int i = 0; i < 0.95 * n_particles; i++) {
    
    best = selectParticle(list, (double) rand() / double(RAND_MAX));
    sum_prob += best->prob;
    p = addParticle(p, best);
   }
   for(int i = 0; i < n_particles - (0.95 * n_particles); i++) {
    sum_prob += minimum;
    p = addParticle(p, minimum);
   }
   deleteList(list);
   list = p;
   p = list;
   while(p != NULL) {
    p->prob /= sum_prob;
    p = p->next;
   }
  }  // End if (!first_frame)

  /***************************************************
   OpenGL stuff
   You DO NOT need to read code below here. It only
   takes care of updating the screen.
  ***************************************************/
  if (RESETflag)	// If user pressed r, reset particles
  {
   deleteList(list);
   list=NULL;
   initParticles();
   RESETflag=0;
  }
  renderFrame(map,map_b,sx,sy,robot,list);

  // Clear the screen and depth buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  glGenTextures( 1, &texture);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glBindTexture( GL_TEXTURE_2D, texture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, sx, sy, 0, GL_RGB, GL_UNSIGNED_BYTE, map_b);

  // Draw box bounding the viewing area
  glBegin (GL_QUADS);
  glTexCoord2f (0.0, 0.0);
  glVertex3f (0.0, 100.0, 0.0);
  glTexCoord2f (1.0, 0.0);
  glVertex3f (800.0, 100.0, 0.0);
  glTexCoord2f (1.0, 1.0);
  glVertex3f (800.0, 700.0, 0.0);
  glTexCoord2f (0.0, 1.0);
  glVertex3f (0.0, 700.0, 0.0);
  glEnd ();

  p=list;
  max=0;
  while (p!=NULL)
  {
   if (p->prob>max)
   {
    max=p->prob;
    pmax=p;
   }
   p=p->next;
  }

  if (!first_frame)
  {
   sprintf(&line[0],"X=%3.2f, Y=%3.2f, th=%3.2f, EstX=%3.2f, EstY=%3.2f, Est_th=%3.2f, Error=%f",robot->x,robot->y,robot->theta,\
           pmax->x,pmax->y,pmax->theta,sqrt(((robot->x-pmax->x)*(robot->x-pmax->x))+((robot->y-pmax->y)*(robot->y-pmax->y))));
   glColor3f(1.0,1.0,1.0);
   glRasterPos2i(5,22);
   for (int i=0; i<strlen(&line[0]); i++)
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,(int)line[i]);
  }

  // Make sure all OpenGL commands are executed
  glFlush();
  // Swap buffers to enable smooth animation
  glutSwapBuffers();

  glDeleteTextures( 1, &texture );

  // Tell glut window to update ls itself
  glutSetWindow(windowID);
  glutPostRedisplay();

  if (first_frame)
  {
   fprintf(stderr,"All set! press enter to start\n");
   gets(&line[0]);
   first_frame=0;
  }
}

/*********************************************************************
 OpenGL and display stuff follows, you do not need to read code
 below this line.
*********************************************************************/
// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    // Create window
    glutInitWindowPosition (0, 0);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle window-related events.
    // In particular, OpenGL has to be informed of which functions
    // to call when the image needs to be refreshed, and when the
    // image window is being resized.
    glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
    glutDisplayFunc(ParticleFilterLoop);   // Main display function is also the main loop
    glutKeyboardFunc(kbHandler);
}

void kbHandler(unsigned char key, int x, int y)
{
 if (key=='r') {RESETflag=1;}
 if (key=='q') {deleteList(list); free(map); free(map_b); exit(0);}
}

void WindowReshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();			// Initialize with identity matrix
    gluOrtho2D(0, 800, 800, 0);
    glViewport(0,0,w,h);
    Win[0] = w;
    Win[1] = h;
}
