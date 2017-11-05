/***********************************************************
             CSC C85 – Lab 3 - Control

      Inverted Pendulum control exercise.

      Your Task:

        Write a function that determines the force to be applied to the
        car in order to control the inverted pendulum bar.

      What you need to read in this file:

        Look at the global variables and their definition. These specify
        the quantities involved in the simulation of the pendulum.

        Aside from that, you only need to look at the function

         ApplyHorizontalForce();

        In this function you are supposed to compute and set the force
        variable for the pendulum simulation.

        You can safely ignore the rest of the code, or, if you find it useful
        you can learn about simple OpenGL in preparation for taking
        the Computer Graphics course D18 next year.

     Usage:

        IntertedPendulum cart_mass bar_mass bar_length sampling_period perturbation

        sampling_period specifies how often ApplyHorizontalForce()
                        is called during the simulation, for example,
                        if sampling_period=1, your code will be called
                        at intervals of one second (simulation time, not
                        wall-clock time).
                        .01 <= sampling_period < 5

        perturbation specifies the maximum initial value for theta (though
                     the actual initial value is randomized).
		     0 < perturbation < PI/2

     Updated Aug. 15, 2014 by F. Estrada

***********************************************************/

#include "InvertedPendulum.h"

// *************** GLOBAL VARIABLE DEFINITIONS *************************

double m;			// Mass of the cart
double m1;			// Mass of the bar
double l1;			// Length of the bar
double theta;			// Angle of the bar w.r.t. vertical
double x;			// Position of the cart along the x direction
double F;			// Force applied on cart along the x direction
double sp;			// Sampling period
double g=9.81;			// Gravitational acceleration constant
double st;			// Internal clock
const float PI = 3.14159;

// --------------- OpenGL Stuff -----------------
// Window settings
int windowID;               // Glut window ID (for display)
int Win[2];                 // window (x,y) size

// ******************** FUNCTION IMPLEMENTATIONS ***********************


void ApplyHorizontalForce(void)
{
 /*
   Attempt to control the inverted pendulum via a force applied on the cart
   along the x direction.
 */

 //////////////////////////////////////////////////////////////////////////
 //
 // Learning objectives:
 //
 // This assignment is designed to help you understand better the workings
 // of a 'closed loop' control system
 //
 // - You will learn to measure and use the variables that affect the state
 // of the system.
 //
 // - You will gain experience on designing a controller for a simple physical
 // system that nevertheless has many real-world applications
 //
 // - You will gain experience solving a hard problem for which you have
 // incomplete knowledge. We haven't discussed all the math that is
 // used to design control systems, so you will have to make do with
 // your wits, programming expertise, and the fact that you have team-mates
 // and can rely on them to solve the problem together. This is good
 // practice for work after you graduate!
 //
 // - You will gain a new appreciation of how awesome the human brain is.
 //
 //////////////////////////////////////////////////////////////////////////

 //////////////////////////////////////////////////////////////////////////
 // TO DO:
 //   Control the pendulum by applying appropriate amounts of force
 //  along the X axis.
 //
 //   What you can use:
 //
 //    You have access to the global variables:
 //    m - the cart's mass
 //    m1 - the bar's mass
 //    l1 - the length of the bar
 //    sp - the sampling period
 //    g - gravity's acceleration
 //    st - the internal clock
 //    theta - the angle of the bar w.r.t. the vertical position
 //    x - the position of the cart along the x axis
 //    F - current value of the force being applied on the cart
 //
 //   You MUST NOT use any other information than the above variables.
 //
 //   Your function should use these variables to update F, but note
 //   that you ARE NOT ALLOWED to change any global variables except
 //   for F itself!
 //
 //   Try to design a smart algorithm that will control the pendulum
 //   for whatever values of m, m1, l1, and sp I may choose to use to
 //   test your code. Also, note that the initial position of the
 //   pendulum is randomized.
 //
 ///////////////////////////////////////////////////////////////////////////

 ///////////////////////////////////////////////////////////////////////////
 //
 // Testing:
 //
 //   Consider the effect of the system's variables on controllability:
 //
 //  You may want to start with an 'easy' pendulum. Once you have a handle
 //  on that one, see if you can get your code to deal with
 //  harder problems.
 //
 //  Note that the pendulum can never reach 90 degrees from vertical. This
 //     means that in theory you should be able to jank it back to vertical
 //     by applying a large enough force even if it looks like a lost cause.
 //     However, the software only gives you 5 simulation 'seconds'
 //     to recover before the program exits.
 //
 ///////////////////////////////////////////////////////////////////////////

 fprintf(stderr,"Current simulation state: time=%f, x=%f, theta=%f, force applied= ",st,x,theta);

 F=0;

 fprintf(stderr,"%f\n",F);
 return;
}

//////////////////////////////////////////////////////////////////////////
//
// You DO NOT need to read any code below this point.
// You are NOT ALLOWED to modify any code below this point.
//
//////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
 /*
   Initialize global system parameters from command line arguments.
   In general:

   - light carts should be easier to control
   - longer bars are easier to control
   - longer sampling periods make the system harder (or impossible!) to stabilize
 */
 double perturb;

 srand48(time(NULL));

 if(argc != 6) {
     printf("Usage: InvertedPendulum cart_mass bar_mass bar_length sampling_period perturbation\n");
     exit(0);
 } else {
     m=atof(argv[1]);
     m1=atof(argv[2]);
     l1=atof(argv[3]);
     sp=atof(argv[4]);
     perturb=atof(argv[5]);
     x=0;
     while(theta==0||fabs(theta)>(.5*PI)) theta=(drand48()*perturb)-(.5*perturb);
     st=0;
     Win[0] = 500;		// OpenGL Window size - you can adjust this if you wish
     Win[1] = 500;
 }

 if (sp<.01||sp>5)
 {
  fprintf(stderr,"Sampling period should be in [.01 5]\n");
  exit(0);
 }

 // Initialize glut, glui, and opengl
 glutInit(&argc, argv);
 initGlut(argv[0]);
 GL_Settings_Init();

 // Invoke the standard GLUT main event loop
 glutMainLoop();

 exit(0);
}

// Initialize GLUT and create an OpenGL window with the desired
// properties.
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    // Double buffering is used to provide smooth animation
    // RGB indicates the image will be in colour RGB format
    // And the depth buffer will be used to properly handle visibility
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    // Create window
    glutInitWindowPosition (0, 0);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle events
    glutReshapeFunc(WindowReshape);   // Called whenever window resized
    glutDisplayFunc(WindowDisplay);   // Called to update the image for each frame 
}

// Handles the window being resized by updating the viewport
// and projection matrices
void WindowReshape(int w, int h)
{
    // Setup projection matrix for new window
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Set up perspective projection
    gluPerspective(25,1,15,500);

    // Camera tracks the cart...
    gluLookAt(x-75,50,150,x,0,0,0,1,0);

    // Update OpenGL viewport and internal variables
    glViewport(0,0, w,h);
    Win[0] = w;
    Win[1] = h;
}

void GL_Settings_Init()
{
 // Initialize OpenGL parameters to be used throughout the
 // life of the graphics window

    // Set the background colour
    glClearColor(0.01f,0.01f,0.01f,1.0f);

/**** Illumination set up start ****/

    // The section below controls the illumination of the scene.
    // Nothing can be seen without light, so the section below
    // is quite important. We will discuss different types of
    // illumination and how they affect the appearance of objecs
    // later in the course.
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);    // Enable depth testing
    glEnable(GL_LIGHTING);      // Enable lighting
    glEnable(GL_LIGHT0);        // Enable LIGHT0 for diffuse illumination
    glEnable(GL_LIGHT1);        // Enable LIGHT1 for ambient illumination

    // Set up light source colour, type, and position
    GLfloat light0_colour[]={1.0,1.0,1.0};
    GLfloat light1_colour[]={.25,.25,.25};
    GLfloat light0_pos[]={500,0,500,0};
    glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_colour);
    glLightfv(GL_LIGHT1,GL_AMBIENT,light1_colour);
    glLightfv(GL_LIGHT0,GL_POSITION,light0_pos);
    glShadeModel(GL_SMOOTH);

    // Enable material colour properties
    glEnable(GL_COLOR_MATERIAL);

/***** Illumination setup end ******/
}

void drawAxisLines(void)
{
  // Generate a set of lines so that we can see motion along x
  // Eventually the car runs out of the universe and into the void...
  float i;

  for (i=(int)(x/15)-500;i<=x+500;i+=15)
  {
   glColor3f(0.5,0.5,0.0);
   glBegin(GL_LINES);
   glVertex3f(i,0,-500);
   glVertex3f(i,0,500);
   glEnd();
  }

  for (i=-500;i<=500;i+=15)
  {
   glColor3f(0.5,0.5,0.0);
   glBegin(GL_LINES);
   glVertex3f(x-500,0,i);
   glVertex3f(x+500,0,i);
   glEnd();
  }
}

// Main OpenGL display function. Draws the scene given the parameters
// of the simulation.
void WindowDisplay(void)
{
    // Update projection so camera stays on target
    // Setup projection matrix for new window
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Set up perspective projection
    gluPerspective(25,1,15,500);
    // Camera tracks the cart...
    gluLookAt(x-75,50,150,x,0,0,0,1,0);

    // OK, now clear the screen with the background colour
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Setup the model-view transformation matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Draw the cart and bar system
    GLUquadric *quadObject;
    quadObject=gluNewQuadric();

    // Push the current transformation matrix on the stack
    glPushMatrix();
    drawAxisLines();			// Generate ground lines

    glTranslatef(x,0,0);		// Global coordinates now centered at cart

        // Draw the body of the cart
        glPushMatrix();
	  glTranslatef(-7.5,6,0);	// Move upward from ground
          glRotatef(90,0,1,0);	// Rotate onto x axis
          glColor3f(0.0, 0.3, 1.0);
          gluCylinder(quadObject,3,3,15,10,4);
 	glPopMatrix();
        glPushMatrix();
          glTranslatef(-7.5,6,0);
          glScalef(.01,1,1);
          gluSphere(quadObject,3,4,4);
        glPopMatrix();
        glPushMatrix();
          glTranslatef(7.5,6,0);
          glScalef(.01,1,1);
          gluSphere(quadObject,3,4,4);
        glPopMatrix();

        // Rear wheels
        glPushMatrix();
	  glTranslatef(-3.5,2,2.5);
          glColor3f(.5,.5,.5);
          glScalef(1,1,.01);
          gluSphere(quadObject,2,10,10);
        glPopMatrix();
        glPushMatrix();
	  glTranslatef(-3.5,2,-2.5);
          glColor3f(.5,.5,.5);
          glScalef(1,1,.01);
          gluSphere(quadObject,2,10,10);
        glPopMatrix();

        // Front wheels
        glPushMatrix();
	  glTranslatef(3.5,2,2.5);
          glColor3f(.5,.5,.5);
          glScalef(1,1,.01);
          gluSphere(quadObject,2,10,10);
        glPopMatrix();
        glPushMatrix();
	  glTranslatef(3.5,2,-2.5);
          glColor3f(.5,.5,.5);
          glScalef(1,1,.01);
          gluSphere(quadObject,2,10,10);
        glPopMatrix();

	// Joint
	glPushMatrix();
	  glTranslatef(0,9,0);
          glColor3f(.7,.7,.7);
          gluSphere(quadObject,1.5,9,10);\
        glPopMatrix();

        // Bar
        glPushMatrix();
          glTranslatef(0,9,0);
          glRotatef(-90,1,0,0);
          glRotatef(theta*(180.0/(PI)),0,1,0);
          glColor3f(1.0,.2,.3);
          gluCylinder(quadObject,1,1,l1,10,10);
        glPopMatrix();

    // Retrieve the previous state of the transformation stack
    glPopMatrix();

    // Execute any GL functions that are in the queue just to be safe
    glFlush();

    // Destroy our quadrics object
    gluDeleteQuadric(quadObject);

    // Now, show the frame buffer that we just drew into.
    // (this prevents flickering).
    glutSwapBuffers();

    // Here call the function to update the simulation parameters
    simulationEngine();

    // Tell glut window to update itself with the new parameters (this creates an endless loop)
    glutSetWindow(windowID);
    glutPostRedisplay();
}

void simulationEngine(void)
{
 /*
    This is an (OVERSIMPLIFIED) simulation of the physical system for the pendulum,
    if you're curious you can look up online how the state equations are derived.

    * YOU ARE NOT ALLOWED TO CHANGE ANYTHING IN THIS FUNCTION *
 */
 static double dt;
 static double dtheta;
 static double dx;
 static double ddtheta;
 static double ddx;
 static double th;
 double I1;
 double LH[4][4];
 double invLH[4][4];
 double RH[2];
 double det;
 double const s_inc=.005;

 if (st==0)	// Initial conditions. No motion yet.
 {
  dx=0;
  ddx=0;
  dtheta=0;
  ddtheta=0;
  th=0;
 }

 I1=(4.0/3.0)*m1*(l1*l1);

 LH[0][0]=m+m1;
 LH[0][1]=m1*l1*cos(theta);
 LH[1][0]=m1*l1*cos(theta);
 LH[1][1]=I1+(m1*(l1*l1));

 RH[0]=F+(m1*l1*sin(theta)*(dtheta*dtheta));
 RH[1]=m1*l1*sin(theta)*g;

 det=(LH[0][0]*LH[1][1])-(LH[1][0]*LH[0][1]);
 if (det==0)
 {
  fprintf(stderr,"Determinant is zero, simulation stop!\n");
  return;
 }

 invLH[0][0]=LH[1][1]/det;
 invLH[0][1]=-LH[0][1]/det;
 invLH[1][0]=-LH[1][0]/det;
 invLH[1][1]=LH[0][0]/det;

 ddx=(invLH[0][0]*RH[0])+(invLH[0][1]*RH[1]);
 ddtheta=(invLH[1][0]*RH[0])+(invLH[1][1]*RH[1]);
 dx=dx+(s_inc*ddx);
 x=x+(dx*s_inc);

 // Angle limit. The joint won't permit the pendulum to swing beyond 90 degrees minus a small amount
 if (fabs(theta)<.485*PI)
 {
  dtheta=dtheta+(s_inc*ddtheta);
  theta=theta+(dtheta*s_inc);
  if (theta>.49*PI) theta=.49*PI;
  else if (theta<-.49*PI) theta=-.49*PI;
  th=0;
 }
 else
 {
  dtheta=0;
  ddtheta=0;
  ddx=0;
  if (theta>0) theta=.49*PI; else theta=-.49*PI;
  th+=s_inc;
  if (th>5) exit(0);
 }

 st+=s_inc;
 dt+=s_inc;

 if (dt>=sp)
 {
  ApplyHorizontalForce();
  dt-=sp;
 }

}

