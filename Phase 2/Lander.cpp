/*
  Lander Control simulation.

  Updated by F. Estrada for CSC C85, Oct. 2013
  Updated by Per Parker, Sep. 2015

  Learning goals:

  - To explore the implementation of control software
    that is robust to malfunctions/failures.

  The exercise:

  - The program loads a terrain map from a .ppm file.
    the map shows a red platform which is the location
    a landing module should arrive at.
  - The control software has to navigate the lander
    to this location and deposit the lander on the
    ground considering:

    * Maximum vertical speed should be less than 5 m/s at touchdown
    * Maximum landing angle should be less than 10 degrees w.r.t vertical

  - Of course, touching any part of the terrain except
    for the landing platform will result in destruction
    of the lander

  This has been made into many videogames. The oldest one
  I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

  Your task:

  - These are the 'sensors' you have available to control
          the lander.

    Velocity_X();  - Gives you the lander's horizontal velocity
    Velocity_Y();  - Gives you the lander's vertical velocity
    Position_X();  - Gives you the lander's horizontal position (0 to 1024)
    Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();   - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

    SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

  - Variables accessible to your 'in flight' computer

    MT_OK   - Boolean, if 1 indicates the main thruster is working properly
    RT_OK   - Boolean, if 1 indicates the right thruster is working properly
    LT_OK   - Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X  - X position of the landing platform
          PLAY_Y        - Y position of the landing platform

  - Control of the lander is via the following functions
          (which are noisy!)

    Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
    Left_Thruster(double power);   - Sets left thruster power in [0 1]
    Right_Thruster(double power);  - Sets right thruster power in [0 1]
    Rotate(double angle);    - Rotates module 'angle' degrees clockwise
             (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

             Note that rotation takes time!


  - Important constants

    G_ACCEL = 8.87  - Gravitational acceleration on Venus
    MT_ACCEL = 35.0 - Max acceleration provided by the main thruster
    RT_ACCEL = 25.0 - Max acceleration provided by right thruster
    LT_ACCEL = 25.0 - Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

  - Functions you need to analyze and possibly change

    * The Lander_Control(); function, which determines where the lander should
      go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

  - You *can* add your own helper functions (e.g. write a robust thruster
    handler, or your own robust sensor functions - of course, these must
    use the noisy and possibly faulty ones!).

  - The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

       Initial lander position, orientation, and velocity are
                         randomized.

    * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

    * Failure modes: 0 - Nothing ever fails, life is simple
         1 - Controls can fail, sensors are always reliable
         2 - Both controls and sensors can fail (and do!)
         3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

    * Note - while running. Pressing 'q' on the keyboard terminates the 
      program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

  Have fun! try not to crash too many landers, they are expensive!

    Credits: Lander image and rocky texture provided by NASA
     Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>

#include "Lander_Control.h"
#include <iostream>

using namespace std;

//Dictates which thruster to use, 0->main, 1->right, 2->left
int main_thruster = 0;
int sampleRate = 300;
double angle, thrust;
double pxavg = 0, pyavg = 0, pvxavg = 0, pvyavg = 0, paavg = 0;
double xavg = 0, yavg = 0, vxavg = 0, vyavg = 0, aavg = 0;
double ax = 0, ay = 0;
int override = 0;
int configuration = 0;
double conversion = 180.0 / PI;

void Set_Angle(double angle) {

//angle is 0
 if(angle == 0) {

   if(aavg>1&&aavg<359)
   {
    if(aavg>=180) Rotate(360-aavg);
    else Rotate(-aavg);
    return;
   }
 }

//otherwise
 else {
  if(aavg>(angle+1)||aavg<(angle-1)) {
   if(aavg>angle){
    if(aavg-angle>=180) Rotate(360-(aavg-angle));
    else Rotate(angle-aavg);
   }
   else {
    if(angle-aavg>=180) Rotate(angle-aavg-360);
    else Rotate(angle-aavg);
   }
   return;
  }
 }
}

 void Set_Angle2(double angle, int override) {

  // if override is on, preparing for landing so straighten
  if (override) {

    Set_Angle(0);

  } else {

    if (MT_OK) Set_Angle(angle);
    else if (RT_OK) Set_Angle( ((int)angle + 90) % 360 );
    else Set_Angle( ((int)angle - 90) % 360 );

  }
 }


void Set_Thrust(double power, int override) {

 ax = 0;
 ay = 0;

 if (override) {
  Main_Thruster(0);
  Left_Thruster(0);
  Right_Thruster(0);
  return;
 }

 if(MT_OK) {
  Main_Thruster(power);
  ax += 35*sin(aavg);
  ay += 35*cos(aavg);
 }
 else if(RT_OK) {
  Right_Thruster(power);
  ax += 25*sin(aavg - 90);
  ay += 25*cos(aavg - 90);
 }
 else {
  Left_Thruster(power);
  ax += 25*sin(aavg + 90);
  ay += 25*cos(aavg + 90);
 }
}

void filter() {
  xavg = 0;
  yavg = 0;
  vxavg = 0;
  vyavg = 0;
  aavg = 0;

  for(int i = 0; i < sampleRate; i++) {
   xavg += Position_X();
   vxavg += Velocity_X();
   yavg += Position_Y();
   vyavg += Velocity_Y();
   aavg+=Angle();
  }

  xavg /= sampleRate;
  yavg /= sampleRate;
  vxavg /= sampleRate;
  vyavg /= sampleRate;
  aavg /= sampleRate;
}

void configure() {

  double cx, cy, cvx, cvy, ca;
  double errx, erry, errvx, errvy, erra;

  configuration = 0;

  // First state ** BELOW DID NOTHING
/*  if(pxavg == 0 && pyavg == 0 && pvxavg == 0 &&pvyavg == 0 && aavg == 0) {

   return;
  }*/

  // Mostly does as expected
  cvx = pvxavg + (ax * T_STEP);

  errvx = fabs(vxavg - cvx);

  if(errvx > 0.5) {
    configuration = 4;
  }

  // Mostly does as expected
  cvy = pvyavg + (ay * T_STEP);

  errvy = fabs(vyavg - cvy);

  if(errvy > 0.5) {
    configuration = 5;
  }


  // Does as expected
  cx = pxavg + (pvxavg * T_STEP) + (ax * T_STEP * T_STEP);

  errx = fabs(xavg - cx);

  if(errx > 2) {
    configuration = 6;
  }

  // Mostly does as expected
  cy = pyavg + (pvyavg * T_STEP) + (ay * T_STEP * T_STEP);

  erry = fabs(yavg - cy);

  if(erry > 2) {
    configuration = 7;
  }

}

void Set_Main_Thruster(void) {
 if(MT_OK) main_thruster=0;
 else if(RT_OK) main_thruster=1;
 else main_thruster=2;
}

void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
  DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/

 double VXlim;
 double VYlim;
// double xavg = 0, xsd=0, yavg = 0, vxavg = 0, vyavg = 0;

// double x1, x2, x3, x4, x5;


 filter();

 configure();

/*
  x1 = Position_X();
  cout << "x1:" << x1 << "\n";
  x2 = Position_X();
  cout << "x2:" << x2 << "\n";
  x3 = Position_X();
  cout << "x3:" << x3 << "\n";
  x4 = Position_X();
  cout << "x4:" << x4 << "\n";
  x5 = Position_X();
  cout << "x5:" << x5 << "\n";
*/

  //xavg = (x1 + x2 + x3 + x4 + x5) / 5;
  cout << "pxavg:" << pxavg << "--pyavg:" << pyavg << "--pvxavg:" << pvxavg << "--pvyavg:" << pvyavg << "--paavg: " << paavg << "\n";
  cout << "xavg1:" << xavg << "\n";
  cout << "yavg1:" << yavg << "\n";
  cout << "vxavg1:" << vxavg << "\n";
  cout << "vyavg1:" << vyavg << "\n";
  cout << "aavg:" << aavg << "\n";
  //xsd = sqrt(((xavg-x1)*(xavg-x1)+(xavg-x2)*(xavg-x2)+(xavg-x3)*(xavg-x3)+(xavg-x4)*(xavg-x4)+(xavg-x5)*(xavg-x5))/4);
  //cout << "xsd:" << xsd << "\n\n";
  cout << "config: " << configuration << "\n";
  cout << "\n";


  pxavg=xavg;
  pyavg=yavg;
  pvxavg=vxavg;
  pvyavg=vyavg;
  paavg = aavg;
// }

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
 if (fabs(xavg-PLAT_X)>200) VXlim=25;
 else if (fabs(xavg-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-yavg>200) VYlim=-20;
 else if (PLAT_Y-yavg>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;              // limit descent velocity

  VXlim = VXlim/2;
  VYlim = VYlim/2;

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-xavg)/fabs(vxavg)>1.25*fabs(PLAT_Y-yavg)/fabs(vyavg)) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Rotate first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Rotate() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.
 if(fabs(PLAT_X-xavg)<30 && fabs(PLAT_Y-yavg)<30 && vxavg<3 && vyavg<3)
 {
  override = 1;  //thrust will be 0, angle will make sure to straighten
 }
 else
 {
  // Module is oriented properly, check for horizontal position
  // and set thrusters appropriately.
 // if (Position_X()>PLAT_X)
 // {
   // Lander is to the LEFT of the landing platform, use Right thrusters to move
   // lander to the left.
 //  Left_Thruster(0);  // Make sure we're not fighting ourselves here!
 //  if (Velocity_X()>(-VXlim)) Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
 //  else
 //  {
    // Exceeded velocity limit, brake
 //   Right_Thruster(0);
 //   Left_Thruster(fabs(VXlim-Velocity_X()));
 //  }
 // }
 // else
 // {
   // Lander is to the RIGHT of the landing platform, opposite from above
 //  Right_Thruster(0);
 //  if (Velocity_X()<VXlim) Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
 //  else
 //  {
 //  Left_Thruster(0);
 //   Right_Thruster(fabs(VXlim-Velocity_X()));
 //  }
 // }
  // Vertical adjustments. Basically, keep the module below the limit for
  // vertical velocity and allow for continuous descent. We trust
  // Safety_Override() to save us from crashing with the ground.

//  Right_Thruster(0);
//  Left_Thruster(0);

//  angle=90*((PLAT_X-Position_X())/PLAT_X);
  angle = conversion * atan((PLAT_X-xavg)/(PLAT_Y-yavg));
  if (angle>0 && vxavg<=(-VXlim)) angle=90;
  if (angle<0 && vxavg>=VXlim) angle=270;
  if (vyavg<VYlim) thrust=1.0;
//  if (Velocity_Y()<VYlim) thrust=0.25;
  else thrust=0.0;
 }

 if(angle < 0) Set_Angle2(360+angle, override);
 else Set_Angle2(angle, override);
 Set_Thrust(thrust, override);



} 
void Safety_Override(void)
{
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/

 return;

 double DistLimit;
 double Vmag;
 double dmin, min_angle, min_angle_x, min_angle_y, curr_ang;
 int close_right = 0, close_left = 0, close_bottom = 0, close_top = 0;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=vxavg*vxavg;
 Vmag+=vyavg*vyavg;

 DistLimit=fmax(75,Vmag);

/* if(xavg >= 980) {
    angle = conversion * atan((PLAT_X-xavg)/(PLAT_Y-yavg));
    if (angle>0 && vxavg<0) angle=90;

    if(angle < 0) Set_Angle2(360+angle, override);
    else Set_Angle2(angle, override);
    Set_Thrust(thrust, override);
    return;
 }

 if(xavg <= 60) {
    angle = conversion * atan((PLAT_X-xavg)/(PLAT_Y-yavg));
    if (angle>0 && vxavg>0) angle=270;

    if(angle < 0) Set_Angle2(360+angle, override);
    else Set_Angle2(angle, override);
    Set_Thrust(thrust, override);
    return;
 }*/

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;
// if (fabs(PLAT_X-xavg)<150 || yavg < 40) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 min_angle_x=180;
 min_angle_y=180;

 if (vxavg>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_x=i*10;
   }
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_x=i*10;
   }
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?

 if (dmin<DistLimit*fmax(.25,fmin(fabs(vxavg)/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (vxavg>0){  // on right
/*   Left_Thruster(1.0);
   Right_Thruster(0.0);
   Rotate(min_angle-Angle());
*/
//   if(MT_OK) close_right = 1;

//   if(RT_OK) close_top = 1;

//   if(LT_OK) close_bottom = 1;

    curr_ang = conversion * atan((PLAT_X-xavg)/(PLAT_Y-yavg));

//    if (angle>0 && vxavg<0) angle=90;
//    if (angle<0 && vxavg>0) angle=270;
//    if (vyavg<5) thrust=1.0;
    thrust = 1;

    if(MT_OK) curr_ang = curr_ang - 90;
    if(LT_OK) curr_ang = 0;
    if(RT_OK) curr_ang = curr_ang + 90;

    if(curr_ang < 0) Set_Angle2(360+curr_ang, override);
    else Set_Angle2(curr_ang, override);
    Set_Thrust(thrust, override);

  }
  else // on left
  {
/*
   Right_Thruster(1.0);
   Left_Thruster(0.0);
*/

    // angle between lander and platform
    curr_ang = conversion * atan((PLAT_X-xavg)/(PLAT_Y-yavg));

//    if (angle>0 && vxavg<0) angle=90;
//    if (angle<0 && vxavg>0) angle=270;
//    if (vyavg<5) thrust=1.0;
    thrust = 1;

    if(MT_OK) curr_ang = curr_ang - 90;
    if(RT_OK) curr_ang = 0;
    if(LT_OK) curr_ang = curr_ang + 90;

    if(curr_ang < 0) Set_Angle2(360+curr_ang, override);
    else Set_Angle2(curr_ang, override);
    Set_Thrust(thrust, override);

//    return;

/*  if(MT_OK) close_left = 1;

  if(RT_OK) close_bottom = 1;

  if(LT_OK) close_top = 1;*/
  }
  thrust=1.0;
 }

 // Vertical direction
 dmin=1000000;
 if (vyavg>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_y=i*10;
   }
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_y=i*10;
   }
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_y=i*10;
   }
 }

 min_angle = (min_angle_x + min_angle_y) / 2;

// if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 if (dmin<DistLimit*fmax(.25,fmin(fabs(vyavg)/5.0,1)))
// if(0) 
/* if (close_right)
 {

  // new angle calc.

*/
  if (aavg>1||aavg>359)
  {
   if (aavg>=180) Rotate(360-aavg);
   else Rotate(-aavg);
//   return;
  }


    if(dmin<100) {

//  angle=90*((PLAT_X-Position_X())/PLAT_X);
    angle = conversion* atan((PLAT_X-xavg)/(PLAT_Y-yavg));
/*    if (angle>0 && vxavg<0) angle=90;
    if (angle<0 && vxavg>0) angle=270;
    if (vyavg<5) thrust=1.0;
//  if (Velocity_Y()<VYlim) thrust=0.25;
    else thrust=0.0;

    if(angle < 0) Set_Angle2(360+angle, override);
    else Set_Angle2(angle, override);
    Set_Thrust(thrust, override);

    return;
    }*/

  if (Velocity_Y()>2.0)
  {
   thrust=0.0;
  }
  else
  {
   //min_angle = 180;
   thrust=1.0;
  }

   /*if(dmin<100) {
   thrust=1;
   if(min_angle>=50&&min_angle<140&&Velocity_X()>0) thrust=1;
   if(min_angle>=220&&min_angle<320&&Velocity_X()<0) thrust=1;
   if(min_angle>=0&&min_angle<50&&Velocity_Y()<=2) thrust=1;
   if(min_angle>=320&&min_angle<360&&Velocity_Y()<=2) thrust=1;
   if(dmin<50) thrust=1;
   }
   else min_angle=angle+180;

   if(min_angle<180) Set_Angle2(min_angle + 180, override);
   else Set_Angle2(min_angle-180, override);
   Set_Thrust(thrust, override);
   close_right = 0;
   close_left = 0;
   return;*/
  }


// }
/* for (int i=0; i<36; i++) {
   if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle=i*10;
   }
 }*/



 if(dmin<100) {
 thrust=1;
 if(min_angle>=50&&min_angle<140&&vxavg>0) thrust=1;
 if(min_angle>=220&&min_angle<320&&vxavg<0) thrust=1;
 if(min_angle>=0&&min_angle<50&&vyavg<=2) thrust=1;
 if(min_angle>=320&&min_angle<360&&vyavg<=2) thrust=1;
 if(dmin<50) thrust=1;
 }
 else min_angle=angle+180;
 //min_angle=angle+180;
// if(min_angle<=90 || min_angle>=270)
// {

// thrust = 1;

  if(min_angle<180) Set_Angle2(min_angle + 180, override);
  else Set_Angle2(min_angle-180, override);
  Set_Thrust(thrust, override);
// }
}
