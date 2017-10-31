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
double ax = 0, ay = 0, pax = 0, pay = 0;
int override = 0;
int configuration = 0;
double conversion = 180.0 / PI;
int wall_count = 0;
 double VXlim;
 double VYlim;

double quack = 0.005;

/****** Sensor State Variables ******/
//float ANG_OK = 1;

/************************************/


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

    angle = 0;
    Set_Angle(0);
    return;

  } else {

    if (MT_OK) Set_Angle(angle);
    else if (RT_OK) Set_Angle( ((int)angle + 90) % 360 );
    else Set_Angle( ((int)angle - 90) % 360 );

  }
 }

void Set_Thrust(double power, int override) {

 double qang;

 ax = 0;
 ay = G_ACCEL;

 if (override) {
  Main_Thruster(0);
  Left_Thruster(0);
  Right_Thruster(0);
  return;
 }

 qang = aavg;

 if(MT_OK) {
  Main_Thruster(power);
  ax += (power*35)*sin(qang*(1/conversion));
  ay -= (power*35)*cos(qang*(1/conversion));
 }
 else if(RT_OK) { // 270 is down

  qang -= 90;

  Right_Thruster(power);
  ax += (power*25)*sin(qang*(1/conversion));
  ay -= (power*25)*cos(qang*(1/conversion));
 }
 else {

  qang += 90;

  Left_Thruster(power);
  ax += (power*25)*sin(qang*(1/conversion));
  ay -= (power*25)*cos(qang*(1/conversion));
 }
}

void Set_Angle3(int state){
  // State 0: moving away from wall.
  if (state == 0) {

   Set_Angle2(0, override);
   thrust = 1;
  }

  // state == 1: adjust angle relative to landing pad.
  else if (state == 1) {
    angle = conversion * atan((PLAT_X-xavg)/(PLAT_Y-yavg));
    if (angle>0 && vxavg<=(-VXlim)) angle=90;
    if (angle<0 && vxavg>=VXlim) angle=270;
    if (vyavg<VYlim) thrust=1.0;
    else thrust=0.0;

    if(angle < 0) Set_Angle2(360+angle, override);
    else Set_Angle2(angle, override);

  }



  Set_Thrust(thrust,override);

}

void filter() {
 /*
  * Calculates average readings from each sensor by taking 
  * the average of a large number of samples. 
  */
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
   aavg += Angle();
  }

  xavg /= sampleRate;
  yavg /= sampleRate;
  vxavg /= sampleRate;
  vyavg /= sampleRate;
  aavg /= sampleRate;
}

void configure() {

  double cx, cy, cvx, cvy, ca, height, ang;
  double errx, erry, errvx, errvy, erra;

  int fx = 0, fy = 0, fvx = 0, fvy = 0, fa = 0;

  configuration = 0;

  // First state ** BELOW DID NOTHING
  if(pxavg == 0 && pyavg == 0 && pvxavg == 0 && pvyavg == 0 && paavg == 0) {

   return;
  }

  // Mostly does as expected
  cvx = pvxavg + (pax * quack);

  errvx = fabs(vxavg - cvx);
//  errvx = vxavg - cvx;


  if(errvx > 0.03) {
    configuration = 4;
    fvx = 1;
//    vxavg = cvx;
    if((vxavg - cvx) > 0) {

      vxavg = cvx + 0.0010;

    }
    else if((vxavg - cvx) < 0) {

      vxavg = cvx - 0.0010;

    }
  }

  // Mostly does as expected
  cvy = pvyavg - (pay * quack);

  errvy = fabs(vyavg - cvy);

  if(errvy > 0.04) {
    configuration = 5;
    fvy = 1;

    if((vyavg - cvy) > 0) {

      vyavg = cvy + 0.0025;

    }
    else if((vyavg - cvy) < 0) {

      vyavg = cvy - 0.0025;

    }

  }


  // Does as expected
  cx = pxavg + (pvxavg * quack) + (((pax * quack) * quack)/2);
//  cx = pxavg + (((pvxavg + (pax * quack))/2) * quack);
  cx = pxavg + (pvxavg * quack);

  errx = xavg - cx;

  if(fabs(errx) > 2) {
    configuration = 6;
    fx = 1;

/*   if((xavg - cx) > 0) {
//     xavg = pxavg + (0.15 * pvxavg * quack);
     xavg = cx;
    }
    else if((xavg - cx) < 0) {
//     xavg = pxavg - (0.15 * pvxavg * quack);
     xavg = cx;
    } */

  }

  // Mostly does as expected
  cy = pyavg - (pvyavg * quack) - ((pay * quack * quack)/2);

  erry = fabs(yavg - cy);

  if(erry > 2) {
    configuration = 7;
    fy = 1;
//    yavg = cy;
/*    if((yavg - cy) > 0) {
     yavg = cy + (0.0025 / quack);
    }
    else if((yavg - cy) < 0) {
     yavg = cy - (0.0025 / quack);
    }*/
  }


  if(fx == 1) {
    cout << "Loooooop\n";	
    xavg = pxavg + (pvxavg * 0.022);

/*
    if(errx > 0) {
     xavg += 0.0005;
    }
    else if(errx < 0) {
     xavg -= 0.0005;
    }

*/
  }


  if(fy == 1) {
    cout << "Scoooooop\n";	
    yavg = pyavg - (pvyavg * 0.0235);

    if(fabs(PLAT_X-xavg)<100) {


      height = RangeDist();
      ang = aavg;

      if(ang > 90) {
        ang -= 360;
      }

      if(fabs(1024 - PLAT_Y - height) < yavg && fabs(ang) < 15) {

       cout << "hello?\n";
       yavg = fabs(PLAT_Y - height);
      }
    }

  }


}

void Set_Main_Thruster(void) {
 if(MT_OK) main_thruster=0;
 else if(RT_OK) main_thruster=1;
 else main_thruster=2;
}

void checkSensorState(void) {
  /*
   * Monitors sensor outputs, inidicates if they fail.
  */

  // ANG_State = Angle();
  // PX_State = Position_X();
}

void print_status (void) {

  cout << "\n";  
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

  // //xavg = (x1 + x2 + x3 + x4 + x5) / 5;
  cout << "pxavg:" << pxavg << "--pyavg:" << pyavg << "--pvxavg:" << pvxavg << "--pvyavg:" << pvyavg << "--paavg: " << paavg << "\n";
  cout << "xavg1:" << xavg << "\n";
  cout << "yavg1:" << yavg << "\n";
  cout << "vxavg1:" << vxavg << "\n";
  cout << "vyavg1:" << vyavg << "\n";
  cout << "aavg:" << aavg << "\n";
  cout << "ax:" << ax << "\n";
  cout << "ay:" << ay << "\n";
  //xsd = sqrt(((xavg-x1)*(xavg-x1)+(xavg-x2)*(xavg-x2)+(xavg-x3)*(xavg-x3)+(xavg-x4)*(xavg-x4)+(xavg-x5)*(xavg-x5))/4);
  //cout << "xsd:" << xsd << "\n\n";
  cout << "config: " << configuration << "\n";


  /* Sensor outputs */
  //cout << "X: " << Angle() << "\n";


  cout << "\n";

}

void Lander_Control(void)
{
 
// double xavg = 0, xsd=0, yavg = 0, vxavg = 0, vyavg = 0;

// double x1, x2, x3, x4, x5;


 filter();

 configure();

 print_status();

 //Rotate(90);


  pxavg = xavg;
  pyavg = yavg;
  pvxavg = vxavg;
  pvyavg = vyavg;
  paavg = aavg;
  pax = ax;
  pay = ay;
// }

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
 if (fabs(xavg-PLAT_X)>200) VXlim=25/2;
 else if (fabs(xavg-PLAT_X)>100) VXlim=15/2;
 else VXlim=5/2;

 if (PLAT_Y-yavg>200) VYlim=-20/2;
 else if (PLAT_Y-yavg>100) VYlim=-10/2;  // These are negative because they
 else VYlim=-3;              // limit descent velocity

//  VXlim = VXlim/2;
//  VYlim = VYlim/2;

 // Ensure we will be OVER the platform when we land
// if (fabs(PLAT_X-xavg)/fabs(vxavg)>1.25*fabs(PLAT_Y-yavg)/fabs(vyavg)) VYlim=0;

 // Free Falling
 if(fabs(PLAT_X-xavg)<30 && fabs(PLAT_Y-yavg)<30 && vxavg<3 && vyavg<3)
 {
  override = 1;  //thrust will be 0, angle will make sure to straighten
 }
 else
 {

  // set angle relative to the landing pad.
 Set_Angle3(1);

 }

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

 // return;


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

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-xavg)<100&&fabs(PLAT_Y-yavg)<200) { 
  cout << "close to platform: safetyOverride disabled" << "\n";
  return;
}

  cout << "Beep Boop" << "\n";

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 min_angle_x=180;
 min_angle_y=180;


 // Select which area of sonor is checked.
 // if (MT_OK) Set_Angle(angle);
 // else if (RT_OK) Set_Angle( ((int)angle + 90) % 360 );
 // else Set_Angle( ((int)angle - 90) % 360 );


 if (vxavg>0)
 {
  // check right side.
  for (int i=5;i<13;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_x=i*10;
   }
 }
 else
 {
  // check left side.
  for (int i=23;i<31;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin)
   {
    dmin=SONAR_DIST[i];
    min_angle_x=i*10;
   }
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


  if (dmin < 70) {
    //cout << dmin << " Near wall! " << wall_count << "\n";
    wall_count++;
    Set_Angle3(0);
    //thrust = 1;

  } else {
    //cout << dmin << " NOT Near wall! " << wall_count << "\n";
    wall_count++;
    Set_Angle3(1);
//    if (vyavg<VYlim) thrust=1.0;
//    else thrust=0.0;
  }


//  Set_Thrust(thrust,0);
}
