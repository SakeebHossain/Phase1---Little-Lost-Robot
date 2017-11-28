/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Version: 0.2 - Updated Oct 2, 2014 - F. Estrada
***************************************************************************/

#include "imagecapture/imageCapture.h"
#include "roboAI.h"			// <--- Look at this header file!
#include <nxtlibc/nxtlibc.h>
#include <stdio.h>
#include <stdlib.h>

void clear_motion_flags(struct RoboAI *ai)
{
 // Reset all motion flags. See roboAI.h for what each flag represents
 // *You may or may not want to use these*
 ai->st.mv_fwd=0; 
 ai->st.mv_back=0;
 ai->st.mv_bl=0;
 ai->st.mv_br=0;
 ai->st.mv_fl=0;
 ai->st.mv_fr=0;
}

struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This function looks for and identifies a blob with the specified colour.
 // It uses colour contrast betwen the R, G, and B channels to pick the 
 // blob that is closest in colour to the specified target. If multiple
 // blobs with similar colour exist, then it picks the most saturated one.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> R
 //                   1 -> G
 //                   2 -> B
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double BCRT=1.05;			// Ball colour ratio threshold
 double c1,c2,c3,m,mi,ma;
 double oc1,oc2,oc3;
 int i;

 oc1=1000;
 oc2=1;
 oc3=1;

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  if (col==0) {c1=p->R; c2=p->G; c3=p->B;} 	// detect red
  else if (col==1) {c1=p->G; c2=p->R; c3=p->B;} // detect green
  else if (col==2){c1=p->B; c2=p->G; c3=p->R;}  // detect blue

  // Normalization and range extension
  mi=p->R;
  if (p->G<mi) mi=p->G;
  if (p->B<mi) mi=p->B;
  ma=p->R;
  if (p->G>ma) ma=p->G;
  if (p->B>ma) ma=p->B;

  c1=(c1-mi)/(ma-mi);
  c2=(c2-mi)/(ma-mi);
  c3=(c3-mi)/(ma-mi);
  c1+=.001;
  c2+=.001;
  c3+=.001;
  
  if (c1/c2>BCRT&&c1/c3>BCRT)			// Blob has sufficient colour contrast
  {
   m=(c1/c2)+(c1/c3);				// Total color contrast ch1 vs ch2 and ch3
   if (fnd==NULL||m>(oc1/oc2)+(oc1/oc3)) 	// Found the first blob with this color, or a more colorful one
   {
    fnd=p;
    oc1=c1;
    oc2=c2;
    oc3=c3;
   }
  }
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Heading (a unit vector in the direction of motion). Not valid
 //   while rotating - possibly valid while turning
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // Note that the blob data
 // structure itself contains another useful vector with the blob
 // orientation (obtained directly from the blob shape, valid at all
 // times even under rotation, but can be pointing backward!)
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 double NOISE_VAR=5;

 // Reset ID flags
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;

 // Find the ball
 p=id_coloured_blob(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute heading direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,1);
 else p=id_coloured_blob(ai,blobs,0);
 if (p)
 {
  ai->st.self=p;			// Update pointer to self-blob

  // Adjust Y position if we have calibration data
  if (fabs(p->adj_Y[0][0])>.1)
  {
   dmax=384.0-p->adj_Y[0][0];
   dmin=767.0-p->adj_Y[1][0];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][0]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;

  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }

  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
 }
 else ai->st.self=NULL;

 // ID our opponent
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,0);
 else p=id_coloured_blob(ai,blobs,1);
 if (p)
 {
  ai->st.opp=p;	

  if (fabs(p->adj_Y[0][1])>.1)
  {
   dmax=384.0-p->adj_Y[0][1];
   dmin=767.0-p->adj_Y[1][1];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][1]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/5.0;

 drive_speed(30);			// Start forward motion to establish heading
					// Will move for a few frames.

 track_agents(ai,blobs);		// Call the tracking function to find each agent

 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  all_stop();
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 all_stop();			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 clear_motion_flags(ai);
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 track_agents(ai,blobs);
}

int d_speed = 60, r_speed = 25, t_speed = 50, face_right, bound_prox = 0, face_down, not_moving, line_up[3]={1,0,2}, last_mode = -1, spawn_top;
double old_dir_x = -10, old_dir_y = -10, bal_euc, fixed_x, fixed_y, dists[2] = {-1.0, -1.0};


int *current_state, prev_state;

double init_ball_x, init_ball_y;

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is the main AI loop.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your NXT to get damaged!

  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	// The id_bot() routine will change the AI state to initial state + 1
  {				// if robot identification is successful.
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;
   all_stop();
   clear_motion_flags(ai);
   initPhantomBall(ai, blobs);  // set initial ball size
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.self->mx,ai->st.self->my,ai->st.state);
  }
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
//  fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d, side=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.state, ai->st.side);
  track_agents(ai,blobs);		// Currently, does nothing but endlessly track
 }

  /****************************************************************************
  PENALTY KICK 

  Below are summaries of what each state is responsible for:	
  

  *****************************************************************************/


  if(checkBlobsExist(ai, blobs)) {
    return;
  }
  
  // Check if phantom ball appears.
  if (phantomBall(ai, blobs)) {
    return;
  }
  current_state = &(ai->st.state);

  //printf("Old: (%f, %f)\n",old_dir_x,old_dir_y);
  //printf("New: (%f, %f)\n",ai->st.self->dx,ai->st.self->dy);

  if(old_dir_x != -10) {
     directionCorrection(ai, blobs);
  } else {
    initialDir(ai, blobs);

    double ball_x_size = (ai->st.ball->x2 - ai->st.ball->x1);
    ball_x_size *= ball_x_size;
    double ball_y_size = (ai->st.ball->y2 - ai->st.ball->y1);
    ball_y_size *= ball_y_size;

    bal_euc = sqrt(ball_x_size + ball_y_size);
  }

  bound_prox = closeToBoundary(ai, blobs);


  if((*current_state / 100) < 1) {

    // Will we attack or defend?
     int mode = modeChoice(ai, blobs);
    // Say chunk out 20 states per (least in dist)
    // Defense: 10-30
    // Attack: 40-60
    // Kick: 70-80 or whatever (will then keep track of old state?)
    // Kicking has own state section used by both if needed
    // Can have default mode default states, if still doing one, continue

    // Consider having a unique check just for soccer that doesn't reset to init when things vanish

    // If we vanish, we should still have our old move flag set, so do the inverse
    // Above requires a change for inclusion in directionCorrection (ignores bl/br)
    // Also if we disappear may need to somewhat reinitialize direction vector

/*
    if(!mode) {
      //if(!(*current_state > 4) && !(*current_state == 1)) {
        // attack!!!!!
      //}

      if(last_mode && !(*current_state < 1)) {
        *current_state = 2;
        all_stop();
        stop_kicker();
        clear_motion_flags(ai);
        not_moving = 1;
      }
    }
*/

    if(bound_prox != 0 && *current_state > 1 && *current_state < 96) {
      boundCheckSoccer(ai,blobs);
      all_stop();
      clear_motion_flags(ai);
      not_moving = 1;
      
    }

    if(*current_state == 1) {

      if( ai->st.smx > 0) {
          face_right = 1;
       } else {
          face_right = 0;
      }

      if( ai->st.smy > 0) {
          face_down = 1;
      } else {
          face_down = 0;
      }
      //lineUp(ai, blobs);
      //distancesFromBall(ai, blobs);

      *current_state = 2;
    }

    // Defense
    // STATE 2: If ball is behind us and not approaching our net
    //	- Either turn around and run, or run backwards
    //	- If about parallel to ball, move to different position of field and move back
    //	- Same condition if parallel to opponent

    else if ( *current_state == 2 ) {

      toBallVec(ai, blobs); 
 
      if(facingBall(ai,blobs)) {

        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 3;
     
      } 
    }
    else if ( *current_state == 3 ) {

      if(closeToBall(ai,blobs)) {
        *current_state = 4;
      }

      if(!facingBall(ai,blobs)) {
        *current_state = 2;
      }

      drive_speed(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_fwd = 1;

    }
    else if ( *current_state == 4 ) {
      retract();
      *current_state = 5;
    }
    else if ( *current_state == 5 ) {
      stop_kicker();
      *current_state = 6;
    }
    else if ( *current_state == 6 ) {
      kick();
      *current_state = 7;
    }
    else if ( *current_state == 7 ) {
      kick();
      *current_state = 8;
    }
    else if ( *current_state == 8 ) {
      stop_kicker();
      *current_state = 2;
    }

/*
    if(*current_state == 2) {

      toBallVecRev(ai, blobs); 
 
      if(facingBallRev(ai, blobs)) {
//        if(closeToBall(ai, blobs)) {
          all_stop();
          clear_motion_flags(ai);
          not_moving = 1;
          *current_state = 3;
 //       }
      } 

    }

    else if ( *current_state == 3 ) {

      if(closeToBall(ai,blobs)) {
        *current_state = 4;
      }

      if(!facingBallRev(ai,blobs)) {
        *current_state = 2;
      }

      reverse_speed(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_back = 1;

    }

    if(*current_state == 4) {

      if(!ai->st.side && ai->st.self->cy < ai->st.ball->cy) {
        pivot_left_speed(50);
        clear_motion_flags(ai);
        ai->st.mv_fl = 1;
      }
      else if(!ai->st.side && ai->st.self->cy > ai->st.ball->cy) {
        pivot_right_speed(50);
        clear_motion_flags(ai);
        ai->st.mv_fr = 1;
      }
      else if(ai->st.side && ai->st.self->cy < ai->st.ball->cy) {
        pivot_right_speed(50);
        clear_motion_flags(ai);
        ai->st.mv_fr = 1;
      }
      else if(ai->st.side && ai->st.self->cy > ai->st.ball->cy) {
        pivot_left_speed(50);
        clear_motion_flags(ai);
        ai->st.mv_fl = 1;
      }

      if(!closeToBall(ai,blobs)) {
        *current_state = 2;
      }
      
    }
*/
    // STATE 3: If ball is behind us and heading towards our net, try to intercept
    //	- If parallel to ball, make our route ark upwards/downwards get the ball
    //	- If opp. kicking ball, run in front of ball
    //	- Otherwise approach ball and kick away

    // STATE 4: If opp. have control, get in front of ball
    // 	- May move backwards when defending

    else if(*current_state == 96) {
      
      toWallVec(ai, blobs);

      if(facingWall(ai,blobs)) {
        
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = prev_state;
             
      } 
    }
    else if(*current_state == 97) {

      toWallVec(ai, blobs);

      if(facingWall(ai,blobs)) {
        
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = prev_state;
             
      } 
    }
    else if(*current_state == 98) {

      toWallVec(ai, blobs);
      
      if(facingWall(ai,blobs)) {
        
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = prev_state;
             
      }   
    }
    else if(*current_state == 99) {

      toWallVec(ai, blobs);
      
      if(facingWall(ai,blobs)) {
        
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = prev_state;
             
      } 
    }

    last_mode = mode;
  }
  

  //fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d, side=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.state, ai->st.side);
//  fprintf(stderr,"Current position: (%f,%f), current heading: [%f, %f], current direction: [%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.self->dx,ai->st.self->dy,ai->st.state);
  //fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f]\n",ai->st.self->cx,ai->st.self->cy,ai->st.self->mx,ai->st.self->my);

//  fprintf(stderr,"state: %d\n", *current_state);
  
  else if((*current_state / 100) < 2) {
    if ( *current_state == 101 ) {
      
      if( ai->st.smx > 0) {
         face_right = 1;
      } else {
         face_right = 0;
      }

      if( ai->st.smy > 0) {
          face_down = 1;
      } else {
          face_down = 0;
      }

      if( ai->st.self->cy < 384) {
         spawn_top = 1;
      } else {
         spawn_top = 0;
      }

      *current_state = 102;
    }

    // STATE 102: turn to face wall
    else if ( *current_state == 102 ) {
      if (lookTop(ai, blobs))  {
        *current_state = 103;
      }
    }

    // STATE 103: facing X wall, so stop
    else if ( *current_state == 103) {
      all_stop();
      clear_motion_flags(ai);
      not_moving = 1;
      *current_state = 104;   
    }

    // STATE 104: move towards X wall
    else if ( *current_state == 104 ) {
    //      if (ai->st.self->cy > 720 || ai->st.self->cy < 200) {

      if(spawn_top) {
        if( (ai->st.ball->cy - ai->st.self->cy) > 80 ) {
          all_stop();
          clear_motion_flags(ai);
          not_moving = 1;
          *current_state = 105;
        } else {
          drive_speed(d_speed);
          clear_motion_flags(ai);
          ai->st.mv_fwd = 1;
        }
      } else {
        if( (ai->st.self->cy - ai->st.ball->cy) > 80 ) {
          all_stop();
          clear_motion_flags(ai);
          not_moving = 1;
          *current_state = 105;
        } else {
          drive_speed(d_speed);
          clear_motion_flags(ai);
          ai->st.mv_fwd = 1;
        }
      }
    }

    // STATE 105: wall reached, turn to face our side.
    else if ( *current_state == 105 ) {

      if (lookSide(ai, blobs))  {
       all_stop();
       clear_motion_flags(ai);
       not_moving = 1;
       *current_state = 106;
      }
    }

    // STATE 106: move towards Y wall
    else if ( *current_state == 106 ) {

      //   if(ai->st.self->cx > 954 || ai->st.self->cx < 50) {
      //if( (ai->st.self->cx - ai->st.ball->cx) > 80 ) {
      if( (ai->st.self->x1 - ai->st.ball->cx) > 80 ) {
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 107;
      }
      else {
        drive_speed(d_speed);
        clear_motion_flags(ai);
        ai->st.mv_fwd = 1;
      }
    }

    // STATE 107: face our net
    else if ( *current_state == 107 ) {

      if (lookNet(ai, blobs))  {
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 108;
      }  
    }

    // STATE 108: move towards center of our net (measured relative to ball)
    else if ( *current_state == 108 ) {

      if( (abs(ai->st.self->cy - ai->st.ball->cy) < 30 && spawn_top) ||
       (abs(ai->st.self->cy - ai->st.ball->cy) < 100 && !(spawn_top))) {
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 109;
      }
      else {
        drive_speed(d_speed);
        clear_motion_flags(ai);
        ai->st.mv_fwd = 1;
      }
    }

    // STATE 109: face the ball
    else if ( *current_state == 109 ) {

      if (lookBall(ai, blobs))  {
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 110;
      }  

    }

    // STATE 110: move towards ball (measured relative to ball)
    else if ( *current_state == 110 ) {
      //if( abs(ai->st.self->cx - ai->st.ball->cx) < 300 ) {
      if( abs(ai->st.self->x1 - ai->st.ball->cx) < 150 ) {
        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 113;
      }
      else {
        drive_speed(d_speed);
        clear_motion_flags(ai);
        ai->st.mv_fwd = 1;
      }
    }
    else if ( *current_state == 113 ) {
      retract();
      *current_state = 114;
    }
    else if ( *current_state == 114 ) {
      stop_kicker();
      *current_state = 115;
    }
    else if ( *current_state == 115 ) {
      kick();
      *current_state = 116;
    }
    else if ( *current_state == 116 ) {
      kick();
      *current_state = 117;
    }
    else if ( *current_state == 117 ) {
      stop_kicker();
      *current_state = 118;
    }
    else if ( *current_state == 118 ) {
      fprintf(stderr,"Turn at Wall");
    }

  }
//----------------------------------------------
// CHASE BALL
//----------------------------------------------


  else {

    if ( *current_state == 201 ) {


      if( ai->st.smx > 0) {
        face_right = 1;
      } else {
        face_right = 0;
      }

      if( ai->st.smy > 0) {
          face_down = 1;
      } else {
          face_down = 0;
      }

      *current_state = 202;    
    }  

    else if ( *current_state == 202 ) {

      toBallVec(ai, blobs); 
 
      if(facingBall(ai,blobs)) {

        all_stop();
        clear_motion_flags(ai);
        not_moving = 1;
        *current_state = 209;
     
      } 
    }
    else if ( *current_state == 203 ) {
      all_stop();
      clear_motion_flags(ai);
      not_moving = 1;
      *current_state = 209;      
    }
    else if ( *current_state == 204 ) {
      retract();
      *current_state = 205;
    }
    else if ( *current_state == 205 ) {
      stop_kicker();
      *current_state = 206;
    }
    else if ( *current_state == 206 ) {
      kick();
      *current_state = 207;
    }
    else if ( *current_state == 207 ) {
      kick();
      *current_state = 208;
    }
    else if ( *current_state == 208 ) {
      stop_kicker();
      *current_state = 202;
    }
    else if ( *current_state == 209 ) {

      if(closeToBall(ai,blobs)) {
        *current_state = 204;
      }

      if(!facingBall(ai,blobs)) {
        *current_state = 202;
      }

      drive_speed(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_fwd = 1;

    }

  }

  //fprintf(stderr, "Old bot direction: (%f,%f)\n", old_dir_x, old_dir_y);

  if(!((*current_state % 100) == 0)) {
    old_dir_x = ai->st.self->dx;
    old_dir_y = ai->st.self->dy;
    ai->st.self->dx = fixed_x;
    ai->st.self->dy = fixed_y;
  }


 //fprintf(stderr, "Current bot direction: (%f,%f)\n", ai->st.self->dx, ai->st.self->dy);


} 

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

void initialDir(struct RoboAI *ai, struct blob *blobs) {

  //drive_speed(10);

//  if( ai->st.smx < 0 && ai->st.self->dx > 0) {
    //ai->st.self->dx *= -1;
    //ai->st.self->dy *= -1;
    //fixed_x = ai->st.self->dx * (-1.0);
    //fixed_y = ai->st.self->dy * (-1.0);
    fixed_x = ai->st.self->mx;
    fixed_y = ai->st.self->my;
//  }

  //all_stop();

}

void directionCorrection(struct RoboAI *ai, struct blob *blobs) {


if(ai->st.mv_back && face_right) {
  return;
}
else if(ai->st.mv_fwd && face_right) {
  return;
}
else if(not_moving) {
  not_moving = 0;
  return;
}


 //double bound = 2.5 * old_dir_y;

int flipped = 0;
double cross = (ai->st.self->dx * old_dir_y) - (old_dir_x * ai->st.self->dy);

/*
 if(ai->st.self->dy > 0 && old_dir_y < 0) {

   if(fabs(ai->st.self->dy - old_dir_y) > 1.0) {
      flipped = 1;
   }

 }
 else if(ai->st.self->dy < 0 && old_dir_y > 0) {

   if(fabs(ai->st.self->dy - old_dir_y) > 1.0) {
      flipped = 1;
   }

 }

*/

  if(ai->st.mv_fl) {
    if(cross < 0) {
      flipped = 1;
    }
  }

  if(ai->st.mv_fr) {
    if(cross > 0) {
      flipped = 1;
    }
  }

  if(flipped && face_right) {
    //ai->st.self->dx *= -1;
    //ai->st.self->dy *= -1;
    fixed_x = ai->st.self->dx * (-1.0);
    fixed_y = ai->st.self->dy * (-1.0);
    face_right = 0;
  }
  else if(flipped && !face_right) {
    fixed_x = ai->st.self->dx;
    fixed_y = ai->st.self->dy;
    face_right = 1;
  }
  else if(!flipped && !face_right) {
    //ai->st.self->dx *= -1;
    //ai->st.self->dy *= -1;
    fixed_x = ai->st.self->dx * (-1.0);
    fixed_y = ai->st.self->dy * (-1.0);
  } else {
    fixed_x = ai->st.self->dx;
    fixed_y = ai->st.self->dy;
  }


 //ai->st.self->dy
 //ai->st.smy
 // If vector flips
// printf("Old: (%f, %f)\n",ai->st.self->dx,ai->st.self->dy);

/*

  if(fabs(ai->st.smy - old_dir_y) > 1.2 && face_right) {
    ai->st.self->dx *= -1;
    ai->st.self->dy *= -1;
    face_right = 0;
  }
  else if(fabs(ai->st.smy - old_dir_y) > 1.2 && !face_right) {
    face_right = 1;
  }
  else if(fabs(ai->st.smy - old_dir_y) < 1.2 && !face_right) {
    ai->st.self->dx *= -1;
    ai->st.self->dy *= -1;
  }

  // printf("New: (%f, %f)\n",ai->st.self->dx,ai->st.self->dy);
  */
}


int closeToBall(struct RoboAI *ai, struct blob *blobs) {
/*  fprintf(stderr, "Current bot position: (%f,%f); Current ball position: (%f,%f)\n", ai->st.self->cx,ai->st.self->cy, ai->st.ball->cx,ai->st.ball->cy); */

  double ball_x_size = (ai->st.ball->x2 - ai->st.ball->x1);
  ball_x_size *= ball_x_size;
  double ball_y_size = (ai->st.ball->y2 - ai->st.ball->y1);
  ball_y_size *= ball_y_size;

  double euc_dist = sqrt(ball_x_size + ball_y_size);

  double x, y, mag;

    x = ai->st.ball->cx - ai->st.self->cx;
    y = ai->st.ball->cy - ai->st.self->cy;

    mag = sqrt((x*x) + (y*y));

  

//  if ( fabs(ai->st.self->cx - ai->st.ball->cx) < ball_x_size && fabs(ai->st.self->cy - ai->st.ball->cy) < ball_y_size) {

  if(fabs(mag - bal_euc) < (1.75*bal_euc)) {
    return 1;
  }

  return 0;
}


int lookTop(struct RoboAI *ai, struct blob *blobs) {

  if ( fabs(ai->st.self->dx) > .1 ) {
    pivot_left_speed(r_speed);
    clear_motion_flags(ai);
    ai->st.mv_fl = 1;
    return 0;

  }

  return 1;
}

int lookSide(struct RoboAI *ai, struct blob *blobs) {

  if ( fabs( ai->st.self->dy ) > .1 ) {
    if(face_right) {
      pivot_right_speed(r_speed);
      clear_motion_flags(ai);
      ai->st.mv_fr = 1;
    //} else if(ai->st.mv_back) {
    } else {
      pivot_left_speed(r_speed);
      clear_motion_flags(ai);
      ai->st.mv_fl = 1;
    }
    return 0;

  }

  return 1;
}

int lookNet(struct RoboAI *ai, struct blob *blobs) {

  if ( ai->st.self->dx > .1 ) {
    if(spawn_top) {
      pivot_right_speed(r_speed);
      clear_motion_flags(ai);
      ai->st.mv_fr = 1;
    } else {
      pivot_left_speed(r_speed);
      clear_motion_flags(ai);
      ai->st.mv_fl = 1;
    }
    return 0;

  }

  return 1;
}

int lookBall(struct RoboAI *ai, struct blob *blobs) {

  if ( fabs(ai->st.self->dy) > .1 ) {
    if(spawn_top) {
      pivot_right_speed(r_speed);
      clear_motion_flags(ai);
      ai->st.mv_fr = 1;
    } else {
      pivot_left_speed(r_speed);
      clear_motion_flags(ai);
      ai->st.mv_fl = 1;
    }
    return 0;

  }

  return 1;
}

void toWallVec(struct RoboAI *ai, struct blob *blobs) {
  
      double theta, phi, x, y, mag, dot, cross;
      int correction, speed;

      if(*current_state == 96) {
        x = 0;
        y = 1;
      }
      else if(*current_state == 97) {
        x = -1;
        y = 0;
      }
      else if(*current_state == 98) {
        x = 0;
        y = -1;
      }
      else if(*current_state == 99) {
        x = 1;
        y = 0;
      }
  
      dot = (fixed_x * x) + (fixed_y * y);
      dot = 1 - dot;
  
      if(dot > 1) {
         dot = 1;
      }
  
      cross = (fixed_x * y) - (x * fixed_y);
  
      speed = t_speed * dot;
  
      if(speed < 40) {
         speed = 40;
      }
  
  
      if(cross >= 0) {
         //turn_right_speed(speed);
         pivot_right_speed(speed);
         clear_motion_flags(ai);
         ai->st.mv_fr = 1;
      }
      else if(cross < 0) {
         //turn_left_speed(speed);
         pivot_left_speed(speed);
         clear_motion_flags(ai);
         ai->st.mv_fl = 1;       
      }
  
  }

int facingWall(struct RoboAI *ai, struct blob *blobs) {
    
    
  double theta, phi, x, y, mag, dot, cross;
  int correction, speed;

  if(*current_state == 96) {
    x = 0;
    y = 1;
  }
  else if(*current_state == 97) {
    x = -1;
    y = 0;
  }
  else if(*current_state == 98) {
    x = 0;
    y = -1;
  }
  else if(*current_state == 99) {
    x = 1;
    y = 0;
  }
    
  dot = (fixed_x * x) + (fixed_y * y);

  if ( dot > 0.95  ) {
    fprintf(stderr, "%f facing ball.\n", dot);
    return 1;
  }
    
  return 0;
    
}

void toBallVec(struct RoboAI *ai, struct blob *blobs) {

    double theta, phi, x, y, mag, dot, cross;
    int correction, speed;

    // Vector from centre of bot to centre of ball
    x = ai->st.ball->cx - ai->st.self->cx;
    y = ai->st.ball->cy - ai->st.self->cy;

    mag = 1.0 / sqrt((x*x) + (y*y));
    x *= mag;
    y *= mag;

    dot = (fixed_x * x) + (fixed_y * y);
    dot = 1 - dot;

    if(dot > 1) {
       dot = 1;
    }

    cross = (fixed_x * y) - (x * fixed_y);

    speed = t_speed * dot;

    if(speed < 40) {
       speed = 40;
    }


    if(cross >= 0) {
       turn_right_speed(speed);
       //pivot_right_speed(speed);
       clear_motion_flags(ai);
       ai->st.mv_fr = 1;
    }
    else if(cross < 0) {
       turn_left_speed(speed);
       //pivot_left_speed(speed);
       clear_motion_flags(ai);
       ai->st.mv_fl = 1;       
    }

}

void toBallVecRev(struct RoboAI *ai, struct blob *blobs) {

    double theta, phi, x, y, mag, dot, cross;
    int correction, speed;

    // Vector from centre of bot to centre of ball
    x = ai->st.ball->cx - ai->st.self->cx;
    y = ai->st.ball->cy - ai->st.self->cy;

    mag = 1.0 / sqrt((x*x) + (y*y));
    x *= mag;
    y *= mag;

    dot = (fixed_x * x) + (fixed_y * y);
    dot = 1 - dot;

    if(dot > 1) {
       dot = 1;
    }

    cross = (fixed_x * y) - (x * fixed_y);

    speed = t_speed * dot;

    if(speed < 40) {
       speed = 40;
    }


    if(cross >= 0) {
       pivot_left_speed(speed);
       //pivot_right_speed(speed);
       clear_motion_flags(ai);
       ai->st.mv_fl = 1;
    }
    else if(cross < 0) {
       pivot_right_speed(speed);
       //pivot_left_speed(speed);
       clear_motion_flags(ai);
       ai->st.mv_fr = 1;       
    }

  }


int checkBlobsExist(struct RoboAI *ai, struct blob *blobs) {


    if (ai->st.self == NULL ) {
      all_stop();
      clear_motion_flags(ai);
      not_moving = 1;
      stop_kicker();
      return 1;
    }

    int diff = ai->st.state % 100;

    if(ai->st.opp == NULL && ai->st.state < 100) {
      fprintf(stderr, "opponent is gone.....\n");
      all_stop();
      clear_motion_flags(ai);
      not_moving = 1;
      stop_kicker();
      ai->st.state -= diff;
      old_dir_x = -10;
      old_dir_y = -10;
      return 1;
    }
    if (ai->st.ball == NULL) {
      fprintf(stderr, "ball is gone.....\n");
      all_stop();
      clear_motion_flags(ai);
      not_moving = 1;
      stop_kicker();
      ai->st.state -= diff;
      old_dir_x = -10;
      old_dir_y = -10;
      return 1;
    }

    return 0;
  }

int facingBall(struct RoboAI *ai, struct blob *blobs) {


    double x, y, mag, dot;
    int correction;

    // Vector from centre of bot to centre of ball
    x = ai->st.ball->cx - ai->st.self->cx;
    y = ai->st.ball->cy - ai->st.self->cy;

    mag = 1.0 / sqrt((x*x) + (y*y));
    x *= mag;
    y *= mag;

    dot = (fixed_x * x) + (fixed_y * y);



 if ( dot > 0.95  ) {
     fprintf(stderr, "%f facing ball.\n", dot);
     return 1;
 }

  return 0;

}

int facingBallRev(struct RoboAI *ai, struct blob *blobs) {


    double x, y, mag, dot;
    int correction;

    // Vector from centre of bot to centre of ball
    x = ai->st.ball->cx - ai->st.self->cx;
    y = ai->st.ball->cy - ai->st.self->cy;

    mag = 1.0 / sqrt((x*x) + (y*y));
    x *= mag;
    y *= mag;

    dot = (fixed_x * x) + (fixed_y * y);



 if ( dot < -0.95  ) {
     fprintf(stderr, "%f facing ball.\n", dot);
     return 1;
 }

  return 0;

}

int closeToBoundary(struct RoboAI *ai, struct blob *blobs) {

   if(ai->st.self->x1 < 65 && ai->st.self->mx < 0) {
      return 1;
   }

   if(ai->st.self->y1 < 65 && ai->st.self->my < 0) {
      return 2;
   }

   if(ai->st.self->x2 > 960 && ai->st.self->mx > 0) {
      return 3;
   }

   if(ai->st.self->y2 > 700 && ai->st.self->my > 0) {
      return 4;
   }

   return 0;


}


int boundCheckSoccer(struct RoboAI *ai, struct blob *blobs) {
//printf("%d", *current_state);
  if(bound_prox == 1) {
    /*
    if(fixed_x < 0) {

      //if(ai->st.mv_fl || ai->st.mv_fr || ai->st.mv_fwd || ai->st.mv_back) {
        all_stop();
      //}

      reverse(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_back = 1;

    } else { // drives over boundary

     // if(ai->st.mv_bl) {
        all_stop();
      //}

      drive(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_fwd = 1;
    }
    */

    if(*current_state < 96) {
      prev_state = *current_state;
    }
    
    *current_state = 99;
  }

  if(bound_prox == 2) {

    /*
    if(fixed_y < 0) { // over

      //if(ai->st.mv_fl) {
        all_stop();
      //}

      reverse(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_back = 1;
    } else {

     // if(ai->st.mv_bl) {
        all_stop();
      //}

      drive(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_fwd = 1;
    }
    */

    if(*current_state < 96) {
      prev_state = *current_state;
    }

    *current_state = 98;
  }

  if(bound_prox == 3) {
    /*
    if(fixed_x > 0) { //over bound

      //if(ai->st.mv_fl) {
        all_stop();
      //}

      reverse(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_back = 1;
    } else {

     // if(ai->st.mv_bl) {
        all_stop();
     // }

      drive(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_fwd = 1;
    }
    */

    if(*current_state < 96) {
      prev_state = *current_state;
    }

    *current_state = 97;
  }

  if(bound_prox == 4) {

    /*
    if(fixed_y > 0) { // over bound

      //if(ai->st.mv_fl) {
        all_stop();
     // }

      reverse(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_back = 1;
    } else {

      //if(ai->st.mv_bl) {
        all_stop();
     // }

      drive(d_speed);
      clear_motion_flags(ai);
      ai->st.mv_fwd = 1;
    }
    */

    if(*current_state < 96) {
      prev_state = *current_state;
    }

    *current_state = 96;
  }

}

int modeChoice(struct RoboAI *ai, struct blob *blobs) {
    //0 for defend, 1 for attack
    if(!(ai->st.side) && ai->st.ball->cx < ai->st.self->cx) return 0;
    if(!(ai->st.side) && ai->st.ball->cx >= ai->st.self->cx) return 1;
    if(ai->st.side && ai->st.ball->cx > ai->st.self->cx) return 0;
    if(ai->st.side && ai->st.ball->cx <= ai->st.self->cx) return 1;
}

void lineUp(struct RoboAI *ai, struct blob *blobs) {
    // Tells us the order from left (our net) to right (opp net) of all blobs placements
    // (0->ball, 1->self, 2->opp)
    if(ai->st.ball->cx < ai->st.self->cx && ai->st.self->cx < ai->st.opp->cx) {
        line_up[0] = 0;
        line_up[1] = 1;
        line_up[2] = 2;
    }
    if(ai->st.ball->cx < ai->st.opp->cx && ai->st.opp->cx < ai->st.self->cx) {
        line_up[0] = 0;
        line_up[1] = 2;
        line_up[2] = 1;
    }
    if(ai->st.self->cx < ai->st.ball->cx && ai->st.ball->cx < ai->st.opp->cx) {
        line_up[0] = 1;
        line_up[1] = 0;
        line_up[2] = 2;
    }
    if(ai->st.self->cx < ai->st.opp->cx && ai->st.opp->cx < ai->st.ball->cx) {
        line_up[0] = 1;
        line_up[1] = 2;
        line_up[2] = 0;
    }
    if(ai->st.opp->cx < ai->st.ball->cx && ai->st.ball->cx < ai->st.self->cx) {
        line_up[0] = 2;
        line_up[1] = 0;
        line_up[2] = 1;
    }
    if(ai->st.opp->cx < ai->st.self->cx && ai->st.self->cx < ai->st.ball->cx) {
        line_up[0] = 2;
        line_up[1] = 1;
        line_up[2] = 0;
    }
    printf("Order-> Left: %d, Middle: %d, Right: %d\n", line_up[0],line_up[1],line_up[2]);
}

void distancesFromBall(struct RoboAI *ai, struct blob *blobs) {
    // Set our distance from ball
    dists[0] = sqrt((ai->st.self->cx*ai->st.self->cx)+(ai->st.ball->cx*ai->st.ball->cx));
    // Set opp distance from ball
    dists[1] = sqrt((ai->st.opp->cx*ai->st.opp->cx)+(ai->st.ball->cx*ai->st.ball->cx));
    printf("Distances from ball: 1: %f, 2: %f\n", dists[0],dists[1]);
}

void initPhantomBall(struct RoboAI *ai, struct blob *blobs) {
  
    /* 
     * gets the initial parameters of the ball for comparisons later.  
    */

    if(ai->st.ball == NULL) {
      return;
    }

    init_ball_x = fabs(ai->st.ball->x1 - ai->st.ball->x2);
    init_ball_y = fabs(ai->st.ball->y1 - ai->st.ball->y2);
  
  }
  
  int phantomBall(struct RoboAI *ai, struct blob *blobs) {
    
      /* 
       * check if the ball has changed in size dramatically. If it has it's probably not a ball!  
      */

      double current_ball_x = fabs(ai->st.ball->x1 - ai->st.ball->x2);
      double current_ball_y = fabs(ai->st.ball->y1 - ai->st.ball->y2);
  
      if ( fabs(current_ball_x - init_ball_x) > 50 ||
          fabs(current_ball_y - init_ball_y) > 50) {
            return 1;   //it's not the ball!
          }
      
      else {
        return 0;  // it is the ball!
      }
  
  }
    