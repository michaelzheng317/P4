#include "imagecapture/imageCapture.h"
#include "roboAI.h"
#include <stdio.h>
#include <stdlib.h>

// Your AI goes here! ----------------------
void clear_motion_flags(struct RoboAI *ai)
{
 ai->st.mv_fwd=0; 
 ai->st.mv_back=0;
 ai->st.mv_bl=0;
 ai->st.mv_br=0;
 ai->st.mv_fl=0;
 ai->st.mv_fr=0;
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 // Top-level AI

 fprintf (stderr,"AI_main() called for ai at %p and blob list at %p\n",ai,blobs);

 if (ai->st.state==0)  // Initial set up - find own, ball, and opponent blobs			
 {
  fprintf(stderr,"State is 0, self-id in progress...\n");
  id_bot(ai,blobs);
  if (ai->st.selfID==1)
  {
   ai->st.state=1;
   all_stop();
   clear_motion_flags(ai);
  }
 }
}

void particle_filter(struct RoboAI *ai, struct blob *blobs)
{
 // Partile filter - uses the current state of the ai and
 // the estimated blob parameters to determine which blobs
 // are consistent with the expected robot motion. The
 // blob with highest p and those within 80% of that value
 // are considered part of the bot. Of the remaining blobs,
 // the largest blob with high (1-p) is selected as the
 // opponent. Note that no effort is made to determine
 // a set of blobs for the opponent.
 // If the AI has found a ball, the ball is not considered.

 int i;
 struct blob *p,*q;
 double vx1,vy1,vx2,vy2;
 double ppx,ppy;
 double dx,dy;
 double dp;
 double len,len2,mx;
 double noiseV=5;			// Expected motion magnitude due to noise for a static object
					// (it's just a guess - haven't measured it)
 double oppSizeT=2500;			// Opponent's blob nimimum size threshold
 double psum;

 fprintf(stderr,"Particle filter running\n");
 p=blobs;
 psum=0;
 while (p!=NULL)
 {
  if (p!=ai->st.ball)			// Ignore the ball!
  {
   if (ai->st.mv_fwd+ai->st.mv_back+ai->st.mv_bl+ai->st.mv_br+ai->st.mv_fr+ai->st.mv_fl>0)
   {
    fprintf(stderr,"Checking blob %d for motion: %f %f\n",p->blobId, p->vx[1],p->vy[1]);
    // Robot should be moving. Check motion consistency
    if (fabs(p->vx[1])>noiseV||fabs(p->vy[1])>noiseV)			// Use v[1] to check for motion. v[0] may have been re-initialized
    {									// for un-tracked blobs.
     fprintf(stderr,"It moves!\n",p->blobId, p->vx[1],p->vy[1]);
     vx1=p->vx[0];
     vy1=p->vy[0];
     len=1.0/sqrt((vx1*vx1)+(vy1*vy1));
     vx1*=len;
     vy1*=len;
     vx2=p->vx[1];
     vy2=p->vy[1];
     len2=1.0/sqrt((vx2*vx2)+(vy2*vy2));
     vx2*=len2;
     vy2*=len2;
     dp=(vx1*vx2)+(vy1*vy2);		// Get the dot product between the current and last motion vector

     // Prediction step
     ppx=p->cx[1]+p->vx[1];
     ppy=p->cy[1]+p->vy[1];
     dx=p->cx[0]-ppx;
     dy=p->cy[0]-ppx;
     // Component perpendicular to previous motion direction (currently in vx2, vy2)
     len=((-vy2*dx)+(vx2*dy))/len2;	// Scale so exp doesn't go nuts

     if (p->tracked_status==1)
      fprintf(stderr,"Tracked blob: %d, v[0]=[%f %f],v[1]=[%f %f],dp=%f, len=%f, like=%f\n",p->blobId,p->vx[0],p->vy[0],p->vx[1],p->vy[1],\
              dp,len,exp(-(len*len)/(2*noiseV*noiseV)));
     else
      fprintf(stderr,"Un-tracked blobkk: %d, v[0]=[%f %f],v[1]=[%f %f],dp=%f, len=%f, like=%f\n",p->blobId,p->vx[0],p->vy[0],p->vx[1],p->vy[1],\
              dp,len,exp(-(len*len)/(2*noiseV*noiseV)));

     if (ai->st.mv_fwd||ai->st.mv_back)
     {
      if (dp>.05) p->p *= sqrt(dp);
      else p->p *= .05;
     }
     else
     {
      // Need to check turning direction - assumes a meaningful direction change has taken place between
      // calls to particle filter
      if (dp>.98) p->p *= (.85*dp);	// Looks to be moving straight-ahead but short turns will be hard to tell
      else if (dp<.05) p->p *= .05;	// Moving in the wrong direction
      else
       if (ai->st.mv_fr||ai->st.mv_bl)
       {
        if ((vx1*vy2)-(vx2*vy1)<0) p->p *= sqrt(dp);
        else p->p *= .05+pow(dp,5);
       }
       else
       {
        if ((vx1*vy2)-(vx2*vy1)>0) p->p *= sqrt(dp);
        else p->p *= .05+pow(dp,5);
       }
     }
    }	
    else
    {
     fprintf(stderr,"Particle %d is static, tracked status=%d\n",p->blobId,p->tracked_status);
     p->p *= .05;		// Blob is static!
    }
   }
   else
   {
    if (fabs(p->vx[0])<noiseV&&fabs(p->vy[0])<noiseV) p->p *=1.0;
    else p->p *= 1.0/(p->vx[0]+p->vy[0]);
    // Robot should be static, blobs should be static
   }
   psum+=p->p;
  }
  p=p->next;
 }

 p=blobs;				// Probability normalization
 while (p!=NULL) {p->p /= psum; p=p->next;}

 // Print current blob list
 p=blobs;
 while (p!=NULL) {if (p!=ai->st.ball) fprintf(stderr,"BlobId=%d, p=%f\n",p->blobId,p->p); p=p->next;}

 // Mark current blobs
 mx=0;
 p=blobs;
 while (p!=NULL) {if (p->p > mx&&ai->st.ball!=p) mx=p->p; p=p->next;}

 p=blobs;
 while (p!=NULL) {if (p->p >= .25*mx && p!=ai->st.ball) p->idtype=1; else if (p!=ai->st.ball) p->idtype=0; p=p->next;}

 // Here check for blobs that are significantly above uniform, select the top one and
 // mark it as well as any other within 80% probability as own bot.
 // Then check for large blobs with large 1-p to mark as opponent.

}

void id_ball(struct RoboAI *ai, struct blob *blobs)
{
 // Find the halflings! find the halflings! ahem! I mean,
 // the ball...
 struct blob *p;
 double BCRT=2.0;			// Ball colour ratio threshold
 int i;

 p=blobs;
 while (p!=NULL)
 {
  if (p->R/p->G>BCRT&&p->R/p->B>BCRT)		// Red coloured blob
  {
   if (ai->st.ball!=NULL)			// Got some other red blob
   {
    if (ai->st.ball==p)
    {
     ai->st.ballID=1;
     ai->st.bvx=ai->st.ball->cx[0]-ai->st.old_bcx;
     ai->st.bvy=ai->st.ball->cy[0]-ai->st.old_bcy;
     ai->st.old_bcx=ai->st.ball->cx[0]; 
     ai->st.old_bcy=ai->st.ball->cy[0];
     i=1;
    }
    else if (ai->st.ball->R/ai->st.ball->G < p->R/p->G) 	// But this is redder!
    {
     ai->st.ball=p;
     ai->st.ballID=1;
     ai->st.bvx=ai->st.ball->cx[0]-ai->st.old_bcx;
     ai->st.bvy=ai->st.ball->cy[0]-ai->st.old_bcy;
     ai->st.old_bcx=ai->st.ball->cx[0]; 
     ai->st.old_bcy=ai->st.ball->cy[0];
     ai->st.ball->idtype=2;
     i=1;
    }
   }
   else					// First red blob found
   {
    ai->st.ball=p;
    ai->st.ballID=1; 
    ai->st.bvx=0;
    ai->st.bvy=0;
    ai->st.old_bcx=ai->st.ball->cx[0]; 
    ai->st.old_bcy=ai->st.ball->cy[0];
    ai->st.ball->idtype=2;
    i=1;
   }
  }
  p=p->next;
 }
 if (i==0)
 {
  ai->st.ballID=0;
  ai->st.bvx=0;
  ai->st.bvy=0;
  ai->st.ballID=0;
 }

 if (ai->st.ballID==1)
 {
  fprintf(stderr,"Found the ball for blobId=%d, at (%f,%f), with velocity v=[%f %f]\n",ai->st.ball->blobId,\
          ai->st.ball->cx[0],ai->st.ball->cy[0],ai->st.bvx,ai->st.bvy);
 }

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 // Drive a simple pattern forward/reverse and call the particle filter to
 // find the blobs that are consistent with this in order to identify the
 // bot as well as the opponent.

 double noiseV=5;			// Expected motion magnitude due to noise for a static object
					// (it's just a guess - haven't measured it)
 double oppSizeT=2500;			// Opponent's blob nimimum size threshold
 static double blobData[10][4];	// Keep track of up to 10 blobs that could be
					// the bot. Unlikely there are more since
					// these have to be active tracked blobs.
					// blobData[i][:]=[blobID mx my p]
 static int bdataidx=0;
 double psum;
 double px,siz,len,vx,vy,wx,wy;
 int ownID,i;
 struct blob *oppID;

 struct blob *p;
 static double IDstep=0;		// Step tracker
 double frameInc=1.0/10;		// 1 over the number of frames each step should get

 // Get a handle on the ball
 id_ball(ai,blobs);
 
 // Forward motion for a few frames
 if (IDstep<3*frameInc)
 {
  drive_speed(35);
  clear_motion_flags(ai);
  ai->st.mv_fwd=1;
 }
 else if (IDstep<1.0)
 {
  drive_speed(35);
  clear_motion_flags(ai);
  ai->st.mv_fwd=1;
  particle_filter(ai,blobs);
 }
 else if (IDstep<1.0+(3*frameInc))	// Reverse drive
 {
  drive_speed(-35);
  clear_motion_flags(ai);
  ai->st.mv_back=1;
 }
 else if (IDstep<2.0)
 {
  drive_speed(-35);
  clear_motion_flags(ai);
  ai->st.mv_back=1;
  particle_filter(ai,blobs);  
 }
 else if (IDstep<2.0+(5*frameInc)) {clear_motion_flags(ai); all_stop();}	// Stop and clear blob motion history
 else {IDstep=0; ai->st.state=1;}	// For debug...
 		 
 IDstep+=frameInc; 

}

void confused(struct RoboAI *ai, struct blob *blobs, void *state)
{
	int dx, dy;
	if (0 == blobs) {
	  pivot_left_speed(25);
	  return;
	}

	dx = blobs->cx[0] * 100 / (blobs->cx[0] + blobs->cy[0]);
	dy = blobs->cy[0] * 100 / (blobs->cx[0] + blobs->cy[0]);
	if (dx - dy > -10 && dx - dy < 10)
		turn_left_speed(dx - dy);
	else
		pivot_left_speed(dx - dy);
}

// Provided --------------------------------
int setupAI(int mode, struct RoboAI *ai)
{
	//Note: set state to 0 if it is not used!
	int i;

	switch (mode) {
	case AI_CONFUSED:
		ai->state = 0;
		ai->runAI = confused;
		break;
        case AI_PACO:
		ai->state = NULL;
		ai->runAI = AI_main;
                ai->st.side=0;
		ai->st.state=0;
		all_stop();
		ai->st.ball=NULL;
		ai->st.old_bcx=0;
		ai->st.old_bcy=0;
		ai->st.selfID=0;
		ai->st.oppID=0;
		ai->st.ballID=0;
		clear_motion_flags(ai);
                break;
	default:
		fprintf(stderr, "AI mode %d is not implemented, sorry\n", mode);
		return 0;
	}

	return 1;
}

void cleanupAI(struct RoboAI *ai)
{
	if (ai->state)
		free(ai->state);
}


