/**************************************************************************
  CSC C85 - Fall 2013 - UTSC RoboSoccer AI core

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
  - Hog the ball (kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Release version: 0.1
***************************************************************************/

#include "imagecapture/imageCapture.h"
#include "roboAI.h"         // <--- Look at this header file!
#include <nxtlibc/nxtlibc.h>
#include <stdio.h>
#include <stdlib.h>


#define ANGLE_TOL 0.0
#define PI 3.14159265359
#define MOVE_SPEED 100

/*states*/
#define PENALTY 101
#define KICK_LEFT 102
#define KICK_RIGHT 103
#define SOCCER 1
#define CHASE 201
#define P_CHASE_TO_LEFT 104
#define P_CHASE_TO_RIGHT 105

int direction = 1;
void clear_motion_flags(struct RoboAI *ai)
{
    // Reset all motion flags. See roboAI.h for what each flag represents
    ai->st.mv_fwd = 0;
    ai->st.mv_back = 0;
    ai->st.mv_bl = 0;
    ai->st.mv_br = 0;
    ai->st.mv_fl = 0;
    ai->st.mv_fr = 0;
}

struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col)
{
    // ** DO NOT CHANGE THIS FUNCTION **
    // Find a blob with the specified colour. If similar colour blobs around,
    // choose the most saturated one.
    // Colour parameter: 0 -> R
    //                   1 -> G
    //                   2 -> B
    struct blob *p, *fnd;
    double BCRT = 1.0;         // Ball colour ratio threshold
    double c1, c2, c3, m;
    int i;

    p = blobs;
    fnd = NULL;
    while (p != NULL)
    {
        if (col == 0)
        {
            c1 = p->R;    // detect red
            c2 = p->G;
            c3 = p->B;
        }
        else if (col == 1)
        {
            c1 = p->G;    // detect green
            c2 = p->R;
            c3 = p->B;
        }
        else if (col == 2)
        {
            c1 = p->B;    // detect blue
            c2 = p->G;
            c3 = p->R;
        }

        if (c1 / c2 > BCRT && c1 / c3 > BCRT && p->tracked_status == 1) // tracked coloured blob!
        {
            //   fprintf(stderr,"col=%d, c1/c2=%f, c1/c3=%f, blobId=%d\n",col,c1/c2,c1/c3,p->blobId);
            if (c1 / c2 < c1 / c3) m = c1 / c2; else m = c1 / c3;

            if (fnd == NULL) fnd = p;        // first one so far
            else if (col == 0)              // Fond a more colorful one!
                if (m > (fnd->R / fnd->G) || m > (fnd->R / fnd->B)) fnd = p;
                else if (col == 1)
                    if (m > (fnd->G / fnd->R) || m > (fnd->G / fnd->B)) fnd = p;
                    else if (m > (fnd->B / fnd->G) || m > (fnd->B / fnd->R)) fnd = p;
        }
        p = p->next;
    }
    // if (fnd!=NULL) fprintf(stderr,"Selected for col=%d, blobId=%d\n",col,fnd->blobId);
    return (fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
    // ** DO NOT CHANGE THIS FUNCTION **
    // Find the current blobs that correspond to the bot, opponent, and ball
    //  by detecting blobs with the specified colours. One bot is green,
    //  one bot is blue. Ball is always red.

    // NOTE: Add a command-line parameter to specify WHICH bot is own

    struct blob *p;
    double mg, vx, vy;
    double NOISE_VAR = 15;

    // Find the ball
    p = id_coloured_blob(ai, blobs, 0);
    if (p)
    {
        if (ai->st.ball != NULL && ai->st.ball != p)
        {
            ai->st.ball->idtype = 0;
            ai->st.ball->p = 0;
        }
        ai->st.ballID = 1;
        ai->st.ball = p;
        ai->st.bvx = p->cx[0] - ai->st.old_bcx;
        ai->st.bvy = p->cy[0] - ai->st.old_bcy;
        ai->st.old_bcx = p->cx[0];
        ai->st.old_bcy = p->cy[0];
        ai->st.ball->p = 0;
        ai->st.ball->idtype = 3;
        vx = ai->st.bvx;
        vy = ai->st.bvy;
        mg = sqrt((vx * vx) + (vy * vy));
        if (mg > NOISE_VAR)       // Enable? disable??
        {
            ai->st.ball->vx[0] = vx;
            ai->st.ball->vx[1] = vy;
            vx /= mg;
            vy /= mg;
            ai->st.ball->mx = vx;
            ai->st.ball->my = vy;
        }
    }
    else
    {
        ai->st.ball = NULL;
    }

    // ID our bot
    if (ai->st.botCol == 0) p = id_coloured_blob(ai, blobs, 1);
    else p = id_coloured_blob(ai, blobs, 2);
    if (p)
    {
        if (ai->st.self != NULL && ai->st.self != p)
        {
            ai->st.self->idtype = 0;
            ai->st.self->p = 0;
        }
        ai->st.selfID = 1;
        ai->st.self = p;
        ai->st.svx = p->cx[0] - ai->st.old_scx;
        ai->st.svy = p->cy[0] - ai->st.old_scy;
        ai->st.old_scx = p->cx[0];
        ai->st.old_scy = p->cy[0];
        ai->st.self->p = 0;
        ai->st.self->idtype = 1;
    }
    else ai->st.self = NULL;

    // ID our opponent
    if (ai->st.botCol == 0) p = id_coloured_blob(ai, blobs, 2);
    else p = id_coloured_blob(ai, blobs, 1);
    if (p)
    {
        if (ai->st.opp != NULL && ai->st.opp != p)
        {
            ai->st.opp->idtype = 0;
            ai->st.opp->p = 0;
        }
        ai->st.oppID = 1;
        ai->st.opp = p;
        ai->st.ovx = p->cx[0] - ai->st.old_ocx;
        ai->st.ovy = p->cy[0] - ai->st.old_ocy;
        ai->st.old_ocx = p->cx[0];
        ai->st.old_ocy = p->cy[0];
        ai->st.opp->p = 0;
        ai->st.opp->idtype = 2;
    }
    else ai->st.opp = NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
    // ** DO NOT CHANGE THIS FUNCTION **
    // This routine calls track_agents() to identify the blobs corresponding to the
    // robots and the ball. It commands the bot to move forward slowly so heading
    // can be established from blob-tracking.
    //
    // NOTE 1: All heading estimates are noisy.
    //
    // NOTE 2: Heading estimates are only valid when the robot moves with a
    //         forward or backward direction. Turning destroys heading data
    //         (why?)
    //
    // You should *NOT* call this function during the game. This is only for the
    // initialization step. Calling this function during the game will result in
    // unpredictable behaviour since it will update the AI state.

    struct blob *p;
    static double stepID = 0;
    double frame_inc = 1.0 / 5.0;

    drive_speed(30);       // Need a few frames to establish heading

    track_agents(ai, blobs);

    if (ai->st.selfID == 1 && ai->st.self != NULL)
        fprintf(stderr, "Successfully identified self blob at (%f,%f)\n", ai->st.self->cx[0], ai->st.self->cy[0]);
    if (ai->st.oppID == 1 && ai->st.opp != NULL)
        fprintf(stderr, "Successfully identified opponent blob at (%f,%f)\n", ai->st.opp->cx[0], ai->st.opp->cy[0]);
    if (ai->st.ballID == 1 && ai->st.ball != NULL)
        fprintf(stderr, "Successfully identified ball blob at (%f,%f)\n", ai->st.ball->cx[0], ai->st.ball->cy[0]);

    stepID += frame_inc;
    if (stepID >= 1 && ai->st.selfID == 1)
    {
        ai->st.state += 1;
        stepID = 0;
        all_stop();
    }
    else if (stepID >= 1) stepID = 0;

    return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
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

    switch (mode)
    {
    case AI_SOCCER:
        fprintf(stderr, "Standard Robo-Soccer mode requested\n");
        ai->st.state = 0;   // <-- Set AI initial state to 0
        break;
    case AI_PENALTY:
        fprintf(stderr, "Penalty mode! let's kick it!\n");
        ai->st.state = 100; // <-- Set AI initial state to 100
        break;
    case AI_CHASE:
        fprintf(stderr, "Chasing the ball...\n");
        ai->st.state = 200; // <-- Set AI initial state to 200
        break;
    default:
        fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
        ai->st.state = 0;
    }

    all_stop();            // Stop bot,
    ai->runAI = AI_main;       // and initialize all remaining AI data
    ai->st.ball = NULL;
    ai->st.self = NULL;
    ai->st.opp = NULL;
    ai->st.side = 0;
    ai->st.botCol = own_col;
    ai->st.old_bcx = 0;
    ai->st.old_bcy = 0;
    ai->st.old_scx = 0;
    ai->st.old_scy = 0;
    ai->st.old_ocx = 0;
    ai->st.old_ocy = 0;
    ai->st.bvx = 0;
    ai->st.bvy = 0;
    ai->st.svx = 0;
    ai->st.svy = 0;
    ai->st.ovx = 0;
    ai->st.ovy = 0;
    ai->st.selfID = 0;
    ai->st.oppID = 0;
    ai->st.ballID = 0;
    clear_motion_flags(ai);
    fprintf(stderr, "Initialized!\n");
    return (1);
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
    /*************************************************************************
     You will be working with a state-based AI. You are free to determine
     how many states there will be, what each state will represent, and
     what actions the robot will perform based on the state as well as the
     state transitions.

     You must *FULLY* document your state representation in the report

     Here two states are defined:
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

    if (ai->st.state == 0 || ai->st.state == 100 || ai->st.state == 200) // Initial set up - find own, ball, and opponent blobs
    {
        // Carry out self id process.
        fprintf(stderr, "Initial state, self-id in progress...\n");
        id_bot(ai, blobs);
        if ((ai->st.state % 100) != 0) // The id_bot() routine will change the AI state to initial state + 1
        {
            // if robot identification is successful.
            if (ai->st.self->cx[0] >= 512) ai->st.side = 1; else ai->st.side = 0;
            all_stop();
            clear_motion_flags(ai);
            fprintf(stderr, "Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n", ai->st.self->cx[0], ai->st.self->cy[0], ai->st.self->mx, ai->st.self->my, ai->st.state);
        }
    }
    else
    {
        /****************************************************************************
         TO DO:
         You will need to replace this 'catch-all' code with actual program logic to
         have the robot do its work depending on its current state.
         After id_bot() has successfully completed its work, the state should be
         1 - if the bot is in SOCCER mode
         101 - if the bot is in PENALTY mode
         201 - if the bot is in CHASE mode

         Your AI code needs to handle these states and their associated state
         transitions which will determine the robot's behaviour for each mode.
        *****************************************************************************/
        // track_agents(ai,blobs);        // Currently, does nothing but endlessly track
        // fprintf(stderr,"Just trackin'!\n");    // bot, opponent, and ball.

        switch(ai->st.state){
            case 101:

                kick();
                sleep(2);
                break;
            case 201:
                chase(ai);
                break;
        }
    }

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
int my_round(double number)
{
    int ret = round(number);
    if (ret > 100)
    {
        ret = 100;
    }
    else if (ret < -100)
    {
        ret = -100;
    }
    return ret;
}
void apply_power(int left_power, int right_power)
{
    if (direction > 0)
    {
        drive_custom(left_power, right_power);
    }
    else
    {
        drive_custom(-right_power, -left_power);
    }
    fprintf(stderr, "left: %d right: %d", left_power, right_power);
    // sleep(0.1);
    // all_stop();
    // sleep(5);

}

void move(double theta)
{
    double left_power = 0.0;
    double right_power = 0.0;
    if (theta >= ANGLE_TOL)
    {
        left_power = MOVE_SPEED;
        right_power = cos(theta ) * MOVE_SPEED;
    }
    else if (theta <= -ANGLE_TOL)
    {
        right_power = MOVE_SPEED;
        left_power = cos(theta ) * MOVE_SPEED;
    }
    else
    {
        left_power = MOVE_SPEED;
        right_power = MOVE_SPEED;
    }

    apply_power(my_round(left_power), my_round(right_power));
}
void chase(struct RoboAI *ai)
{
    // ball position
    double xb = *(ai->st.ball->cx);
    double yb = -(*(ai->st.ball->cy));
    // robot position
    double yr = -(*(ai->st.self->cy));
    double xr = *(ai->st.self->cx);
    // vactor to the ball
    double d[2] = {xb - xr, yb - yr};
    // heading
    double h[2] = {(ai->st.self->mx), -(ai->st.self->my)};
    // theta
    double td = atan2(d[0], d[1]);
    double th = atan2(h[0], h[1]);
    // theta between two vectors
    double theta = atan2(sin(td)*cos(th) - sin(th)*cos(td), cos(td)*cos(th) + sin(td)*sin(th));
    // if (theta > PI / 2.0)
    // {
    //     theta = theta - PI;
    //     direction = -direction;
    // }
    // else if (theta < -(PI / 2.0))
    // {
    //     theta = theta + PI;
    //     direction = -direction;
    // }
    //fprintf(stderr, "Robot: Current position: (%f,%f), current heading: %f, AI state=%d\n", ai->st.self->cx[0], ai->st.self->cy[0], atan2(ai->st.self->mx, ai->st.self->my), ai->st.state);
    //fprintf(stderr, "Ball: Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n", ai->st.ball->cx[0], ai->st.ball->cy[0], ai->st.ball->mx, ai->st.ball->my, ai->st.state);
    fprintf(stderr, "Theta: %f, Heading: %f, dir: %f\n", theta, atan2(h[0], h[1]), atan2(d[0], d[1]));
    fprintf(stderr, "H: [%f, %f], D: [%f, %f]\n", h[0], h[1], d[0], d[1]);
    move(theta);
}

