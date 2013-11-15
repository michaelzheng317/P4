#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"

struct AI_data{
// Insert here any daya that your AI needs to do its work

	// blobIDs and blob data for the bot, the ball, and the opponent
	int side;		// side=0 implies the robot's own side is the left side
				// side=1 implies the robot's own side is the right side
				// This is initialized based on the robot's initial position
				// on the field

	int state;		// Current AI state

	// Motion flags
	int mv_fwd;		// forward
	int mv_back;		// backward
	int mv_bl;		// moving back while turning left
	int mv_br;		// moving back while turning right	
	int mv_fl;		// moving forward while turning left
	int mv_fr;		// moving forward while turning right

	// Object ID status for self and opponent
	int selfID;
	int oppID;

	// Blob track data. Ball likely needs to be detected at each frame
	// separately. So we keep old location to estimate v
	struct blob *ball;		// Current ball blob
	double old_bcx, old_bcy;	// Previous ball cx,cy
	double bvx,bvy;		// Ball velocity vector
	int ballID;
};

struct RoboAI {
	void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
	struct AI_data st;
	void *state;
};

#define AI_CONFUSED 0 //A confused robot
#define AI_PER 1 //The ballos cruncher
#define AI_DAVID 2 //Does backflips while discarding (useless?) pieces
#define AI_KEVIN 3 //Treads of doom
#define AI_PACO 4 //8x9 Horadric hamburger solver (MSeGfault free)

/**
 * \brief Set up an AI structure for playing roboSoccer
 *
 * Set up an AI structure for playing roboSoccer. Must be
 * called before using the AI structure during gameplay.
 * \param[in] mode The operational mode for the AI
 * \param[out] ai A structure containing data necessary for
 * 		AI algorithms
 * \pre ai is uninitialized
 * \post ai is set up for use, and must be cleaned up using
 * 		cleanupAI
 */
int setupAI(int mode, struct RoboAI *ai);

/**
 * \brief Clean up an AI structure
 *
 * Clean up an AI structure. Some AI implementations may not
 * require cleanup, but this should be called regardless.
 * \param[out] ai The RoboAI structure to clean up
 * \post ai is no longer a valid AI structure
 */
void cleanupAI(struct RoboAI *ai);

/**
 * \brief Top-level AI loop.
 * 
 * Decides based on current state and blob configuration what
 * the bot should do next, and calls the appropriate behaviour
 * function.
 *
 * \param[in] ai, pointer to the data structure for the running AI
 * \param[in] blobs, pointer to the current list of tracked blobs
 * \param[out] void, but the state description in the AI structure may have changed
 * \pre ai is not NULL, blobs is not NULL
 * \post ai is not NULL, blobs is not NULL
 */
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state);

/* PaCode */
void id_bot(struct RoboAI *ai, struct blob *blobs);
void id_ball(struct RoboAI *ai, struct blob *blobs);
void clear_motion_flags(struct RoboAI *ai);
void particle_filter(struct RoboAI *ai, struct blob *blobs);
#endif
