#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"

struct RoboAI {
	void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
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


#endif
