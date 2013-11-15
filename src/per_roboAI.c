#include "imagecapture/imageCapture.h"
#include "roboAI.h"
#include <stdio.h>
#include <stdlib.h>

// Your AI goes here! ----------------------
void confused(struct RoboAI *ai, struct blob *blobs, void *data)
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

	switch (mode) {
	case AI_CONFUSED:
		ai->state = 0;
		ai->runAI = confused;
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


