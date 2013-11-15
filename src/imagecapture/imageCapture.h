/***************************************************************
 CSC C85 - Fall 2013 - UTSC RoboSoccer image processing core

 This file contains the headers for all functions in the image
 processing code. This part of the robo-soccer code handles
 camera input, blob detection/tracking, and user input via
 the keyboard as well as OpenGL display.

 You *DO NOT* have to look at the code here or in imageCapture.c,
 however, you can (if you think it is useful) access the data
 stored for each blob, or even the images obtained by this
 code to use within your AI.

 If you wish to do that, look at the definition of the blob
 data structure, and drop by my office to discuss your plans.

 Image processing code - F. Estrada, Summer 2013
****************************************************************/

#ifndef __imageCapture_header

#define __imageCapture_header

// Image processing library from time-lapse fusion code
#include"imageProc.h"

// Open GL libs
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <time.h>

// libraries from luvcview
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <X11/Xlib.h>
#include "v4l2uvc.h"
#include "gui.h"
#include "utils.h"
#include "color.h"
/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))

#define INCPANTILT 64 // 1°

static const char version[] = "RoboSoccer v1.0.2013";

struct blob{
        int label;		// Label in the labels image
	int blobId;		// Unique blobId
	double cx[5];		// Current and previous 4 centroid locations (for tracking)
        double cy[5];		
	double vx[5];		// [vx, vy] smothed motion vectors for the current and past 4 frames
        double vy[5];
        double mx,my;		// Heading direction - Last non-zero motion direction - unit vector!
	int size;		// Size in pixels
	int x1,y1,x2,y2;	// Bounding box (top-left, bottom-right)
	double R,G,B;		// Average colour of pixels in the blob
	int tracked_status;	// 0 - not tracked, 1 - tracked
        int age;		// Number of frames the blob has been tracked/not tracked
	double p;		// Probability value, useful for particle filtering.
	int idtype;		// Integer set to the id of this blob: 0->ball, 1->green bot, 2->blue bot
	struct blob *next;	// If needed for linked lists of blobs
        int updated;		// Update flag - used by the image processing loop - don't change it!
};

// Startup
int imageCaptureStartup(char *devName, int rx, int ry, int own_col, int ai_mode);

// OpenGL, GLUT, and main loop
void initGlut(char* winName);
void kbHandler(unsigned char key, int x, int y);
void kbUpHandler(unsigned char key, int x, int y);
void WindowReshape(int w, int h);
void FrameGrabLoop(void);

// Webcam setup and frame capture
unsigned char *yuyv_to_rgb (struct vdIn *vd, int sx, int sy);
struct vdIn *initCam(const char *videodevice, int width, int height);
unsigned char *getFrame(struct vdIn *videoIn, int sx, int sy);
void closeCam(struct vdIn *videoIn);

// Frame processing
void fieldUnwarp(double *H, struct image *im);
void bgSubtract(void);
void releaseBlobs(struct blob *blobList);
struct image *blobDetect(unsigned char *fgIm, int sx, int sy, struct blob **blob_list, int *nblobs);
struct image *renderBlobs(unsigned char *fgIm, int sx, int sy, struct image *labels, struct blob *list);
void blobTrack(struct blob **tracked, struct blob *current);
void drawLine(int x1, int y1, double vx, double vy, double scale, double R, double G, double B, struct image *dst);
void drawBox(int x1, int y1, int x2, int y2, double R, double G, double B, struct image *dst);
void drawCross(int mcx, int mcy, double R, double G, double B, int len, struct image *dst);

#endif

