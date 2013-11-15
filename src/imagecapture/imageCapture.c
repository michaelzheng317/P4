/************************************************************************************
 CSC C85 - Fall 2013 - UTSC RoboSoccer image processing core

 Implementation of the image processing functionality for robo-soccer.
 This code initializes the web-cam, obtains a rectified playfield from
 the corners specified by the user, detects the bots and ball, and
 processes the user's keyboard input as well as the OpenGL display.

 You *DO NOT* have to look at the code here or in imageCapture.h,
 however, you can (if you think it is useful) access the data
 stored for each blob, or even the images obtained by this
 code to use within your AI.

 * DO NOT SPEND TIME ON THIS CODE UNTIL YOU HAVE A MOSTLY WORKING
   AI CORE that at the very least can drive the bot toward the ball
   to kick it.

 If you wish to do that, look at the definition of the blob
 data structure, and drop by my office to discuss your plans.

 Image processing code - F. Estrada, Summer 2013

 Code for initializing the camera and grabbing a frame adapted                    
                    from luvcview and uvccapture.                                
                    luvcview and uvccapture by Laurent Pinchart & Michel Xhaard  
		    This code is distributed under GPL V2.0. Therefore:
                                                                                  
 As per the GLP V2.0 license terms. This code is hereby distributed freely. You   
  are free to modify and redistribute this code under the provisions of the      
  GPL V2 or newer. A copy of the license should be included with this code,      
  but should this not be the case, please consult                                 
  http://www.gnu.org/licenses/gpl-2.0.html                                       
************************************************************************************/

#include "imageCapture.h"
#include "svdDynamic.h"
#include "../roboAI.h"
#include <nxtlibc/nxtlibc.h>
#include <time.h>

// Global data for OpenGL display and current frame pointer
// ** DO NOT MODIFY THESE GLOBAL VARIABLE DEFINITIONS **
int toggleProc;			// Toggles processing on/off
GLuint texture;			    // GL texture identifier 
int windowID;               	// Glut window ID (for display)
int Win[2];                 	// window (x,y) size
int sx,sy;			// Image size - set by the webcam init function
unsigned char *im;		// The current frame in RGB
struct vdIn *webcam;		// The input video device
struct image *proc_im;		// Image structure for processing
double thr=.58;			// Threshold for track detection 
unsigned char bigIm[1024*1024*3];	// Big texture to hold our image for OpenGL
unsigned char fieldIm[1024*768*3]; // Unwarped field 
unsigned char bgIm[1024*768*3];	// Background image
int gotbg=0;			// Background acquired flag
double Mcorners[4][2];		// Manually specified corners for the field
double mcx,mcy;			// Crosshair x and y
int cornerIdx;			// Index for manual corner selection.
double bgThresh=2000;		// Background subtract threshold
double colThresh=.90;		// Colour detection threshold
double colAngThresh=.725;
int blob_num=0;			// Global blobId for blob natching purposes
struct blob *blobs=NULL;		// Blob list for the current frame * DO NOT USE THIS LIST *
struct blob *tracked_blobs=NULL;	// Tracked blob list * USE THIS ONE *
double *H = NULL;		// Homography matrix for field rectification
int frameNo=0;			// Frame id
time_t time1,time2;		    // timing variables
int DIR_L=0, DIR_R=0, DIR_FWD=0, DIR_BACK=0;	// Toggle flags for robot motion
struct RoboAI skynet;		// Bot's AI structure
int doAI=0;			// Toggle switch for AI processing
int printFPS=0;			// Flag that controls FPS printout
int AIMode = 0;			// AI mode, as per command line
int botCol = 0;			// Own bot's colour as per command line

/*********************************************************************
 OpenGL display setup.
*********************************************************************/
// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    // Create window
    glutInitWindowPosition (0, 0);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle window-related events.
    // In particular, OpenGL has to be informed of which functions
    // to call when the image needs to be refreshed, and when the
    // image window is being resized.
    glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
    glutDisplayFunc(FrameGrabLoop);   // Main display function is also the main loop
    glutKeyboardFunc(kbHandler);
}

void kbHandler(unsigned char key, int x, int y)
{
 // Exit!
 if (key=='q') 
 {
  all_stop();				// < - Re-enable when bots are ready!
  releaseBlobs(tracked_blobs);
  deleteImage(proc_im);
  glDeleteTextures(1,&texture);
  closeCam(webcam);
  exit(0);
 }

 // Toggle AI processing on/off
 if (key=='t') if (doAI) doAI=0; else doAI=1;
 if (key=='r') {setupAI(AIMode,botCol,&skynet); doAI=0;}		// Resets the state of the AI (may need full reset)

 // Controls for recording the corners of the playing field
 if (key=='m')
 {
  if (toggleProc==1) toggleProc=0;
  memset(&Mcorners[0][0],0,8*sizeof(double));
  cornerIdx=0;
  toggleProc=2;		// Indicates manual corner detection active  
  mcx=340;
  mcy=340;
 }
 if (key=='d'&&toggleProc==2) if (mcx<sx-1) mcx++;
 if (key=='a'&&toggleProc==2) if (mcx>1) mcx--;
 if (key=='w'&&toggleProc==2) if (mcy>1) mcy--;
 if (key=='s'&&toggleProc==2) if (mcy<sy-1) mcy++;
 if (key=='D'&&toggleProc==2) if (mcx<sx-6) mcx+=5;
 if (key=='A'&&toggleProc==2) if (mcx>5) mcx-=5;
 if (key=='W'&&toggleProc==2) if (mcy>5) mcy-=5;
 if (key=='S'&&toggleProc==2) if (mcy<sy-6) mcy+=5;
 if (key==' '&&toggleProc==2) 
 {
  fprintf(stderr,"Recorded corner %d at %f,%f\n",cornerIdx,mcx,mcy);
  Mcorners[cornerIdx][0]=mcx;
  Mcorners[cornerIdx][1]=mcy;
  cornerIdx++;
  if (cornerIdx==4) toggleProc=0;	// Done!
 }

 // Image processing controls
 if (key=='<') {bgThresh-=5;fprintf(stderr,"BG subtract threshold now at %f\n",bgThresh);}
 if (key=='>') {bgThresh+=5;fprintf(stderr,"BG subtract threshold now at %f\n",bgThresh);}
 if (key=='{') {colAngThresh-=.01;fprintf(stderr,"Colour Angle Threshold now at %f\n",colAngThresh);}
 if (key=='}') {colAngThresh+=.01;fprintf(stderr,"Colour Angle Threshold now at %f\n",colAngThresh);}
 if (key=='[') {colThresh-=.05;fprintf(stderr,"Colour threshold now at %f\n",colThresh);}
 if (key==']') {colThresh+=.05;fprintf(stderr,"Colour threshold now at %f\n",colThresh);}
 if (key=='f') {if (printFPS==0) printFPS=1; else printFPS=0;}

 // NXT robot manual override
 if (key=='i') {if (DIR_FWD==0) {DIR_FWD=1; DIR_L=0; DIR_R=0; DIR_BACK=0; drive_speed(75);} else {DIR_FWD=0; all_stop();}}
 if (key=='j') {if (DIR_L==0) {DIR_L=1; DIR_R=0; DIR_FWD=0; DIR_BACK=0; pivot_left_speed(50);} else {DIR_L=0; all_stop();}}
 if (key=='l') {if (DIR_R==0) {DIR_R=1; DIR_L=0; DIR_FWD=0; DIR_BACK=0; pivot_right_speed(50);} else {DIR_R=0; all_stop();}}
 if (key=='k') {if (DIR_BACK==0) {DIR_BACK=1; DIR_L=0; DIR_R=0; DIR_FWD=0; reverse_speed(75);} else {DIR_BACK=0; all_stop();}}
 if (key=='o') {all_stop();doAI=0;}	// <-- Important!
}

void WindowReshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();			// Initialize with identity matrix
    gluOrtho2D(0, 800, 800, 0);
    glViewport(0,0,w,h);
    Win[0] = w;
    Win[1] = h;
}
/*********************************************************************
End of OpenGL display setup
*********************************************************************/

/*********************************************************************
Camera initialization, frame grab, and frame conversion. 
*********************************************************************/

unsigned char *yuyv_to_rgb (struct vdIn *vd, int sx, int sy)
{
  // Allocate and return a frame buffer containing one captured
  // frame in standard RGB format. The size of the frame is 
  // returned via sx and sy.
  // Derived from compress_yuyv_to_jpeg() in uvccapture.c

  unsigned char *frame_buffer, *yuyv;
  int z;
  int row=0;
  int ii;
  unsigned char *ptr;

  frame_buffer = (unsigned char *)calloc (vd->height*vd->width * 3, sizeof(unsigned char));
  if (!frame_buffer)
  {
   fprintf(stderr,"yuyv_to_rgb(): Can not allocate memory for frame buffer.\n");
   return(NULL);
  }

#pragma omp parallel for schedule(dynamic,32) private(ii,ptr,yuyv,z)
  for (ii=0;ii<vd->height;ii++)
  {
   int x;
   ptr=frame_buffer+((ii*vd->width)*3);
   yuyv=vd->framebuffer+((ii*vd->width)*2);
   for (x = 0; x < vd->width; x+=2) 
   {
    int r, g, b;
    int y, u, v;
    y = yuyv[0] << 8;
    u = yuyv[1] - 128;
    v = yuyv[3] - 128;

    r = (y + (359 * v)) >> 8;
    g = (y - (88 * u) - (183 * v)) >> 8;
    b = (y + (454 * u)) >> 8;

    *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
    *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
    *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

    y = yuyv[2] << 8;
    r = (y + (359 * v)) >> 8;
    g = (y - (88 * u) - (183 * v)) >> 8;
    b = (y + (454 * u)) >> 8;

    *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
    *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
    *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);
    yuyv += 4;
   }   // End for x  
  }  
  return (frame_buffer);
}
    
struct vdIn *initCam(const char *videodevice, int width, int height)
{
 /* 
    Camera initialization - sets up the camera communication, image
    format, fps, and other camera parmeters. Derived from uvccapture.c
 */
	int status;
	unsigned char *p = NULL;
	int hwaccel = 0;
	const char *mode = NULL;
	int format = V4L2_PIX_FMT_YUYV;
	int i;					
	int grabmethod = 1;
	int fps = 30;
	unsigned char frmrate = 0;
	char *avifilename = NULL;
	int queryformats = 0;
	int querycontrols = 0;
	int readconfigfile = 0;
	char *separateur;
	char *sizestring = NULL;
	char *fpsstring  = NULL;
	int enableRawStreamCapture = 0;
	int enableRawFrameCapture = 0;
	struct vdIn *videoIn;
	FILE *file;

	printf("roboSoccer v1.0.2013\n\n");

	if (avifilename == NULL || *avifilename == 0) {
		avifilename = "video.avi";
	}
	videoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));

	if (init_videoIn
			(videoIn, (char *) videodevice, width, height, fps, format,
			 grabmethod, avifilename) < 0)
		return(NULL);
	/* if we're supposed to list the controls, do that now */
	if ( querycontrols )
		enum_controls(videoIn->fd);

	/* if we're supposed to read the control settings from a configfile, do that now */
	if ( readconfigfile )
		load_controls(videoIn->fd);

	return(videoIn);		// Successfully opened a video device
}

unsigned char *getFrame(struct vdIn *videoIn, int sx, int sy)
{
 /*
   Grab a single frame from the camera. Derived from uvccapture.c
 */
	unsigned char *frame;
        double dtime;

	// Grab a frame from the video device
	if (uvcGrab(videoIn) < 0) {
		printf("Error grabbing\n");
		return(NULL);
	}
        frame=yuyv_to_rgb(videoIn, sx, sy); 
        videoIn->getPict = 0;

	// Print FPS if needed.
        frameNo++;
        time(&time2);
        dtime=difftime(time2,time1);
        if (printFPS) fprintf(stderr,"FPS= %f\n",(double)frameNo/dtime);
         
        return(frame);
}

void closeCam(struct vdIn *videoIn)
{
	close_v4l2(videoIn);
	free(videoIn);
}

/*********************************************************************
End of Webcam manipulation code
*********************************************************************/

/*********************************************************************
Frame processing functions
**********************************************************************/
void fieldUnwarp(double *H, struct image *im)
{
 // Unwarp the image of the field using the estimated homography
 // H. Bilinear interpolation filtering for smooth results.

 int i,j;
 double px,py,pw;
 double dx,dy;
 unsigned char *fi;
 double r1,g1,b1,r2,g2,b2,r3,g3,b3,r4,g4,b4;
 double R,G,B;
 
 if (H==NULL) return;

 fi=&fieldIm[0];
 memset(fi,0,1024*768*3*sizeof(unsigned char));

#pragma omp parallel for schedule(dynamic,32) private(i,j,px,py,pw,dx,dy,R,G,B,r1,g1,b1,r2,g2,b2,r3,g3,b3,r4,g4,b4)
 for (j=1;j<767;j++)
  for (i=1;i<1023;i++)
  {
   // Obtain coordinates for this pixel in the unwarped image
   px=((*(H+0))*i) + ((*(H+1))*j) + (*(H+2));
   py=((*(H+3))*i) + ((*(H+4))*j) + (*(H+5));
   pw=((*(H+6))*i) + ((*(H+7))*j) + (*(H+8));
   px=px/pw;
   py=py/pw;
   dx=px-(int)px;
   dy=py-(int)py;
   if (px>0&&px<im->sx-1&&py>0&&py<im->sy-1)
   {
    R=G=B=0;
    r1=*(im->layers[0]+ ((int)px) + (((int)py) * im->sx));
    g1=*(im->layers[1]+ ((int)px) + (((int)py) * im->sx));
    b1=*(im->layers[2]+ ((int)px) + (((int)py) * im->sx));
    r2=*(im->layers[0]+ ((int)px+1) + (((int)py) * im->sx));
    g2=*(im->layers[1]+ ((int)px+1) + (((int)py) * im->sx));
    b2=*(im->layers[2]+ ((int)px+1) + (((int)py) * im->sx));
    r3=*(im->layers[0]+ ((int)px) + (((int)py+1) * im->sx));
    g3=*(im->layers[1]+ ((int)px) + (((int)py+1) * im->sx));
    b3=*(im->layers[2]+ ((int)px) + (((int)py+1) * im->sx));
    r4=*(im->layers[0]+ ((int)px+1) + (((int)py+1) * im->sx));
    g4=*(im->layers[1]+ ((int)px+1) + (((int)py+1) * im->sx));
    b4=*(im->layers[2]+ ((int)px+1) + (((int)py+1) * im->sx));
    r1=((1.0-dx)*r1)+(dx*r2);
    g1=((1.0-dx)*g1)+(dx*g2);
    b1=((1.0-dx)*b1)+(dx*b2);
    r3=((1.0-dx)*r3)+(dx*r4);
    g3=((1.0-dx)*g3)+(dx*g4);
    b3=((1.0-dx)*b3)+(dx*b4);
    R=((1.0-dy)*r1)+(dy*r3);
    G=((1.0-dy)*g1)+(dy*g3);
    B=((1.0-dy)*b1)+(dy*b3);
    *(fi+((i+(j*1024))*3)+0)=(unsigned char)(R);
    *(fi+((i+(j*1024))*3)+1)=(unsigned char)(G);
    *(fi+((i+(j*1024))*3)+2)=(unsigned char)(B);  
   }
  }

}

double *getH(void)
{
 // Compute a homography from manually selected corner points
 double cD[4],tcorners[4][2];
 double corners2[4][2]={1.0,1.0,1023.0,1.0,1023.0,767.0,1.0,767.0};
 double A[8][9],B[8][8],b[8],Binv[8][8];
 double *U,*s,*V,*rv1;
 double *H;
 int i,j;

 for (i=0;i<4;i++)
 {
  A[(2*i)+0][0]=0;
  A[(2*i)+0][1]=0;
  A[(2*i)+0][2]=0;
  A[(2*i)+0][3]=-corners2[i][0];
  A[(2*i)+0][4]=-corners2[i][1];
  A[(2*i)+0][5]=-1;
  A[(2*i)+0][6]=corners2[i][0]*Mcorners[i][1];
  A[(2*i)+0][7]=corners2[i][1]*Mcorners[i][1];
  A[(2*i)+0][8]=Mcorners[i][1];
  A[(2*i)+1][0]=corners2[i][0];
  A[(2*i)+1][1]=corners2[i][1];
  A[(2*i)+1][2]=1;
  A[(2*i)+1][3]=0;
  A[(2*i)+1][4]=0;
  A[(2*i)+1][5]=0;
  A[(2*i)+1][6]=-corners2[i][0]*Mcorners[i][0];
  A[(2*i)+1][7]=-corners2[i][1]*Mcorners[i][0];
  A[(2*i)+1][8]=-Mcorners[i][0];
 }

 // Allocate memory for the homography!
 H=(double *)calloc(9,sizeof(double));	// H should be the last row of U

 U=NULL;
 s=NULL;
 V=NULL;
 rv1=NULL;

 SVD(&A[0][0],8,9,&U,&s,&V,&rv1);
 // A note on sizes: A is 8x9, SVD should return U is 8x8, S is 8x9 
 // (or in this case a vector with 9 values), and V is 9x9. However,
 // Tom's code returns an 8x8 U, and a 9x8 V! missing last column

 // Now, because V is not all there, we have to solve for the eigenvector in V
 // that lies along the null-space of A. It has to be orthogonal to the 8
 // eigenvectors we just got in V. It also has 9 entries but fortunately we
 // can fix the last one to 1. That leaves an 8x8 non-homogeneous system we
 // can solve directly for.

 for (i=0;i<8;i++)
  for (j=0; j<8; j++)
   B[i][j]=(*(V+i+(j*8)));
 for (j=0; j<8; j++)
  b[j]=(-(*(V+j+64)));

 free(U);
 free(s);
 free(V);

 // Compute B^-1
 U=NULL;
 s=NULL;
 V=NULL;
 rv1=NULL;
 SVD(&B[0][0],8,8,&U,&s,&V,&rv1);
 InvertMatrix(U,s,V,8,&Binv[0][0]);
 free(U);
 free(s);
 free(V);

 // Finally, compute the eigenvector's first 8 entries
 for (i=0;i<8;i++)
 {
  *(H+i)=0;
  for (j=0; j<8;j++) (*(H+i))+=Binv[i][j]*b[j];
 }
 *(H+8)=1.0;
 
 fprintf(stderr,"Homography martrix:\n");
 fprintf(stderr,"hh=[%f %f %f\n",*(H),*(H+1),*(H+2));
 fprintf(stderr,"%f %f %f\n",*(H+3),*(H+4),*(H+5));
 fprintf(stderr,"%f %f %f\n];\n",*(H+6),*(H+7),*(H+8));
 
 for (i=0;i<4;i++)
 {
  memset(&cD[0],0,4*sizeof(double));
  cD[0]=((*(H+0))*corners2[i][0])+((*(H+1))*corners2[i][1])+((*(H+2)));
  cD[1]=((*(H+3))*corners2[i][0])+((*(H+4))*corners2[i][1])+((*(H+5)));
  cD[2]=((*(H+6))*corners2[i][0])+((*(H+7))*corners2[i][1])+((*(H+8)));
  fprintf(stderr,"Converts (%f,%f) to (%f,%f)\n",corners2[i][0],corners2[i][1],\
          cD[0]/cD[2],cD[1]/cD[2]);
 }
 return(H);
}

void bgSubtract(void)
{
 // Background subtraction. Zeroes-out any pixels that are too similar to
 // the background and are not colorful. Assumes neutral background such
 // as the lab's gray floor. Will not work well on a colorful floor!
 int j,i;
 double r,g,b,R,G,B,dd,mg;
 if (!gotbg) return;
#pragma omp parallel for schedule(dynamic,32) private(i,j,r,g,b,R,G,B,dd,mg)
 for (j=0;j<768;j++)
  for (i=0; i<1024; i++)
  {
   r=fieldIm[((i+(j*1024))*3)+0];
   g=fieldIm[((i+(j*1024))*3)+1];
   b=fieldIm[((i+(j*1024))*3)+2];
   R=bgIm[((i+(j*1024))*3)+0];
   G=bgIm[((i+(j*1024))*3)+1];
   B=bgIm[((i+(j*1024))*3)+2];

   dd=(r-R)*(r-R);
   dd+=(g-G)*(g-G);
   dd+=(b-B)*(b-B);
   mg=(r+g+b)/3.0;

   // Should be independent of brightness. Dark colour regions should not be lost.
   if (dd<bgThresh||(fabs((r-g)/mg)<colThresh&&fabs((r+g-(2*b))/mg)<2.0*colThresh))     // Background or gray pixel
   {  
    fieldIm[((i+(j*1024))*3)+0]=0;
    fieldIm[((i+(j*1024))*3)+1]=0;
    fieldIm[((i+(j*1024))*3)+2]=0;
   }
  }
}

struct image *blobDetect(unsigned char *fgIm, int sx, int sy, struct blob **blob_list, int *nblobs)
{
 // Perform blob detection on the foreground image (after background subtraction)
 // and generate:
 // - A label image, with a unique label for pixels in each blob (sx * sy * 1 layer)
 // - A list of blob descriptors (centroid, size in pixels, bounding box, average colour)
 // - The number of blobs found
 // 
 // This function will ignore blobs smaller than a certain size

 static int pixStack[1024*768*2];
 int i,j,x,y;
 int mix,miy,mx,my;
 int lab;
 double R,G,B;
 double Ra,Ga,Ba;
 double Yb,Rg,L,tL,tYb,tRg;
 double xc,yc;
 int pixcnt;
 int *stack;
 int stackPtr;
 struct image *labIm, *tmpIm;
 struct blob *bl;

 // Clear any previous list of blobs
 if (*(blob_list)!=NULL)
 {
  releaseBlobs(*(blob_list));
  *(blob_list)=NULL;
 }

 // Assumed: Pixels in the input fgIm that have non-zero RGB values are foreground
 labIm=newImage(sx,sy,1);			// 1-layer labels image
 tmpIm=imageFromBuffer(fgIm,sx,sy,3);
 stack=&pixStack[0];
 lab=1;		
 memset(stack,0,1024*768*2*sizeof(int));

// fprintf(stderr,"Main blob detection loop\n");
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   R=*(tmpIm->layers[0]+i+(j*sx));
   G=*(tmpIm->layers[1]+i+(j*sx));
   B=*(tmpIm->layers[2]+i+(j*sx));
   if (R+G+B>0)    // Found unlabeled pixel
   {
    Yb=((R+G)-(2.0*B)+2.0)*.25;
    Rg=((R-G)+1.0)*.5;
    L=sqrt((Yb*Yb)+(Rg*Rg));
    Yb/=L;
    Rg/=L;
    stackPtr=1;
    *(stack+(2*stackPtr))=i;
    *(stack+(2*stackPtr)+1)=j;
    *(tmpIm->layers[0]+i+(j*sx)) = -(*(tmpIm->layers[0]+i+(j*sx)));
    *(tmpIm->layers[1]+i+(j*sx)) = -(*(tmpIm->layers[1]+i+(j*sx)));
    *(tmpIm->layers[2]+i+(j*sx)) = -(*(tmpIm->layers[2]+i+(j*sx)));
    Ra=0;
    Ga=0;
    Ba=0;
    xc=0;
    yc=0;
    mix=10000;
    miy=10000;
    mx=-10000;
    my=-10000;
    pixcnt=0;
    while (stackPtr>0)
    {
     if (stackPtr>=1024*768) fprintf(stderr," *** Busted the stack!\n");
     x=*(stack+(2*stackPtr));
     y=*(stack+(2*stackPtr)+1);
     stackPtr--;
     *(labIm->layers[0]+x+(y*labIm->sx))=lab;
     Ra-=*(tmpIm->layers[0]+x+(y*tmpIm->sx));
     Ga-=*(tmpIm->layers[1]+x+(y*tmpIm->sx));
     Ba-=*(tmpIm->layers[2]+x+(y*tmpIm->sx));
     xc+=x;
     yc+=y;
     pixcnt++;
     *(tmpIm->layers[0]+x+(y*sx)) = 0;
     *(tmpIm->layers[1]+x+(y*sx)) = 0;
     *(tmpIm->layers[2]+x+(y*sx)) = 0;
     if (mix>x) mix=x;
     if (miy>y) miy=y;
     if (mx<x) mx=x;
     if (my<y) my=y;
     // Check neighbours
     if (y>0)
     {
      R=*(tmpIm->layers[0]+x+((y-1)*sx)); 
      G=*(tmpIm->layers[1]+x+((y-1)*sx)); 
      B=*(tmpIm->layers[2]+x+((y-1)*sx)); 
      tYb=((R+G)-(2.0*B)+2.0)*.25;
      tRg=((R-G)+1.0)*.5;
      tL=sqrt((tYb*tYb)+(tRg*tRg));
      tYb/=tL;
      tRg/=tL;
      if (R+G+B>0&&(tYb*Yb)+(tRg*Rg)>colAngThresh)
      {
       stackPtr++;
       *(stack+(2*stackPtr))=x;
       *(stack+(2*stackPtr)+1)=y-1;
       *(tmpIm->layers[0]+x+((y-1)*sx)) *= -1.0;
       *(tmpIm->layers[1]+x+((y-1)*sx)) *= -1.0;
       *(tmpIm->layers[2]+x+((y-1)*sx)) *= -1.0;
      }
     }
     if (x<sx-1)
     {
      R=*(tmpIm->layers[0]+x+1+(y*sx)); 
      G=*(tmpIm->layers[1]+x+1+(y*sx)); 
      B=*(tmpIm->layers[2]+x+1+(y*sx)); 
      tYb=((R+G)-(2.0*B)+2.0)*.25;
      tRg=((R-G)+1.0)*.5;
      tL=sqrt((tYb*tYb)+(tRg*tRg));
      tYb/=tL;
      tRg/=tL;
      if (R+G+B>0&&(tYb*Yb)+(tRg*Rg)>colAngThresh)
      {
       stackPtr++;
       *(stack+(2*stackPtr))=x+1;
       *(stack+(2*stackPtr)+1)=y;
       *(tmpIm->layers[0]+x+1+(y*sx)) *= -1.0;
       *(tmpIm->layers[1]+x+1+(y*sx)) *= -1.0;
       *(tmpIm->layers[2]+x+1+(y*sx)) *= -1.0;
      }
     }
     if (y<sy-1)
     {
      R=*(tmpIm->layers[0]+x+((y+1)*sx)); 
      G=*(tmpIm->layers[1]+x+((y+1)*sx)); 
      B=*(tmpIm->layers[2]+x+((y+1)*sx)); 
      tYb=((R+G)-(2.0*B)+2.0)*.25;
      tRg=((R-G)+1.0)*.5;
      tL=sqrt((tYb*tYb)+(tRg*tRg));
      tYb/=tL;
      tRg/=tL;
      if (R+G+B>0&&(tYb*Yb)+(tRg*Rg)>colAngThresh)
      {
       stackPtr++;
       *(stack+(2*stackPtr))=x;
       *(stack+(2*stackPtr)+1)=y+1;
       *(tmpIm->layers[0]+x+((y+1)*sx)) *= -1.0;
       *(tmpIm->layers[1]+x+((y+1)*sx)) *= -1.0;
       *(tmpIm->layers[2]+x+((y+1)*sx)) *= -1.0;
      }
     }
     if (x>0)
     {
      R=*(tmpIm->layers[0]+x-1+(y*sx)); 
      G=*(tmpIm->layers[1]+x-1+(y*sx)); 
      B=*(tmpIm->layers[2]+x-1+(y*sx)); 
      tYb=((R+G)-(2.0*B)+2.0)*.25;
      tRg=((R-G)+1.0)*.5;
      tL=sqrt((tYb*tYb)+(tRg*tRg));
      tYb/=tL;
      tRg/=tL;
      if (R+G+B>0&&(tYb*Yb)+(tRg*Rg)>colAngThresh)
      {
       stackPtr++;
       *(stack+(2*stackPtr))=x-1;
       *(stack+(2*stackPtr)+1)=y;
       *(tmpIm->layers[0]+x-1+(y*sx)) *= -1.0;
       *(tmpIm->layers[1]+x-1+(y*sx)) *= -1.0;
       *(tmpIm->layers[2]+x-1+(y*sx)) *= -1.0;
      }
     }
    }	// End while
    if (pixcnt>400)
    {
     // Insert blob
     Ra/=pixcnt;
     Ga/=pixcnt;
     Ba/=pixcnt;
     xc/=pixcnt;
     yc/=pixcnt;
     bl=(struct blob *)calloc(1,sizeof(struct blob));
     bl->label=lab;
     bl->blobId=-1;					
     memset(&(bl->cx),0,5*sizeof(double));
     memset(&(bl->cy),0,5*sizeof(double));
     memset(&(bl->vx),0,5*sizeof(double));
     memset(&(bl->vy),0,5*sizeof(double));
     bl->mx=0;
     bl->my=0;
     bl->cx[0]=xc;
     bl->cy[0]=yc;
     bl->size=pixcnt;
     bl->x1=mix;
     bl->y1=miy;
     bl->x2=mx;
     bl->y2=my;
     bl->R=Ra;
     bl->G=Ga;
     bl->B=Ba;
     bl->tracked_status=0;
     bl->age=0;
     bl->next=NULL;
     bl->idtype=0;
     bl->p=0;
     if (*(blob_list)==NULL) *(blob_list)=bl;
     else {bl->next=(*(blob_list))->next; (*(blob_list))->next=bl;}
    }
    lab++;
   }    // End if
  }   // End for i

 deleteImage(tmpIm);

 // Count number of blobs found
 bl=*blob_list;
 *(nblobs)=0;
 while (bl!=NULL)
 {
  *nblobs=(*nblobs) + 1;  
  bl=bl->next;
 }
 return(labIm);
} 

void releaseBlobs(struct blob *blobList)
{
 struct blob *p,*q;
 p=blobList;
 while(p)
 {
  q=p->next;
  free(p);
  p=q;
 }
}

void blobTrack(struct blob **tracked, struct blob *current)
{
 // Determine which of the current blobs correspond to previously detected blobs so we can
 // estimate a motion vector.

 struct blob *p, *q, *r;
 int found;
 int DIST_TOL;
 int SIZE_TOL;
 int BLOB_RECALL=10;
 double P_DECAY=.95;
 double COL_TOL=.35*255.0;
 double noiseV=15.0;		// Was 5.0
 int i,j,k;
 int bcount;
 double len;
 static clock_t blob_time1;
 static clock_t blob_time2;
 double timediff;

 blob_time2=clock();
 timediff=(double)(blob_time2-blob_time1)/CLOCKS_PER_SEC;
 blob_time1=blob_time2;

 bcount=0;
 p=*(tracked);
 while (p!=NULL) {p->updated=0; p=p->next; bcount++;}

 q=current;
 while (q!=NULL)
 {
  // Check whether this blob matches any existing blobs
  p=*(tracked);
  found=0;
  while (p!=NULL&&!found)
  {
   SIZE_TOL=(p->size)*.5;			// Difference in size no more than 50% of current size

   // Check that the centroids are close-by, the size is similar, and the average colour is
   // close enough. The distance is measured relative to the diagonal of the bounding box
   // containing a blob. 
   DIST_TOL=sqrt(((p->x2-p->x1)*(p->x2-p->x1))+((p->y2-p->y1)*(p->y2-p->y1)));

   if (p->vx[0]>noiseV||p->vy[0]>noiseV)
    DIST_TOL=DIST_TOL*2.5;			// For moving blobs, up to 2.5 * the diagonal of the bounding box
   else
    DIST_TOL=DIST_TOL*.35;			// For static blobs, should be close enough

   // Tested:
   // - Blob is active
   // - Distance between centroids is less than DIST_TOL as computed above
   // - Size within the tolerance defined above
   // - Colour similar enough 
   if (p->blobId>=0&&\
       fabs(q->cx[0]-p->cx[0])<DIST_TOL&&fabs(q->cy[0]-p->cy[0])<DIST_TOL&&\
       fabs(p->size-q->size)<SIZE_TOL&&\
       fabs(p->R-q->R)<COL_TOL&&fabs(p->G-q->G)<COL_TOL&&fabs(p->B-q->B)<COL_TOL)	
   {
    // Found a matching blob. Update its position and history. Update colour (to adjust to slow changes)
    found=1;
    if (p->tracked_status==0) {p->tracked_status=1; p->age=0;}		// Now we're tracking this blob

    // Shift velocity and location history one down
    for (i=3;i>=0;i--) {p->cx[i+1]=p->cx[i]; p->cy[i+1]=p->cy[i]; p->vx[i+1]=p->vx[i]; p->vy[i+1]=p->vy[i];}

    p->cx[0]=q->cx[0];			// New position
    p->cy[0]=q->cy[0];
    p->size=(p->size+q->size)/2;	// Average of old and new size
    p->R=(p->R+q->R)/2.0;		// Average of old and new colour
    p->G=(p->G+q->G)/2.0;
    p->B=(p->B+q->B)/2.0;
    p->age++;
    p->updated=1;

    // Instantaneous velocity vector in pixels/second
    p->vx[0]=(p->cx[0]-p->cx[1])/timediff;
    p->vy[0]=(p->cy[0]-p->cy[1])/timediff;

    // Smoothed velocity vector
    p->vx[0]=(.5*p->vx[0])+(.25*p->vx[1])+(.125*p->vx[2])+(.0625*p->vx[3])+(.0625*p->vx[4]);
    p->vy[0]=(.5*p->vy[0])+(.25*p->vy[1])+(.125*p->vy[2])+(.0625*p->vy[3])+(.0625*p->vy[4]);

    // If the current motion vector is meaningful (x or y component more than 1 pixel/frame) update
    // blob heading as a unit vector.
    if ((p->vx[0]>noiseV || p->vx[0]<-noiseV)&&(p->vy[0]>noiseV||p->vy[0]<-noiseV))
    {
     len=1.0/sqrt((p->vx[0]*p->vx[0])+(p->vy[0]*p->vy[0]));
     p->mx=p->vx[0]*len;
     p->my=p->vy[0]*len;
    }

    // Update the bounding box
    p->x1=q->x1;
    p->y1=q->y1;
    p->x2=q->x2;
    p->y2=q->y2;

    p->label=q->label;		// Blob label in the latest label image
   }
   p=p->next;
  }

  if (!found)			// This is a new blob
  {
   // Create a new node for this blob and add to the tracked list
   p=*(tracked);
   r=(struct blob *)calloc(1, sizeof(struct blob));
   memcpy(r,q,sizeof(struct blob));
   r->blobId=blob_num++;
   r->tracked_status=0;
   r->age=0;
   r->updated=1;
   r->p=.1;					// Small but enough that it can grow... mind this <------- !!!
   r->idtype=0;					// And no type ID

   if (p!=NULL)
   {
    r->next=p->next;
    p->next=r;
   }
   else {r->next=NULL; *(tracked)=r;}
  }
  q=q->next;
 }

 // Check for un-tracked blobs for removal
 // Removal means marking the blobId as -1 (saves re-linking)
 p=*(tracked);
 while (p!=NULL)
 {
  if (p->updated==0)
  {
   if (p->tracked_status==1) {p->tracked_status=0; p->age=0;}
   p->age++;					// Age increased
   p->p*=P_DECAY;				// p for un-tracked blobs decays each frame.
   if (p->age>BLOB_RECALL) p->blobId=-1;	// Too old - delete!
  }
  p=p->next;
 }

 // Remove deleted blobs
 p=*(tracked);
 while (p->blobId==-1)		// Remove deleted blobs at head
 {
  *(tracked)=(*(tracked))->next;
  free(p);
  p=*(tracked);
  if (p==NULL) break;
 }
 if (*(tracked)==NULL) return;	// List is empty. No blobs!

 // Re-link the remaining list and free deleted blobs
 p=*(tracked);
 q=p->next;
 p->next=NULL;
 while (q!=NULL)
 {
  r=q->next;
  if (q->blobId>=0) {q->next=p->next; p->next=q;}
  else free(q);
  q=r; 
 }

 i=0;
 j=0;
 k=0;
 p=*(tracked);
 while (p!=NULL) 
 {
  if (p->blobId>=0)
  {
   i++;
   if (p->tracked_status==1) j++;
  }
  else k++;
  p=p->next;
 }
// fprintf(stderr,"Current blob list has %d entries, %d tracked, %d deleted\n",i+k,j,k);
}

struct image *renderBlobs(unsigned char *fgIm, int sx, int sy, struct image *labels, struct blob *list)
{
 // Render the blobs as uniform-coloured regions with bounding boxes
 // and centroids marked

 int i,j;
 struct blob *p;
 struct image *blobIm;
 double cR,cG,cB;
 double *labRGB;
 int maxLab,lab;

 if (list==NULL) return(NULL);

 // Label map is unstable, so there may be higher labels in the map, or in the existing tracked blobs
 maxLab=-1;
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
   if (*(labels->layers[0]+i+(j*labels->sx))>maxLab) maxLab=*(labels->layers[0]+i+(j*labels->sx));
 p=list;
 while (p!=NULL) {if (p->label>maxLab) maxLab=p->label; p=p->next;}

 labRGB=(double *)calloc(3*(maxLab+1),sizeof(double));
 if (labRGB==NULL) return(NULL);
 p=list;
 while (p!=NULL)
 {
  if (p->tracked_status==1||*(labRGB+((p->label)*3)+0)==0)
  {
   *(labRGB+((p->label)*3)+0)=p->R;
   *(labRGB+((p->label)*3)+1)=p->G;
   *(labRGB+((p->label)*3)+2)=p->B;
  }
  p=p->next;
 }

 blobIm=imageFromBuffer(fgIm,sx,sy,3);
 // Uniform coloured blobs - colour is average colour of the corresponding region in the image  
#pragma omp parallel for schedule(dynamic,32) provate(i,j)
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   if (*(labels->layers[0]+i+(j*labels->sx))!=0)
   {
    lab=*(labels->layers[0]+i+(j*labels->sx));
    *(blobIm->layers[0]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+0);
    *(blobIm->layers[1]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+1);
    *(blobIm->layers[2]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+2);
   }
  }

 // Draw bounding boxes & cross-hairs 
 p=list;
 while (p!=NULL)
 {
  if (p->blobId>=0&&p->idtype>0)
  {
   if (p->idtype==3)
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx[0]-1,p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0]-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0]+1,p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0]+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=255.0;
    cG=0;
    cB=0;
   }
   else if (p->idtype==1)
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx[0]-1,p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0]-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0]+1,p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0]+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=32.0;
    cG=255.0;
    cB=32.0;
   }
   else if (p->idtype==2)
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx[0]-1,p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0]-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0]+1,p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0]+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx[0],p->cy[0],p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=64.0;
    cG=64.0;
    cB=255.0;
   }
   drawBox(p->x1-1,p->y1,p->x2-1,p->y2,cR,cG,cB,blobIm);
   drawBox(p->x1+1,p->y1,p->x2+1,p->y2,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1-1,p->x2,p->y2-1,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1+1,p->x2,p->y2+1,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1,p->x2,p->y2,cR,cG,cB,blobIm);

   drawCross(p->cx[0]-1,p->cy[0],cR,cG,cB,15,blobIm);
   drawCross(p->cx[0]+1,p->cy[0],cR,cG,cB,15,blobIm);
   drawCross(p->cx[0],p->cy[0]-1,cR,cG,cB,15,blobIm);
   drawCross(p->cx[0],p->cy[0]+1,cR,cG,cB,15,blobIm);
   drawCross(p->cx[0],p->cy[0],cR,cG,cB,15,blobIm);
  }
  p=p->next;
 }

 free(labRGB);
 return(blobIm);
}

void drawLine(int x1, int y1, double vx, double vy, double scale, double R, double G, double B, struct image *dst)
{
 // Draw a line from (x1,y1) to (x1+(vx*scale),y1+(vy*scale)) in the specified colour
 int i,j;
 double len;
 double l;

 len=sqrt((vx*vx)+(vy*vy));
 vx=vx/len;
 vy=vy/len;

 for (l=0;l<=len*scale;l++)
 {
  i=(int)(x1+(l*vx));
  j=(int)(y1+(l*vy));
  if (i>=0&&i<dst->sx&&j>=0&&j<dst->sy)
  {
   *(dst->layers[0]+i+(j*dst->sx))=R;
   *(dst->layers[1]+i+(j*dst->sx))=G;
   *(dst->layers[2]+i+(j*dst->sx))=B;
  }
 }

}
 
void drawBox(int x1, int y1, int x2, int y2, double R, double G, double B, struct image *dst)
{
 // Draw a box of the specified colour between (x1,y1) and (x2,y2). dst is assumed to be an RGB image
 int i,j;

 for (i=x1;i<=x2;i++)
 {
  if (i>=0&&i<dst->sx)
  {
   if (y1>=0&&y1<dst->sy)
   {
    *(dst->layers[0]+i+(y1*dst->sx))=R;
    *(dst->layers[1]+i+(y1*dst->sx))=G;
    *(dst->layers[2]+i+(y1*dst->sx))=B;
   }
   if (y2>=0&&y2<dst->sy)
   {
    *(dst->layers[0]+i+(y2*dst->sx))=R;
    *(dst->layers[1]+i+(y2*dst->sx))=G;
    *(dst->layers[2]+i+(y2*dst->sx))=B;
   }
  } 
 }
 for (j=y1;j<=y2;j++)
 {
  if (j>=0&&j<dst->sy)
  {
   if (x1>=0&&x1<dst->sx)
   {
    *(dst->layers[0]+x1+(j*dst->sx))=R;
    *(dst->layers[1]+x1+(j*dst->sx))=G;
    *(dst->layers[2]+x1+(j*dst->sx))=B;
   }
   if (x2>=0&&x2<dst->sx)
   {
    *(dst->layers[0]+x2+(j*dst->sx))=R;
    *(dst->layers[1]+x2+(j*dst->sx))=G;
    *(dst->layers[2]+x2+(j*dst->sx))=B;
   }
  }
 }

}

void drawCross(int mcx, int mcy, double R, double G, double B, int len, struct image *dst)
{
 int i,j;

 for (j=-len; j<len; j++)
  if (mcy+j>=0&&mcy+j<sy&&j!=0) 
  {
   *(dst->layers[0]+mcx+((mcy+j)*dst->sx))=R;
   *(dst->layers[1]+mcx+((mcy+j)*dst->sx))=G;
   *(dst->layers[2]+mcx+((mcy+j)*dst->sx))=B;
  }
 for (i=-len; i<len; i++)
  if (mcx+i>=0&&mcx+i<sx&&i!=0)
  {
   *(dst->layers[0]+mcx+i+(mcy*dst->sx))=R;
   *(dst->layers[1]+mcx+i+(mcy*dst->sx))=G;
   *(dst->layers[2]+mcx+i+(mcy*dst->sx))=B;
  }
}


/*********************************************************************
End of frame processing functions
**********************************************************************/

/*********************************************************************
GLUT display loop and main() initialization
*********************************************************************/

void FrameGrabLoop(void)
{
 /*
   Main loop - Grab and display a frame
               TO DO: Process frame to extract field
		      Do the AI and planning
		      Send commands to bots
 */

  // OpenGL variables. Do not remove
  static int frame=0;
  char line[1024];
  int ox,oy,i,j;
  double *tmpH;
  unsigned char *big, *tframe;
  struct line *lineset;
  struct image *hT;
  struct image *trackMask;
  struct image *t1, *t2, *t3;
  struct image *labIm, *blobIm;
  static int nblobs=0;

  /***************************************************
   Grab a frame and display on the OpenGL window
  ***************************************************/

  big=&bigIm[0];
  im=getFrame(webcam,sx,sy);
  ox=420;		// Center webcame image at the top of the
  oy=1;				// big frame.
  t3=imageFromBuffer(im,sx,sy,3);
  
  if (toggleProc==1)
  {
   // Removed. We probably will do without the line detection and auto-quad - too much hassle
  }
  else
  {  
   if (toggleProc==2)
   {
    if (H!=NULL) {free(H);H=NULL;}

    // Draw a green cross-hair at this location
    mcx-=1;
    for (j=-15; j<15; j++)
     if (mcy+j>=0&&mcy+j<sy&&j!=0)
     {
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+0)=0;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+1)=255;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+2)=0;
     }
    for (i=-15; i<15; i++)
     if (mcx+i>=0&&mcx+i<sx&&i!=0)
     {
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+0)=0;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+1)=255;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+2)=0;
     }
    mcx+=2;
    for (j=-15; j<15; j++)
     if (mcy+j>=0&&mcy+j<sy&&j!=0)
     {
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+0)=0;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+1)=255;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+2)=0;
     }
    for (i=-15; i<15; i++)
     if (mcx+i>=0&&mcx+i<sx&&i!=0)
     {
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+0)=0;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+1)=255;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+2)=0;
     }
    mcx-=1;
    mcy-=1;
    for (j=-15; j<15; j++)
     if (mcy+j>=0&&mcy+j<sy&&j!=0)
     {
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+0)=0;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+1)=255;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+2)=0;
     }
    for (i=-15; i<15; i++)
     if (mcx+i>=0&&mcx+i<sx&&i!=0)
     {
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+0)=0;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+1)=255;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+2)=0;
     }
    mcy+=2;
    for (j=-15; j<15; j++)
     if (mcy+j>=0&&mcy+j<sy&&j!=0)
     {
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+0)=0;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+1)=255;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+2)=0;
     }
    for (i=-15; i<15; i++)
     if (mcx+i>=0&&mcx+i<sx&&i!=0)
     {
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+0)=0;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+1)=255;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+2)=0;
     }
    mcy-=1;
    for (j=-15; j<15; j++)
     if (mcy+j>=0&&mcy+j<sy&&j!=0)
     {
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+0)=0;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+1)=255;
      *(im+(((int)mcx+(((int)mcy+j)*sx))*3)+2)=0;
     }
    for (i=-15; i<15; i++)
     if (mcx+i>=0&&mcx+i<sx&&i!=0)
     {
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+0)=0;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+1)=255;
      *(im+(((int)mcx+i+(((int)mcy)*sx))*3)+2)=0;
     }
   }
   if (cornerIdx==4&&H==NULL)
   {
    fprintf(stderr,"Computing homography and acquiring background image\n");
    H=getH();

    // Turn off auto exposure and auto white-balance
    v4l2SetControl(webcam, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    v4l2SetControl(webcam, V4L2_CID_AUTO_WHITE_BALANCE, 0);

    // Get background image
    t2=newImage(t3->sx,t3->sy,3);
    for (i=0;i<25;i++)
    {
     tframe=getFrame(webcam,sx,sy);
     t1=imageFromBuffer(im,sx,sy,3);
     pointwise_add(t2,t1);
     deleteImage(t1);
     free(tframe);
    }
    image_scale(t2,1.0/25.0);
    fieldUnwarp(H,t2);
    deleteImage(t2);
    for (j=0;j<1024*768*3;j++) bgIm[j]=fieldIm[j];
    gotbg=1;
    time(&time1);
    frameNo=0;
   }

   labIm=blobIm=NULL;

   if (H!=NULL) 
   {
    fieldUnwarp(H,t3);
    bgSubtract();
    // Call blob detector
    labIm=blobDetect(fieldIm,1024,768,&blobs,&nblobs);
    if (tracked_blobs!=NULL||blobs!=NULL)
    {
     blobTrack(&tracked_blobs,blobs);
     // TO DO: DO ALL YOUR AI PROCESSING HERE!
     if (doAI) skynet.runAI(&skynet,tracked_blobs,NULL);
     blobIm=renderBlobs(fieldIm,1024,768,labIm,tracked_blobs);
    }
    deleteImage(labIm);
   }

   // Copy our frame into the bigIm array
   if (H==NULL) // Still in initialization phase
   {
    double ii,jj,dx,dy;
    dx=(double)sx/1023.0;
    dy=(double)sy/767.0;
#pragma omp parallel for schedule(dynamic,32) private(ii,jj,i,j,dx,dy)
    for (j=0;j<768;j++)
     for (i=0;i<1024;i++)
     {
      ii=i*dx;
      jj=j*dy;
      *(big+((i+((j+128)*1024))*3)+0)=*(im+(((int)ii+(((int)jj)*sx))*3)+0);
      *(big+((i+((j+128)*1024))*3)+1)=*(im+(((int)ii+(((int)jj)*sx))*3)+1);
      *(big+((i+((j+128)*1024))*3)+2)=*(im+(((int)ii+(((int)jj)*sx))*3)+2);
     }
   }
   else if (blobIm==NULL)	// Initialized, no blob image - show rectified field
   {
#pragma omp parallel for schedule(dynamic,32) private(i,j)
     for (j=0;j<768;j++)
      for (i=0;i<1024;i++)
      {
       *(big+(((i)+((j+128)*1024))*3)+0)=fieldIm[((i+(j*1024))*3)+0];
       *(big+(((i)+((j+128)*1024))*3)+1)=fieldIm[((i+(j*1024))*3)+1];
       *(big+(((i)+((j+128)*1024))*3)+2)=fieldIm[((i+(j*1024))*3)+2];
      }
   }
   else		// Initialized, got blobs!
   {
#pragma omp parallel for schedule(dynamic,32) private(i,j)
     for (j=0;j<768;j++)
      for (i=0;i<1024;i++)
      {
       *(big+(((i)+((j+128)*1024))*3)+0)=(unsigned char)((*(blobIm->layers[0]+i+(j*blobIm->sx))));
       *(big+(((i)+((j+128)*1024))*3)+1)=(unsigned char)((*(blobIm->layers[1]+i+(j*blobIm->sx))));
       *(big+(((i)+((j+128)*1024))*3)+2)=(unsigned char)((*(blobIm->layers[2]+i+(j*blobIm->sx))));
      }
     deleteImage(blobIm);
   }

  }
  deleteImage(t3);

  // Clear the screen and depth buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  if (frame==0)
  {
   glGenTextures( 1, &texture);
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   glBindTexture( GL_TEXTURE_2D, texture);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, big);
  }
  else
  {
   glBindTexture(GL_TEXTURE_2D, texture);
   glTexSubImage2D(GL_TEXTURE_2D,0,0,0,1024,1024,GL_RGB,GL_UNSIGNED_BYTE,big);
  }
  // Draw box bounding the viewing area
  glBegin (GL_QUADS);
  glTexCoord2f (0.0, 0.0);
  glVertex3f (0.0, 60.0, 0.0);
  glTexCoord2f (1.0, 0.0);
  glVertex3f (800.0, 60.0, 0.0);
  glTexCoord2f (1.0, 1.0);
  glVertex3f (800.0, 740.0, 0.0);
  glTexCoord2f (0.0, 1.0);
  glVertex3f (0.0, 740.0, 0.0);
  glEnd ();

  // Make sure all OpenGL commands are executed
  glFlush();
  // Swap buffers to enable smooth animation
  glutSwapBuffers();

  // Clean Up - Do all the image processing, AI, and planning before this code
  free(im);				// Release memory used by the current frame
  frame++;

  // Tell glut window to update ls itself
  glutSetWindow(windowID);
  glutPostRedisplay();
}

int imageCaptureStartup(char *devName, int rx, int ry, int own_col, int AI_mode)
{
 /*
   The main function intializes the webcam, opens an OpenGL window for display, and
   then launches the FrameGrabLoop() 
 */

 toggleProc=0;
 memset(&bgIm[0],0,1024*768*3*sizeof(unsigned char));
 AIMode=AI_mode;
 botCol=own_col;

 fprintf(stderr,"Camera initialization!\n");
 // Initialize the webcam
 webcam=initCam(devName,rx,ry);

 // Turn on auto exposure - should set the camera to the correct exposure for the 
 // lighting conditions in the lab while the field is being calibrated. Enable
 // auto white-balance to get an initial colour calibration.
 v4l2SetControl(webcam, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO);
 v4l2SetControl(webcam, V4L2_CID_AUTO_WHITE_BALANCE, 1);

 if (webcam==NULL)
 {
  fprintf(stderr,"Unable to initialize webcam!\n");
  return -1;
 }
 sx=webcam->width;
 sy=webcam->height;
 fprintf(stderr,"Camera initialized! grabbing frames at %d x %d\n",sx,sy);

 // Done, set up OpenGL and call particle filter loop
 fprintf(stderr,"Entering main loop...\n");
 Win[0]=800;
 Win[1]=800;

 // Get initial time for FPS
 time(&time1);
 
 // Initialize the AI data structure with the requested mode
 setupAI(AIMode,botCol, &skynet);

 initGlut(devName);
 glutMainLoop();

 return 0;
}

