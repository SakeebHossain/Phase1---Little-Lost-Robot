/*

  Code Optimization exercise.

  The goal of this exercise is to get you to think about
  code optimization.

  The setup:

   One of the most common tasks in image processing is that of 
  taking an image or video at low resolution and rescaling it.
  You see this done all the time when you change the size of your
  video window, or choose to stretch a small video to full screen.

   As you know by now, modern CPUs do this in hardware. However, not
  all devices have such hardware support. Here you will investigate
  how good coding can substantially increase the performance of
  software that does simple image processing.

   This exercise will help you see what kinds of issues arise in
  optimizing code and how much of a performance improvement you can
  expect from careful coding.

  The task:

   The code I am providing you does the following:

   - Load a test image from a .ppm file (my favorite format!)
   - Does image rescaling using bilinear interpolation (more on this below)
     and counts the number of times the rescaling routine can be called
     per second (in other words, it estimates FPS for the image rescaling
     algorithm)
   - Writes the rescaled image to disk for inspection.
   - We don't have a movie, so the code simply calls the re-scaling routine
     multiple times on the same test image. This also allows us to measure
     the speed of the rescaling routine without worrying about any delays
     introduced by movie decoding and data transfer.

   The function vanilla_rescaleImage() is written without much thought for
   optimization and not using any knowledge of how modern CPUs work.

   You are to provide a new function called:

   fast_imageRescale()

   That does the rescaling faster than vanilla_rescaleImage() by paying
   close attention to code optimization.

   I want you to do a bit of research into code optimization in C, and then
   think very carefully about how to write your function.

   You will be graded on how fast your routine is compared to the plain
   vanilla version I'm providing!

   * Special * Special * Special *
   Earn NEGATIVE marks for writing a fast_imageRescale() routine that is
   slower than the vanilla version provided here!

   If you have any questions about how the scaling algorithm works, ask
   me or ask your TA during tutorial.

   Final notes:

   YOU MUST COMPILE THIS CODE USING THE ATTACHED compile.sh
   SCRIPT. Do not alter this script, as changing compilation options will
   cause your code to become impossible to compare to that of your
   colleagues.

   DO NOT MODIFY ANY EXISTING CODE. You are only allowed to:
   - Add code to fast_imageRescale()
   - Add any helper functions you need
   - You CAN use getPixel() and setPixel() if you wish

   COMMENT YOUR CODE properly. In particular, for each part of your
   fast rescaling function that is different from the plain vanilla
   version, explain WHY you expect it to be faster.

   Now go forth and Optimize!

  Code by F. Estrada for CSC C85
  V1.0.0 Completed, Sep. 17, 2011

  Image credits:
  test and reference 1: Michael Maggs, licensed under Creative Commons.
  test and reference 2: Jebulon, licensed under Creative Commons.

*/

// Image size definition for 1080p
#define HD_Xres 1920
#define HD_Yres 1080

/*
  Standard C libraries
*/
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>
#include<time.h>

/*
  Function prototypes
*/

unsigned char *fast_rescaleImage(unsigned char *src, int src_x, int src_y, int dest_x, int dest_y);
unsigned char *vanilla_rescaleImage(unsigned char *src, int src_x, int src_y, int dest_x, int dest_y);
void getPixel(unsigned char *image, int x, int y, int sx, unsigned char *R, unsigned char *G, unsigned char *B);
void setPixel(unsigned char *image, int x, int y, int sx, unsigned char R, unsigned char G, unsigned char B);
int main(int argc, char *argv[]);
unsigned char *readPPMimage(const char *filename, int *sx, int *sy);
void imageOutput(unsigned char *im, int sx, int sy, const char *name);

/*****************************************************
** TO DO:
**
**  Write a fast_rescaleImage() function.
**
**  The vanilla_imageRescale() function explains in
**  detail how the rescaling algorithm works, it is
**  your job to determine how to do this as fast as
**  possible!
**
**  You can add helper functions where needed.
**
**  Document your code carefully and comment at each
**  step that differs from the vanilla version WHY
**  your version should be faster.
*****************************************************/

unsigned char *fast_rescaleImage(unsigned char *src, int src_x, int src_y, int dest_x, int dest_y)
{
 return(NULL);		// Comment out, and write your fast routine here!
}

/*****************************************************
** DO NOT MODIFY ANY CODE BELOW THIS LINE.
**
** However, make sure you read and understand the
** description of how image rescaling works in the
** vanilla_rescaleImage() function.
**
** You may or may not want to read the actual code
** in vanilla_imageResize(). On the one hand,
** reading it can give you ideas. On the other,
** reading it can give you the WRONG ideas.
*****************************************************/
unsigned char *vanilla_rescaleImage(unsigned char *src, int src_x, int src_y, int dest_x, int dest_y)
{
 /*
   Image rescaling routine.

   First, let's look at how the image is stored in memory.

   The image is a chunk of memory of size (sx * sy * 3) bytes. (sx,sy) is the image resolution,
   and each pixel in the image is specified using 3 bytes for the R, G, and B colour components.
   
   Pixels are stored in raster order:
   [(0,0) (0,1) (0,2) ... (0, sx-1) (1,0) (1,1) ... (1, sx-1) ... ... ... (sy-1, sx-1)]
   And for each pixel the RGB values are stored consecutively, so the first 3 bytes are
   the RGB values for the pixel at (0,0), the next three bytes are the RGB values for the
   pixel at (0,1), and so on.

   getPixel() and setPixel() can be used to access image pixel.
   getPixel(x,y, &R, &G, &B);  // Stores the RGB values for the pixel at x,y in local variables R, G, and B
   setPixel(x,y,R,G,B);        // Sets the colour of the pixel at x,y to the specified RGB values

   Image rescaling algorithm:
   
   The general problem is that the destination image will have a different number of pixels. In
   our case, the 1920x1080 HD image is likely to be larger than the test image.

   We will use interpolation to determine the colours of pixels in the destination image. In 
   particular, we will use the extensively used 'bilinear interpolation' process used in all
   forms of graphics applications (e.g. OpenGL and DirectX have routines to do bilinear interpolation
   to smooth out graphical objects).

   For a general overview of interpolation see:     
   http://www.cambridgeincolour.com/tutorials/image-interpolation.htm    

   Here is the algorithm in a nutshell:

   First, determine what is the ratio of pixels in the source to pixels in the destination along
   both x and y.
  
   So, if the size of the source is (src_x, src_y) and the size of the destination image is
   (dst_x, dst_y):

   step_x=src_x/dst_x;
   step_y=src_y/dst_y;

   Example, if the source is (10 x 10) and the destination is (25 x 25) then
   step_x=.4
   step_y=.4;
   And this means that there are .4 pixels in the source image for each pixel in the destination
   in both directions (clearly not enough!)

   To fill the destination image in the above case, we would do the following:
   dst(0,0) <- src(0,0)
   dst(0,1) <- src(0,.4)
   dst(0,2) <- src(0,.8)
   .
   .
   .
   dst(1,0) <- src(.4,0)
   dst(1,1) <- src(.4,.4)
   .
   .
   and so on...
   
   In general, for a given step_x and step_y
   dst(i,j) <- src(i*step_x,j*step_y)

   Easy right?

   The problem is, there is no pixel at src(.4,.4) or any other fractional value. Here's where the
   interpolation happens. 

   For any fractional source coordinate, we can interpolate between the four neighbours to find the
   colour at that fractional location. For example:
   
   for src(.4,.4):
                          
                                 N1               T1                       N2
                               src(0,0) -----------*----------------------src(1,0)
                                                   |
                                                   |
                                                   |
                                                   |
                                                   |
                                                src(.4,.4)
                                                   |
                                                   |
                                                   |
                                                   |
                                                   |
                              src(0,1)-------------*----------------------src(1,1)
                                 N3                T2                       N4

   To find the colour at src(4,4), we first determine who the 4 neighbours of src(4,4) are [shown above],
   we then interpolate between N1 and N2 to find T1, interpolate between N3 and N4 
   to find T2, and finally interpolate between T1 and T2 to find the final colour at src(.4,.4).
   
   Since we interpolate linearly along both directions, this is called bi-linear interpolation.

   In general, to obtain the colour of the pixel at 

   src(fx,fy)  - source image at some fractional coordinates (fx, fy)

   N1=src(floor(fx),floor(fy));
   N2=src(ceil(fx),floor(fy));
   N3=src(floor(fx),ceil(fy));
   N4=src(ceil(fx),ceil(fy));

   dx=fx-floor(fx);
   dy=fy-floor(fy);

   T1=(dx*N2) + ((1-dx)*N1)
   T2=(dx*N4) + ((1-dx)*N3);
   src(fx,fy) = (dy*T2) + ((1-dy)*T1);

   Note that you have to do this for each of R, G, and B!

   For a full HD image, we will need to do the above about 2 million times... can you see why optimizing
   the code will be important?

   Vanilla code below... you need to look at it and run the profiler to see what lines in the code 
   below are potential bottlenecks. Think about this code carefully and use your knowledge of CPU
   architecture to decide how to optimize it.
 */

 double step_x,step_y;			// Step increase as per instructions above
 unsigned char R1,R2,R3,R4;		// Colours at the four neighbours
 unsigned char G1,G2,G3,G4;
 unsigned char B1,B2,B3,B4;
 double RT1, GT1, BT1;			// Interpolated colours at T1 and T2
 double RT2, GT2, BT2;
 unsigned char R,G,B;			// Final colour at a destination pixel
 unsigned char *dst;			// Destination image - must be allocated here! 
 int x,y;				// Coordinates on destination image
 double fx,fy;				// Corresponding coordinates on source image
 double dx,dy;				// Fractional component of source image coordinates

 dst=(unsigned char *)calloc(dest_x*dest_y*3,sizeof(unsigned char));   // Allocate and clear destination image
 if (!dst) return(NULL);					       // Unable to allocate image

 step_x=(double)(src_x-1)/(double)(dest_x-1);
 step_y=(double)(src_y-1)/(double)(dest_y-1);

 for (x=0;x<dest_x;x++)			// Loop over destination image
  for (y=0;y<dest_y;y++)
  {
   fx=x*step_x;
   fy=y*step_y;
   dx=fx-(int)fx;
   dy=fy-(int)fy;   
   getPixel(src,floor(fx),floor(fy),src_x,&R1,&G1,&B1);	// get N1 colours
   getPixel(src,ceil(fx),floor(fy),src_x,&R2,&G2,&B2);	// get N2 colours
   getPixel(src,floor(fx),ceil(fy),src_x,&R3,&G3,&B3);	// get N3 colours
   getPixel(src,ceil(fx),ceil(fy),src_x,&R4,&G4,&B4);	// get N4 colours
   // Interpolate to get T1 and T2 colours
   RT1=(dx*R2)+(1-dx)*R1;
   GT1=(dx*G2)+(1-dx)*G1;
   BT1=(dx*B2)+(1-dx)*B1;
   RT2=(dx*R4)+(1-dx)*R3;
   GT2=(dx*G4)+(1-dx)*G3;
   BT2=(dx*B4)+(1-dx)*B3;
   // Obtain final colour by interpolating between T1 and T2
   R=(unsigned char)((dy*RT2)+((1-dy)*RT1));
   G=(unsigned char)((dy*GT2)+((1-dy)*GT1));
   B=(unsigned char)((dy*BT2)+((1-dy)*BT1));
   // Store the final colour
   setPixel(dst,x,y,dest_x,R,G,B);
  }
 return(dst);
}

void getPixel(unsigned char *image, int x, int y, int sx, unsigned char *R, unsigned char *G, unsigned char *B)
{
 // Get the colour at pixel x,y in the image and return it using the provided RGB pointers
 // Requires the image size along the x direction!
 *(R)=*(image+((x+(y*sx))*3)+0);
 *(G)=*(image+((x+(y*sx))*3)+1);
 *(B)=*(image+((x+(y*sx))*3)+2);
}

void setPixel(unsigned char *image, int x, int y, int sx, unsigned char R, unsigned char G, unsigned char B)
{
 // Set the colour of the pixel at x,y in the image to the specified R,G,B
 // Requires the image size along the x direction!
 *(image+((x+(y*sx))*3)+0)=R;
 *(image+((x+(y*sx))*3)+1)=G;
 *(image+((x+(y*sx))*3)+2)=B;
}

/*****************************************************
** YOU DO NOT NEED TO LOOK AT ANYTHING BELOW THIS POINT
*****************************************************/
int main(int argc, char *argv[])
{
 /*
    Main routine:
    - Load the test image specified in the command line
    - Run both the vanilla and fast image scaling routines for a few seconds
    - Compute FPS for both
    - Save output images to disk
    - Print out FPS ratio of fast routine to vanilla routine (should be > 1!)
 */

 unsigned char *src;		// Used to store the source image
 unsigned char *dst;		// Will be used to hold the rescaled image
 int sx, sy;			// Resolution of the source image (sx * sy pixels)
 time_t t0, t1, t2, t3;
 int c_a,c_b;
 double FPS_a;
 double FPS_b;

 if (argc!=2)
 {
  fprintf(stderr,"Usage: Image_Rescale src_name\n");
  fprintf(stderr," src_name is the name of the test image (must be in .ppm format)\n");
  exit(1);
 }
 src=readPPMimage(argv[1], &sx, &sy);
 if (!src)
 {
  fprintf(stderr,"Unable to open test image\n");
  exit(1);
 }

 fprintf(stderr,"Starting tests...\n");
 // Time plain vanilla routine
 t1=t0=time(NULL);
 c_a=0;
 while(difftime(t1,t0)<15.0)
 {
  dst=vanilla_rescaleImage(src,sx,sy,HD_Xres,HD_Yres);
  if (dst) {c_a++; free(dst);} else break;
  t1=time(NULL);
 }
 if (c_a>0)
 {
  FPS_a=c_a/(double)(t1-t0);
  fprintf(stderr,"Vanilla image rescaling FPS=%f\n",FPS_a);
 }
 else
 {
  fprintf(stderr,"Something went wrong!\n");
 }

 // Time your fast routine
 t3=t2=time(NULL);
 c_b=0;
 while(difftime(t3,t2)<15.0)
 {
  dst=fast_rescaleImage(src,sx,sy,HD_Xres,HD_Yres);
  if (dst) {c_b++; free(dst);} else break;
  t3=time(NULL);
 }
 if (c_b>0)
 {
  FPS_b=c_b/(double)(t3-t2);
  fprintf(stderr,"Fast image rescaling FPS=%f\n",FPS_b);
  fprintf(stderr,"Ratio: %f\n",FPS_b/FPS_a);
 }
 else
 {
  fprintf(stderr,"Fast routine not implemented\n");
 }

 // Output rescaled images for inspection
 dst=vanilla_rescaleImage(src,sx,sy,HD_Xres,HD_Yres);
 if (dst) {imageOutput(dst,HD_Xres,HD_Yres,"vanilla_rescaled.ppm"); free(dst);}
 dst=fast_rescaleImage(src,sx,sy,HD_Xres,HD_Yres);
 if (dst) {imageOutput(dst,HD_Xres,HD_Yres,"fast_rescaled.ppm"); free(dst);}

 fprintf(stderr,"Done!\n");
 free(src);
 exit(0);
}

unsigned char *readPPMimage(const char *filename, int *sx, int *sy)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # Optionally, one or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //
 // readPPMdata converts the image colour information to floating point. This is so that
 // the texture mapping function doesn't have to do the conversion every time
 // it is asked to return the colour at a specific location.
 //
 // On error, the function returns NULL
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int sizx,sizy;

 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }

 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",&sizx,&sizy);           // Read file size
 *(sx)=sizx;
 *(sy)=sizy;

 fgets(&line[0],9,f);  	                // Read the remaining header line
 im=(unsigned char *)calloc(sizx*sizy*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }

 fread(im,sizx*sizy*3*sizeof(unsigned char),1,f);
 fclose(f);
 return(im);
}

void imageOutput(unsigned char *im, int sx, int sy, const char *name)
{
 /*
   Outputs the image stored in 'im' to a .ppm file for inspection
 */
 FILE *f;

 f=fopen(name,"w");
 if (f)
 {
  fprintf(f,"P6\n");
  fprintf(f,"# Image_Rescaling output\n");
  fprintf(f,"%d %d\n",sx,sy);
  fprintf(f,"255\n");
  fwrite(im,sx*sy*3*sizeof(unsigned char),1,f);
  fclose(f);
 }
 else
 {
  fprintf(stderr,"Unable to open file for image output!\n");
 }
}
