 /*
 * c 2017 BBarriage
 * c 2017 Fordham University
 *  Author: Ben Barriage
 *  Date: 1/15/16
 *  Purpose: Flip, Concatenate, and Change Polar Coordinate Images in order to generate HSVD graphics
*/
#include <cv.h>
#include <highgui.h>
#include <stdlib.h>
#include <vector>
#include <math.h>

#define IMG_WIDTH 1024
#define IMG_HEIGHT 768

IplImage *image1, *image2, *image3, *image4;

IplImage *concatenateImages (IplImage * panorama, std::vector < IplImage * >images);

IplImage *decagonImage(IplImage* panorama, std::vector<IplImage*>images);

IplImage * trapezoidImage(IplImage* image);

IplImage* emptyImage(IplImage * image);

IplImage * copyImage(IplImage* image1,IplImage* image2);

void showData(IplImage* image);

int main (int argc, char **argv){

  std::vector < IplImage * >pano;
  IplImage *panorama, *trapezoid;
  
  char *fileName = NULL;
  
  
  for (int i = 1; i < argc; i++)
    {
      char *fileName = argv[i];
      image1 = cvLoadImage (fileName);
      image4 = cvLoadImage(fileName);
      printf("PERFORMING WARP\n");
      image1 = trapezoidImage(image1);
      image4 = trapezoidImage(image4);
      cvSaveImage("image1.pgm",image1);
      image2 = emptyImage(image1);
      image3 = copyImage(image4, image2);
      cvSaveImage("image3.pgm", image3);
      pano.push_back (image3);
      printf ("PUSHED\n");
    }
  
  
  //panorama =cvCreateImage (cvSize ( IMG_HEIGHT*pano.size(),IMG_WIDTH*pano.size()), IPL_DEPTH_8U, 3);
  panorama =cvCreateImage (cvSize ( 4000,4000), IPL_DEPTH_8U, 3);
  printf("Pano height is: %d\n", panorama->height);
  printf("Pano width is: %d\n", panorama->width);
  //  panorama = concatenateImages (panorama, pano);
  panorama = decagonImage(panorama, pano);
  cvSaveImage ("trapezoid.pgm", panorama);
  
  
  //  test = panorama;
  //cvSaveImage ("example.pgm", test);
  printf ("PANORAMA SAVED!\n");
}


/*
  @Purpose: Concatenate images from a vector of images into one single image
  @param panorama: the IplImage which will contain the concatenated vector of images
  @param images: the vector of images which will be concatenated into the single panoramic image
  @return: an IplImage which has been created from a vector of images into a single panorama 
 */
IplImage * concatenateImages (IplImage * panorama, std::vector < IplImage * >images){
  std::vector < IplImage * >::reverse_iterator imageRev; 
  int count = 0;
  CvScalar p;
  
  for (imageRev = images.rbegin (); imageRev != images.rend (); imageRev++){
    IplImage *img = *imageRev;
    for (int u = 1; u < img->width; u++)
      for (int v = 1; v < img->height; v++)
	{
	  p = cvGet2D (img, v, u);
	  cvSet2D (panorama, v, u + (count * IMG_WIDTH), p);
	}
    count++;
  }
  return panorama;
}

/*
  @Purpose: remove values from the image in order to leave only data necessary for the trapezoid representation
  @param image: the image which will have its data removed in order to obtain the trapezoid format
  @return: an IplImage with the original data, except it will have unneccessary data removed 
 */
IplImage * trapezoidImage(IplImage* image){
  //conversion of image attributes into more easily read attributes
  int height = image->height;
  int width = image->width;
  int nChannels = image->nChannels;
  int step = image->widthStep;
  uchar * data  = (uchar*) image->imageData;
  
  int reduce = 0; //used to determine values to be removed as the image is being traversed 
  
  for(int i =0;i<=height;i++){ //cycling through each row
    for(int j =0;j<=width;j++){ //cycling through each column
      if(reduce>0){ //ensures top line contains all data
	int remove = (width/reduce);
	for(int k=remove;k<=width;k+=remove){
	  data[i*step+k*nChannels+0]=0; //assigns red value of data point to 0
	  data[i*step+k*nChannels+1]=0; //assigns green value of data point to 0 
	  data[i*step+k*nChannels+2]=0; //assigns blue value of data point to 0
	}
      }
    }
    
    if(i%2==0) 
      reduce++;
  }

  return image;
}

/*
  @Purpose: Used during testing to find R,G,B values of each data point 
  @param image: the image which will have its data shown
 */
void showData(IplImage* image){
  //making image attributes more easily readable
  int height = image->height;
  int width = image->width;
  int nChannels = image->nChannels;
  int step = image->widthStep;
  int r,g,b;
  
  uchar * data  = (uchar*) image->imageData;
  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      r = data[i*step+j*nChannels+0];
      g = data[i*step+j*nChannels+1];
      b = data[i*step +j*nChannels+2];
    }
    printf("R is: %d \n", r);
    printf("G is: %d \n", g);
    printf("B is: %d \n", b);
  }
}

/*
  @purpose: copy non-zero image data from one image onto a blank image
  @param image1: contains necessary data to be coped over
  @param image2: the image which will have necessary data copied into it
  @preconditions: image1 has had unneccessary data removed, image2 is a blank canvas 
  @postcondition: image2 has had necessary data from image1 copied to it
  @return: image2 with the necessary data of image1 copied
 */
IplImage* copyImage(IplImage* image1, IplImage* image2){
  printf("Starting Data Copy\n");
  //assigning attribute values from image1 in order to make code easier to read/interpret
  int height = image1->height;
  int width = image1->width;
  int nChannels = image1->nChannels;
  int step = image1->widthStep;
  uchar * data  = (uchar*) image1->imageData;

  //assigning attribute values from image2 in order to make code easier to read/interpret
  uchar * newData = (uchar *) image2->imageData;
  int newChannels = image2->nChannels;
  int newStep = image2->widthStep;

  //Doing left side
  for(int i=1;i<height;i++){
    int k=width/2;  
    for(int j=width/2;j>0;j--) //ensures we do the left half only 
      if(data[i*step+j*nChannels+0]!=0 || data[i*step+j*nChannels+1]!=0 || data[i*step+j*nChannels+2]!=0){ //if any red, blue, or green value is not zero, we copy the data
	newData[i*step+(k)*nChannels+0]=data[i*step+j*nChannels+0];
	newData[i*step+(k)*nChannels+1]=data[i*step+j*nChannels+1];
	newData[i*step+(k)*nChannels+2]=data[i*step+j*nChannels+2];
	k--;
      }
  }

  //Doing right side
  for(int i=1;i<height;i++){
    int k=width/2+1;
    for(int j=width/2+1;j<=width;j++) //ensures we do the right half only 
      if(data[i*step+j*nChannels+0]!=0 || data[i*step+j*nChannels+1]!=0 || data[i*step+j*nChannels+2]!=0){//if any red, blue, or green value of a data point is not zero, we copy the data 
	newData[i*step+(k)*nChannels+0]=data[i*step+j*nChannels+0];
	newData[i*step+(k)*nChannels+1]=data[i*step+j*nChannels+1];
	newData[i*step+(k)*nChannels+2]=data[i*step+j*nChannels+2];
	k++;
      }
  }
  //Cropping extra data points
  int reduce = 0;
  for(int i =0;i<=height;i++){
    for(int j =0;j<=width;j++){
      if(j<=reduce||j>=width-reduce){ //reduces data points equally from left and right sides to remove "Staircase" like effect and create a smoother image 
	  newData[i*step+j*nChannels+0]=0;
	  newData[i*step+j*nChannels+1]=0;
	  newData[i*step+j*nChannels+2]=0;
      }
    }
    
    if(i%3==0)
      reduce++;
  }
  
  
  return image2;
}

/*
  @purpose: create a blank canvas of an image for data copying
  @param image: the image which will have all of its data set to 0 
  @return: an image containing 0 for all r,g,b data point values 
 */
IplImage* emptyImage(IplImage * image){
  //assigning attributes of the image to more easily readable names 
  int height = image->height;
  int width = image->width;
  int nChannels = image->nChannels;
  int step = image->widthStep;
  uchar * data  = (uchar*) image->imageData;

  //traversing entire image and changing r,g,b values to 0 
  for(int i =0;i<height;i++)
    for(int j =0;j<width;j++){
      data[i*step+j*nChannels+0]=0; //red value 
      data[i*step+j*nChannels+1]=0; //green value
      data[i*step+j*nChannels+2]=0; //blue value 
    }
  
  return image;
  
}

IplImage *decagonImage(IplImage* panorama, std::vector<IplImage*>images){
  printf("The width of the decagon would be %d \n",panorama->width);
  printf("The height of the decagon would be %d \n", panorama->height);

  std::vector <IplImage *>::reverse_iterator imageRev;
  int count = 0;
  //CvScalar p;
  
  uchar *newData = (uchar*)panorama->imageData;
  int xOffset = panorama->width/2;
  int yOffset = panorama->height/2;
  int goalStep = panorama->widthStep;
  int goalChannels = panorama->nChannels;
  
  //  for (imageRev = images.rbegin (); imageRev != images.rend (); imageRev++){
  for(int i =0;i<images.size();i++){
    IplImage *img = images.at(1);
    //IplImage *img = *imageRev;
    int height = img->height;
    int width = img->width;
    int nChannels = img->nChannels;
    int step = img->widthStep;
    uchar * data  = (uchar*) img->imageData;
    
    
    printf("Count is %d\n",count);
    for(int i=0;i<height;i++){  
      for(int j=0;j<width;j++)
	if(data[i*step+j*nChannels+0]!=0 || data[i*step+j*nChannels+1]!=0 || data[i*step+j*nChannels+2]!=0){
	  double theta = 36*count;
	  double PI = 3.14159265;
	  int x = (j*cos(theta*PI/180.0))-((i+100)*sin(theta*PI/180.0));
	  int y = ((i+100)*sin(theta*PI/180.0))+(j*cos(theta*PI/180.0));
	  x = x+xOffset;
	  y = y+yOffset;
	  newData[y*goalStep+x*goalChannels+0]=data[i*step+j*nChannels+0];
	  newData[y*goalStep+x*goalChannels+1]=data[i*step+j*nChannels+1];
	  newData[y*goalStep+x*goalChannels+2]=data[i*step+j*nChannels+2];
	}
    }
    count++;
  }
    return panorama;
  
}
