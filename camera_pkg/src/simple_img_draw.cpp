#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;

// See www.asciitable.com
#define ESCAPE_KEY (27)
#define SYSTEM_ERROR (-1)
#define IMG_HEIGHT (240)
#define IMG_WIDTH (320)


void img_modified(Mat &frame){
    // cvtColor(frame, frame,COLOR_BGR2RGB);
    for(int i=1; i<((IMG_HEIGHT)-1); i++)
    {

        // Skip first and last column, no neighbors to convolve with
        for(int j=1; j<((IMG_WIDTH)-1); j++)
        {
            Vec3b &color = frame.at<Vec3b>(Point(j,i));
            if(i<4 || i>IMG_HEIGHT-4 || j<4 || j>IMG_WIDTH-4 || i == IMG_HEIGHT/2 || j==IMG_WIDTH/2){
                //BGR
                color.val[0] = 0;
                color.val[1] = 255;
                color.val[2] = 255;
             }
        }
    }

}


int main()
{
   VideoCapture cam0(0);
   namedWindow("320x240");
   char winInput;

   if (!cam0.isOpened())
   {
       exit(SYSTEM_ERROR);
   }

   cam0.set(CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
   cam0.set(CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);

   while (1)
   {
      // Set rows and columns 
      // lets downsize the image using new width and height

      Mat frame;
      cam0.read(frame);
      img_modified(frame);
      // resize down
      imshow("320x240", frame);

      if ((winInput = waitKey(10)) == ESCAPE_KEY)
      //if ((winInput = waitKey(0)) == ESCAPE_KEY)
      {
          break;
      }
      else if(winInput == 'n')
      {
	  cout << "input " << winInput << " ignored" << endl;
      }
      
   }

   destroyWindow("320x240"); 
};
