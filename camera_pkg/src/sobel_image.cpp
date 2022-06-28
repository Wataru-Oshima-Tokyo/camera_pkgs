#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#define rep(i,a,b) for(int i=a;i<b;i++)
using namespace cv;
using namespace std;

// See www.asciitable.com
#define ESCAPE_KEY (27)
#define SYSTEM_ERROR (-1)
#define IMG_HEIGHT (240)
#define IMG_WIDTH (320)


int main(int argc, char *argv[]){
  cout << "hello world" <<endl;
   cv::CommandLineParser parser(argc, argv,
                               "{@input   |../data/lena.jpg|input image}"
                               "{ksize   k|1|ksize (hit 'K' to increase its value)}"
                               "{scale   s|1|scale (hit 'S' to increase its value)}"
                               "{delta   d|0|delta (hit 'D' to increase its value)}"
                               "{help    h|false|show help message}");
  cout << "The sample uses Sobel or Scharr OpenCV functions for edge detection\n\n";
  parser.printMessage();
  cout << "\nPress 'ESC' to exit program.\nPress 'R' to reset values ( ksize will be -1 equal to Scharr function )";
  char winInput;
  int ksize = parser.get<int>("ksize");
  int scale = parser.get<int>("scale");
  int delta = parser.get<int>("delta");
  int ddepth =CV_16S;
  //VideoCapture cap("./Open_Source_HD_Video_1080p_MP4.mp4");
  while(1){
    Mat image = imread("./ppms/frame100.ppm"), src, gray, grad;
    imshow("frame100", image); 

    //earase the noise on the image
    GaussianBlur(image,src, Size(3,3),0,0,BORDER_DEFAULT);

    //conver tot grayscale
    cvtColor(src, gray, COLOR_BGR2GRAY);

    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    /*
    We calculate the "derivatives" in x and y directions. 
    For this, we use the function Sobel() as shown below: 
    The function takes the following arguments:
        src: In our example, the input image. Here it is CV_8U
        grad_x / grad_y : The output image.
        ddepth: The depth of the output image. We set it to CV_16S to avoid overflow.
        x_order: The order of the derivative in x direction.
        y_order: The order of the derivative in y direction.
        scale, delta and BORDER_DEFAULT: We use default values.
    Notice that to calculate the gradient in x direction we use: xorder=1 and yorder=0. 
    We do analogously for the y direction.
    */
    Sobel(gray, grad_x, ddepth,1,0, ksize,scale, delta, BORDER_DEFAULT);
    Sobel(gray, grad_y, ddepth,0,1, ksize,scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    
    /*
    We try to approximate the gradient by adding both directional gradients 
    (note that this is not an exact calculation at all! but it is good for our purposes).
    */
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    imshow("sobel", grad);
    string sobel_img = "./ppms/sobel_frame100.jpeg"; 
    imwrite(sobel_img, grad);
    // waitKey(0);
    // break;
    if ((winInput = waitKey(0)) == ESCAPE_KEY)
    //if ((winInput = waitKey(0)) == ESCAPE_KEY)
    {
        break;
    }
    else if(winInput == 'n')
    {
    cout << "input " << winInput << " ignored" << endl;
    }
  }
    destroyAllWindows();

  
  return 0;
}
