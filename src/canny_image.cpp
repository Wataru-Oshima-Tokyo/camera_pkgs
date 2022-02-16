#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
using namespace cv;
Mat src, src_gray;
Mat dst, detected_edges;
int lowThreshold = 6;
const int max_lowThreshold = 100;
const int ratio = 3;
//set the kernel size 3
const int kernel_size = 3;
const char* window_name = "Edge Map";
static void CannyThreshold(int, void*)
{
    // removeing the noise and ap since the kernel size is 3, set the size 3 by 3
    blur( src_gray, detected_edges, Size(3,3) );
    //detecing an edege by Canny with the lowThreashold and maxThreshold which is 3 times thant the lower one.
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    //set the dist image all black so that you can put the deteceted edges on the black background
    dst = Scalar::all(0);
    // src.copyTo( dst, detected_edges);
    imshow( window_name, detected_edges );
}
int main( int argc, char** argv )
{
//   CommandLineParser parser( argc, argv, "{@input | ../data/fruits.jpg | input image}" );
//   src = imread( parser.get<String>( "@input" ), IMREAD_COLOR ); // Load an image
    Mat src = imread("./ppms/frame200.ppm");
    imshow("frame100", src);
  if( src.empty() )
  {
    std::cout << "Could not open or find the image!\n" << std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return -1;
  }
  dst.create( src.size(), src.type() );
  cvtColor( src, src_gray, COLOR_BGR2GRAY );
  namedWindow( window_name, WINDOW_AUTOSIZE );
  /* you can set the value of Threshold for the canny function. 
    It can be used for ajusting how much we want to detect the edge
  */
//   createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
  //call canny Threashold
  CannyThreshold(0, 0);
  std::string canny_img = "./ppms/canny_frame200.jpeg"; 
  imwrite(canny_img, detected_edges);
  waitKey(0);
  return 0;
}