#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;
#define IMG_HEIGHT (240)
#define IMG_WIDTH (320)


int main(int argc, char** argv)
{
   VideoCapture cam0(0);
   namedWindow("320x240");
   char winInput;

   if (!cam0.isOpened())
   {
       exit();
   }

  cam0.set(CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
  cam0.set(CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("usb_camera/color/image", 1);
//   cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  
//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    Mat frame;
    cam0.read(frame);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    cv::waitKey(3);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

// int main()
// {
//    VideoCapture cam0(0);
//    namedWindow("320x240");
//    char winInput;

//    if (!cam0.isOpened())
//    {
//        exit(SYSTEM_ERROR);
//    }

//    cam0.set(CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
//    cam0.set(CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);

//    while (1)
//    {
//       // Set rows and columns 
//       // lets downsize the image using new width and height

//       Mat frame;
//       cam0.read(frame);
//     //   img_modified(frame);
//       // resize down
//       imshow("320x240", frame);
    
//       if ((winInput = waitKey(10)) == ESCAPE_KEY)
//       //if ((winInput = waitKey(0)) == ESCAPE_KEY)
//       {
//           break;
//       }
//       else if(winInput == 'n')
//       {
// 	  cout << "input " << winInput << " ignored" << endl;
//       }
      
//    }

//    destroyWindow("320x240"); 
// };