 #include <ros/ros.h>

 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>


 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include "std_msgs/String.h"
 #include "std_srvs/Empty.h"
 #include <vector>
 #include <camera_pkg/Coordinate.h>
 #include <map>

// #include <camera_pkg/Camera_CV.h>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define rep(i,a,b) for(int i=a;i<b;i++)
 #define fore(i,a) for(auto &i:a)
 using namespace std;
 using namespace cv;


struct timespec start, stop;
double fstart, fstop;

class CAMERA_CV{
  public:
    Mat src, src_gray, src_hsv, dst, detected_edges, mask;
    Mat depth;
    ros::Publisher pub;
    ros::Subscriber ir_sub;
    ros::NodeHandle nh;
    ros::ServiceServer imshow_start, imshow_stop;
    ros::ServiceClient calibration_start, calibration_stop;
    int lowThreshold;
    // int low_c[3] = {17, 123, 121};
    // int high_c[3] ={37, 143, 201};
    int low_c[3] = {0, 0, 0};
    int high_c[3] = {0, 0, 0};
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    void CannyThreshold(int, void*);
    void DrawCircle(int, void*);
    const std::string OPENCV_WINDOW = "Image window";
    virtual bool calibration_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool calibration_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void ir_callback(const sensor_msgs::ImageConstPtr&);
    // Topics
    const std::string IR_TOPIC = "/camera/ir/image_raw";
    const std::string PUBLISH_TOPIC = "/camera_pkg/coordinate";
    const std::string IMSHOW_SERVICE_START = "/imshow/start";
    const std::string IMSHOW_SERVICE_STOP = "/imshow/stop";
    const std::string CALIB_SERVICE_START = "/calibration/start";
    const std::string CALIB_SERVICE_STOP = "/calibration/stop";
    CAMERA_CV();
    ~CAMERA_CV();
    bool getRun(); 
    const int max_lowThreshold = 100;
    const std::string window_name = "Edge Map";

private:
    bool RUN = false;
    bool start_call = true;
    bool stop_call = false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
};


CAMERA_CV::CAMERA_CV(){};

CAMERA_CV::~CAMERA_CV(){};

bool CAMERA_CV::getRun(){
  std_srvs::Empty _emp;
  if(RUN && start_call){
    calibration_start.call(_emp);
    start_call = false;
    stop_call = true;
  }else if(!RUN && stop_call) {
    calibration_stop.call(_emp);
    start_call = true;
    stop_call = false;
  }
    
  return RUN;
}




void CAMERA_CV::DrawCircle(int, void*){
  int x = src.cols, y = src.rows;
  vector<int> x_array = {x/2-70, x/2, x/2+70, x/2+140};
  vector<int> y_array = {y/2-140, y/2-70, y/2, y/2+70};
  //draw circle 9;
  //top
  rep(i,0,x_array.size()){
    rep(j,0,y_array.size()){
      int _radius =13;
      int _saturation1 = 153;
      int _saturation2 =0;
      if(j%2==0 && i%2==0){ _radius =20;_saturation1 = 100; _saturation2 =100;} 
      else if(j%2==0) {_radius = 17; _saturation1 = 0; _saturation2 =150;}
      else if(i%2==0) {_radius = 25; _saturation1 = 200; _saturation2 =10;}
      // cv::circle(src_hsv, cv::Point(x_array[i],y_array[j]), 15, cv::Scalar(153, 0, 255), 5);
      cv::circle(src, cv::Point(x_array[i],y_array[j]), _radius, cv::Scalar(_saturation1, _saturation2, 255),5);
    }
  }

}


 bool CAMERA_CV::calibration_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  //  cout << "start calibration" << endl;
   RUN = true;
   return RUN;

 }

 bool CAMERA_CV::calibration_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  //  cout << "stop calibration" << endl;
   RUN = false;
   return RUN;
 }


void CAMERA_CV::ir_callback(const sensor_msgs::ImageConstPtr& msg){
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    depth = cv_ptr->image;

}


int main( int argc, char** argv )
{

   ros::init(argc, argv, "work_with_ir_camera_start");
   CAMERA_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(20);
   
   cc.ir_sub = cc.nh.subscribe(cc.IR_TOPIC, 1000, &CAMERA_CV::ir_callback, &cc);
   cc.imshow_start = cc.nh.advertiseService(cc.IMSHOW_SERVICE_START, &CAMERA_CV::calibration_start_service, &cc);
   cc.imshow_stop = cc.nh.advertiseService(cc.IMSHOW_SERVICE_STOP, &CAMERA_CV::calibration_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg::Coordinate>(cc.PUBLISH_TOPIC, 1000);
   std_srvs::Empty _emp;
   while(ros::ok()){
      // cout << cc.getRun() << endl;
       clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
       
      if(cc.getRun()){
            cc.DrawCircle(0,0);
      }
      if(!cc.src.empty()){
        // setMouseCallback("src", mouseEvent, &cc);
        clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        double _fps = (double)1/(double)(fstop-fstart);
        std::string fps= "FPS: " + std::to_string(_fps);
        putText(cc.src, //target image
          fps, //text
          Point(10, 30), //top-left position
          FONT_HERSHEY_DUPLEX,
          1.0,
          Scalar(118, 185, 0), //font color
          2);
      
        imshow( "src", cc.src);
        waitKey(3);
      }

      ros::spinOnce();
      loop_rate.sleep();
   }
  destroyAllWindows();
  return 0;
}
