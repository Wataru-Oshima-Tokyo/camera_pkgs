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
//  #include <object/Coordinate.h>
 #include <camera_pkg_msgs/Coordinate.h>
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

class DETECTOBJ{
  public:
    //variables
    Mat src, src_gray, src_hsv, dst, detected_edges, mask;
    Mat depth;
    ros::Publisher pub;
    ros::Subscriber image_sub, depth_sub, darknet_bbox_sub;
    ros::NodeHandle nh;
    camera_pkg_msgs::Coordinate coordinate;
    ros::ServiceServer pickup_start, pickup_stop;
    int lowThreshold;
    int low_c[3] = {162, 174, 126};
    int high_c[3] = {188, 203, 254};
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
    void CannyThreshold(int, void*);
    void MaskThreshold(int, void*);
    void DrawCircle(int, void*);
//     void detect_object(int , void* userdata);
    void mouseEvent(int event, int x, int y, int flags, void* userdata);
    // Mat getDepth();
    const std::string OPENCV_WINDOW = "Image window";
    virtual bool maskdetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool maskdetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    // const std::string DEPTH_TOPIC = "/camera/depth/color/image_raw";
    const std::string PUBLISH_TOPIC = "/objectdetection/coordinate";
    const std::string PICKUP_SERVICE_START = "/pickup/start";
    const std::string PICKUP_SERVICE_STOP = "/pickup/stop";
    const std::string MASK_DETECT_SERVICE_START = "/maskdetect/start";
    const std::string MASK_DETECT_SERVICE_STOP = "/maskdetect/stop";

    DETECTOBJ();
    ~DETECTOBJ();
    bool getRun(); 
    const int max_lowThreshold = 100;
    const std::string window_name = "Edge Map";
private:
    bool RUN = false;
    double detect_probability =0.0;
    bool detected=false;
    bool start_call = true;
    bool stop_call = false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
};


DETECTOBJ::DETECTOBJ(){
  
  ros::NodeHandle private_nh("~");
  private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
  private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/camera/aligned_depth_to_color/image_raw"));
  lowThreshold = 6;
};

DETECTOBJ::~DETECTOBJ(){};

bool DETECTOBJ::getRun(){    
  return RUN;
}

// void DETECTOBJ::detect_object(int, void* userdata){
//     DETECTOBJ *cc = (DETECTOBJ*)userdata;
//     cv::Point pt1(detected_object.xmin, detected_object.ymin);
//     cv::Point pt2(detected_object.xmax, detected_object.ymax);
//     cv::Point center((detected_object.xmax+detected_object.xmin)/2, (detected_object.ymax+detected_object.ymin)/2);
//     cv::circle(src, center, 5, cv::Scalar(0, 0, 255));
//     cv::putText(src, "Cup", pt2, FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 185, 0), 2);
//     cv::rectangle(src, pt1, pt2, cv::Scalar(0,255,0));
//     int z = cc->depth.at<uint16_t>(center.y,center.x);
//     cc->coordinate.x = center.x;
//     cc->coordinate.y = center.y;
//     if(cc->coordinate.x !=0 && cc->coordinate.y!=0){
//       cc->coordinate.z = z;
//     }else{
//       cc->coordinate.z =0;
//     }
    
//     cc->pub.publish(coordinate);
// }


void DETECTOBJ::DrawCircle(int, void*){
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

void DETECTOBJ::MaskThreshold(int, void*userdata){
   DETECTOBJ *cc = (DETECTOBJ*)userdata;
   
   cv::inRange(src_hsv, cv::Scalar(low_c[0],low_c[1],low_c[1]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
//    Canny(mask, mask, lowThreshold, lowThreshold*ratio, kernel_size );
 
   cv::Moments M = cv::moments(mask); // get the center of gravity
   if (M.m00 >0){
   			int cx = int(M.m10/M.m00); //重心のx座標
   			int cy = int(M.m01/M.m00); //重心のy座標
      
      cv::circle(src, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
        int z = cc->depth.at<uint16_t>(cy,cx);
        cc->coordinate.x = cx;
        cc->coordinate.y = cy;
        if(cc->coordinate.x !=0 && cc->coordinate.y!=0){
        cc->coordinate.z = z;
        }else{
        cc->coordinate.z =0;
        }
        cc->pub.publish(coordinate);
   }
    imshow( "mask", mask);
    waitKey(3);
 
   
   
}


 bool DETECTOBJ::maskdetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "detection starts" << endl;
   RUN = true;
   return RUN;

 }

 bool DETECTOBJ::maskdetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "detection stops" << endl;
   RUN = false;
   return RUN;
 }


void DETECTOBJ::depth_callback(const sensor_msgs::ImageConstPtr& msg){
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

 void DETECTOBJ::image_callback(const sensor_msgs::ImageConstPtr& msg){
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    src = cv_ptr->image;
    dst.create(src.size(), src.type());
    //cvtColor(src, src_gray, COLOR_BGR2GRAY);
    cvtColor(src, src_hsv, COLOR_BGR2HSV);
    
    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

 }


int main( int argc, char** argv )
{

   ros::init(argc, argv, "mask_detector");
   DETECTOBJ cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(20);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &DETECTOBJ::image_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 1000, &DETECTOBJ::depth_callback, &cc);
//    cc.darknet_bbox_sub = cc.nh.subscribe(cc.BBOX_TOPIC, 1000, &DETECTOBJ::bbox_callback, &cc);
   cc.pickup_start = cc.nh.advertiseService(cc.PICKUP_SERVICE_START, &DETECTOBJ::maskdetect_start_service, &cc);
   cc.pickup_stop = cc.nh.advertiseService(cc.PICKUP_SERVICE_STOP, &DETECTOBJ::maskdetect_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 1000);
   
   std_srvs::Empty _emp;
   while(ros::ok()){
      // cout << cc.getRun() << endl;
      clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
      if(cc.getRun()){
          cc.MaskThreshold(0,&cc);
      }
      if(!cc.src.empty()){
        // setMouseCallback("src", mouseEvent, &cc);
        clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));
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
