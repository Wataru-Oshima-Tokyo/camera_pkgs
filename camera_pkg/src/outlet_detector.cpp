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

class OUTLET_CV{
  public:
    //variables
    Mat src, src_hsv, ROI, detected_edges, mask;
    Mat depth;
    ros::Publisher pub;
    ros::Subscriber image_sub, depth_sub, darknet_bbox_sub;
    ros::NodeHandle nh;
    camera_pkg_msgs::Coordinate coordinate;
    ros::ServiceServer pickup_start, pickup_stop;
    int lowThreshold;
    int ix,iy,cx,cy;
    int low_c[3] = {50, 138, 157};
    int high_c[3] = {100, 255, 197};
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
    void CannyThreshold(int, void*);
    void MaskThreshold(int, void*);
    void DrawCircle(int, void*);
    void makeRegion(int, void*userdata);
//     void detect_object(int , void* userdata);
    void mouseEvent(int event, int x, int y, int flags, void* userdata);
    void draw_region_of_interest(int event, int x, int y, int flags, void* userdata);
    void get_hsv(int event, int x, int y, int flags, void* userdata);
    // Mat getDepth();
    const std::string SRC_WINDOW = "src";
    const std::string HSV_WINDOW = "hsv";
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
    std::string mode ="";
    OUTLET_CV();
    ~OUTLET_CV();
    bool getRun(); 
    void setRun(bool run);
    const int max_lowThreshold = 100;
    const std::string window_name = "Edge Map";
    int w,h;
private:
    bool RUN = false;
    bool Drew = false;
    double detect_probability =0.0;
    bool detected=false;
    bool start_call = true;
    bool stop_call = false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
};


OUTLET_CV::OUTLET_CV(){
  
  ros::NodeHandle private_nh("~");
  private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
  private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/camera/aligned_depth_to_color/image_raw"));
  lowThreshold = 6;
};

OUTLET_CV::~OUTLET_CV(){};

bool OUTLET_CV::getRun(){    
  return RUN;
}

void OUTLET_CV::setRun(bool run){
    RUN = run;
}



void OUTLET_CV::MaskThreshold(int, void*userdata){
   OUTLET_CV *cc = (OUTLET_CV*)userdata;
   cvtColor(cc->ROI, cc->src_hsv, COLOR_BGR2HSV);
   cv::inRange(src_hsv, cv::Scalar(low_c[0],low_c[1],low_c[2]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
//    Canny(mask, mask, lowThreshold, lowThreshold*ratio, kernel_size );
    contours, _ = cv::findContours(mask, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      contour = max(contours, key=lambda x: cv::contourArea(x));
      cv::drawContours(mask, [contour], -1, color=255, thickness=-1);
   cv::Moments M = cv::moments(mask); // get the center of gravity
   if (M.m00 >0){
   		int cx = int(M.m10/M.m00); //重心のx座標
   		int cy = int(M.m01/M.m00); //重心のy座標
      std::vector<double> z_array;
      double z=0.0;
      cv::circle(src, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255),-1);
      rep(i,0,5)
        rep(j,0,5){
          z = cc->depth.at<uint16_t>((uint16_t)(y+j),(uint16_t)(x+i));
          z_array.push_back(z);
        }
        std::sort(z_array.begin(), z_array.end());
        z = z_array[z_array.size()-1]; 
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


 bool OUTLET_CV::maskdetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "detection starts" << endl;
   RUN = true;
   return RUN;

 }

 bool OUTLET_CV::maskdetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "detection stops" << endl;
   RUN = false;
   return RUN;
 }

void OUTLET_CV::makeRegion(int, void*userdata){
   OUTLET_CV *cc = (OUTLET_CV*)userdata;
   rep(i,0,w)
    rep(j,0,h){
        if((i>=0 && j<=iy)) || (i>=0 && i<ix) || (i>cx && i<w) ||(j<cy)){
          cv::Vec3b &color = ROI.at<cv::Vec3b>(cv::Point(j,i)); 
          color.val[0] = 0;
          color.val[1] = 0;
          color.val[2] = 0;
        }
    }
    }
}


void OUTLET_CV::depth_callback(const sensor_msgs::ImageConstPtr& msg){
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

 void OUTLET_CV::image_callback(const sensor_msgs::ImageConstPtr& msg){
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
    ROI.create(src.size(), src.type());
    //cvtColor(src, src_gray, COLOR_BGR2GRAY);
    
    w = src.size().width;
    h = src.size().height;
    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

 }

void get_hsv(int event, int x, int y, int flags, void* userdata){
  OUTLET_CV *cc = (OUTLET_CV*)userdata;
  if (event == EVENT_LBUTTONDOWN )
     {
      Vec3b &color = cc->src_hsv.at<Vec3b>(Point(y,x));
      cc->low_c[0] = color[0] -10; cc->low_c[1] = color[1] -10; cc->low_c[2] = color[2] -40;
      cc->high_c[0] = color[0] +10; cc->high_c[1] = color[1] +10; cc->high_c[2] = color[2] +40;
      // ROS_INFO_STREAM("The MIN color: %d, %d, %d", low_c[0],low_c[1],low_c[2]);
      // ROS_INFO_STREAM("The MAX color: %d, %d, %d", high_c[0],high_c[1],high_c[2]);
      printf("The MIN color: %d, %d, %d\n", cc->low_c[0],cc->low_c[1],cc->low_c[2]);
      printf("The MAX color: %d, %d, %d\n", cc->high_c[0],cc->high_c[1],cc->high_c[2]);	
      cc->MaskThreshold(0,0);
	 }
  
}

 void mouseEvent(int event, int x, int y, int flags, void* userdata)
{
     OUTLET_CV *cc = (OUTLET_CV*)userdata;
    //  ros::Publisher* _pub = cc->pub;
    //  _cc.pub = _cc.nh.advertise<std_msgs::String>(_cc.PUBLISH_TOPIC, 1000);
     camera_pkg_msgs::Coordinate coordinate;
    //  Mat* _depth = &depth;
     std::vector<double> z_array;
     double z=0.0;
     // below process needs to be done because the z-value of realsense is not stable
     rep(i,0,5)
        rep(j,0,5){
          z = cc->depth.at<uint16_t>((uint16_t)(y+j),(uint16_t)(x+i));
          z_array.push_back(z);
        }
      std::sort(z_array.begin(), z_array.end());
      z = z_array[z_array.size()-1]; //get the median value 
     if  ( event == EVENT_LBUTTONDOWN )
     {

	  	cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
		  cc->mode = "L";
      if(!cc->getRun()){
		      Vec3b &color = cc->src_hsv.at<Vec3b>(Point(y,x));
        	cc->low_c[0] = color[0] -10; cc->low_c[1] = color[1] -10; cc->low_c[2] = color[2] -40;
        	cc->high_c[0] = color[0] +10; cc->high_c[1] = color[1] +10; cc->high_c[2] = color[2] +40;
        	// ROS_INFO_STREAM("The MIN color: %d, %d, %d", low_c[0],low_c[1],low_c[2]);
        	// ROS_INFO_STREAM("The MAX color: %d, %d, %d", high_c[0],high_c[1],high_c[2]);
        	printf("The MIN color: %d, %d, %d\n", cc->low_c[0],cc->low_c[1],cc->low_c[2]);
        	printf("The MAX color: %d, %d, %d\n", cc->high_c[0],cc->high_c[1],cc->high_c[2]);	
	 }
          cc->setRun(false);
 
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
          cc->mode = "R";
      if(!cc->getRun())
          cc->setRun(true);
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
          cc->mode = "M";
     }
     

}

void draw_region_of_interest(int event, int x, int y, int flags, void* userdata)
{
     OUTLET_CV *cc = (OUTLET_CV*)userdata;
    //  ros::Publisher* _pub = cc->pub;
    //  _cc.pub = _cc.nh.advertise<std_msgs::String>(_cc.PUBLISH_TOPIC, 1000);
     if(event == EVENT_LBUTTONMOVE){
      cc->drawing =true;
      cc->ix = x; cc->iy=y;
     }else if(event == EVENT_MOUSEMOVE){
      if(cc->drawing){
        cv::rectangle(cc->src, (cc->ix,cc->iy) (x,y),(0,255,0),-1);
      }else if(event == EVENT_LBUTTONUP){
        cc->drawing = false;
        cv::rectangle(cc->src, (cc->ix,cc->iy), (x,y), (0,255,255),2);
        cc->cx = x; cc->cy = y;
        cc->Drew = true;
        cv::destroyAllWindows(); 
      }
     }


}

int main( int argc, char** argv )
{

   ros::init(argc, argv, "mask_detector");
   OUTLET_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(20);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &OUTLET_CV::image_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 1000, &OUTLET_CV::depth_callback, &cc);
//    cc.darknet_bbox_sub = cc.nh.subscribe(cc.BBOX_TOPIC, 1000, &OUTLET_CV::bbox_callback, &cc);
   cc.pickup_start = cc.nh.advertiseService(cc.PICKUP_SERVICE_START, &OUTLET_CV::maskdetect_start_service, &cc);
   cc.pickup_stop = cc.nh.advertiseService(cc.PICKUP_SERVICE_STOP, &OUTLET_CV::maskdetect_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 1000);
   
   std_srvs::Empty _emp;
   setMouseCallback("src", draw_region_of_interest, &cc);
   cv::namedWindow(cc.SRC_WINDOW,WINDOW_AUTOSIZE);

   while(ros::ok()){
      // cout << cc.getRun() << endl;
      clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
      if(!cc.src.empty()){
          if (!cc.Drew){
            imshow(cc.SRC_WINDOW, cc.src);
          }else{
            cv::rectangle(cc.src, (cc.ix,cc.iy), (cc.cx,cc.cy), (0,255,255),2);
            //make the region of interest
            cc.makeRegion(0, 0);
            imshow("ROI", cc.ROI);
            cv::namedWindow(cc.HSV_WINDOW,WINDOW_AUTOSIZE);
            cv::setMouseCallback(cc.HSV_WINDOW, get_hsv);
          }
        

	waitKey(3);
      }
      //loop_rate.sleep();
      ros::spinOnce();
      
   }
  destroyAllWindows();
  return 0;
}
