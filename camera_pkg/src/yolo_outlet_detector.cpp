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
 #include "darknet_ros_msgs/BoundingBoxes.h"
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
    Mat src, src_gray, src_hsv, dst, detected_edges, mask;
    Mat depth;
    ros::Publisher pub;
    ros::Subscriber image_sub, depth_sub, darknet_bbox_sub;
    ros::NodeHandle nh;
    camera_pkg_msgs::Coordinate coordinate;
    ros::ServiceServer pickup_start, pickup_stop;
    int lowThreshold;
    int low_c[3] = {50, 138, 157};
    int high_c[3] = {100, 255, 197};
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
    void CannyThreshold(int, void*);
    void MaskThreshold(int, void*);
    void DrawCircle(int, void*);
//     void detect_object(int , void* userdata);
    // void mouseEvent(int event, int x, int y, int flags, void* userdata);
    // Mat getDepth();
    const std::string OPENCV_WINDOW = "Image window";
    virtual bool outletdetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool outletdetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    virtual void bbox_callback(const darknet_ros_msgs::BoundingBoxes& cod);
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    // const std::string DEPTH_TOPIC = "/camera/depth/color/image_raw";
    const std::string PUBLISH_TOPIC = "/objectdetection/coordinate";
    const std::string BOUNDING_BOX_TOPIC ="/darknet_ros/bounding_boxes";
    const std::string PICKUP_SERVICE_START = "/outletdetect/start";
    const std::string PICKUP_SERVICE_STOP = "/outletdetect/stop";
    std::string mode ="";
    OUTLET_CV();
    ~OUTLET_CV();
    bool getRun(); 
    void setRun(bool run);
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

void OUTLET_CV::bbox_callback(const darknet_ros_msgs::BoundingBoxes& cod){
	int xmin = cod.bounding_boxes[0].xmin;
	int xmax = cod.bounding_boxes[0].xmax;
	int ymin = cod.bounding_boxes[0].ymin;
	int ymax = cod.bounding_boxes[0].ymax;
  cv::rectangle(src, cv::Point(xmin, ymin), cv::Point(xmax,ymax), cv::Scalar(255,0,0),3);
  //get the center coordinate of the rectabgle
	int cx = (xmax+xmin)/2;
	int cy = (ymax+ymin)/2;
	std::vector<double> z_array;
	double z=0.0;
	// below process needs to be done because the z-value of realsense is not stable
	rep(i,0,5)
		rep(j,0,5){
		  z = depth.at<uint16_t>((uint16_t)(cy+j),(uint16_t)(cx+i));
		  z_array.push_back(z);
		}
	std::sort(z_array.begin(), z_array.end());
	z = z_array[z_array.size()-1];
        coordinate.x = cx;
        coordinate.y = cy;
        if(coordinate.x !=0 && coordinate.y!=0){
        	coordinate.z = z;
        }else{
        	coordinate.z =0;
        }
	if(coordinate.z !=0){
		pub.publish(coordinate);
		RUN = false;
	}
}

// void OUTLET_CV::DrawCircle(int, void*){
//   int x = src.cols, y = src.rows;
//   vector<int> x_array = {x/2-70, x/2, x/2+70, x/2+140};
//   vector<int> y_array = {y/2-140, y/2-70, y/2, y/2+70};
//   //draw circle 9;
//   //top
//   rep(i,0,x_array.size()){
//     rep(j,0,y_array.size()){
//       int _radius =13;
//       int _saturation1 = 153;
//       int _saturation2 =0;
//       if(j%2==0 && i%2==0){ _radius =20;_saturation1 = 100; _saturation2 =100;} 
//       else if(j%2==0) {_radius = 17; _saturation1 = 0; _saturation2 =150;}
//       else if(i%2==0) {_radius = 25; _saturation1 = 200; _saturation2 =10;}
//       // cv::circle(src_hsv, cv::Point(x_array[i],y_array[j]), 15, cv::Scalar(153, 0, 255), 5);
//       cv::circle(src, cv::Point(x_array[i],y_array[j]), _radius, cv::Scalar(_saturation1, _saturation2, 255),5);
//     }
//   }

// }

// void OUTLET_CV::MaskThreshold(int, void*userdata){
//    OUTLET_CV *cc = (OUTLET_CV*)userdata;
   
//    cv::inRange(src_hsv, cv::Scalar(low_c[0],low_c[1],low_c[2]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
// //    Canny(mask, mask, lowThreshold, lowThreshold*ratio, kernel_size );
 
//    cv::Moments M = cv::moments(mask); // get the center of gravity
//    if (M.m00 >0){
//    			int cx = int(M.m10/M.m00); //重心のx座標
//    			int cy = int(M.m01/M.m00); //重心のy座標
      
//       cv::circle(src, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
//         int z = cc->depth.at<uint16_t>(cy,cx);
//         cc->coordinate.x = cx;
//         cc->coordinate.y = cy;
//         if(cc->coordinate.x !=0 && cc->coordinate.y!=0){
//         cc->coordinate.z = z;
//         }else{
//         cc->coordinate.z =0;
//         }
//         cc->pub.publish(coordinate);
//    }
//     imshow( "mask", mask);
//     waitKey(3);
 
// }


 bool OUTLET_CV::outletdetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "detection starts" << endl;
   RUN = true;
   return RUN;

 }

 bool OUTLET_CV::outletdetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "detection stops" << endl;
   RUN = false;
   return RUN;
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
    clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
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
    //cvtColor(src, src_hsv, COLOR_BGR2HSV);
   
    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

 }


//  void mouseEvent(int event, int x, int y, int flags, void* userdata)
// {
//      OUTLET_CV *cc = (OUTLET_CV*)userdata;
//     //  ros::Publisher* _pub = cc->pub;
//     //  _cc.pub = _cc.nh.advertise<std_msgs::String>(_cc.PUBLISH_TOPIC, 1000);
//      camera_pkg_msgs::Coordinate coordinate;
//     //  Mat* _depth = &depth;
//      double z=0.0;
//      z = cc->depth.at<uint16_t>((uint16_t)y,(uint16_t)x);
     
//     //  cout << color[0] << " "<< color[1] << " " << color[2] <<endl; 
//      if  ( event == EVENT_LBUTTONDOWN )
//      {

// 	  	cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
// 		  cc->mode = "L";
//       if(!cc->getRun()){
// 		      Vec3b &color = cc->src_hsv.at<Vec3b>(Point(y,x));
//         	cc->low_c[0] = color[0] -10; cc->low_c[1] = color[1] -10; cc->low_c[2] = color[2] -40;
//         	cc->high_c[0] = color[0] +10; cc->high_c[1] = color[1] +10; cc->high_c[2] = color[2] +20;
//         	// ROS_INFO_STREAM("The MIN color: %d, %d, %d", low_c[0],low_c[1],low_c[2]);
//         	// ROS_INFO_STREAM("The MAX color: %d, %d, %d", high_c[0],high_c[1],high_c[2]);
//         	printf("The MIN color: %d, %d, %d\n", cc->low_c[0],cc->low_c[1],cc->low_c[2]);
//         	printf("The MAX color: %d, %d, %d\n", cc->high_c[0],cc->high_c[1],cc->high_c[2]);	
// 	 }
//           cc->setRun(false);
 
//      }
//      else if  ( event == EVENT_RBUTTONDOWN )
//      {
//           cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
//           cc->mode = "R";
//       if(!cc->getRun())
//           cc->setRun(true);
//      }
//      else if  ( event == EVENT_MBUTTONDOWN )
//      {
//           cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
//           cc->mode = "M";
//      }
     

// }


int main( int argc, char** argv )
{

   ros::init(argc, argv, "mask_detector");
   OUTLET_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(1000);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &OUTLET_CV::image_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 1000, &OUTLET_CV::depth_callback, &cc);
   cc.darknet_bbox_sub = cc.nh.subscribe(cc.BOUNDING_BOX_TOPIC, 1000, &OUTLET_CV::bbox_callback, &cc);
   cc.pickup_start = cc.nh.advertiseService(cc.PICKUP_SERVICE_START, &OUTLET_CV::outletdetect_start_service, &cc);
   cc.pickup_stop = cc.nh.advertiseService(cc.PICKUP_SERVICE_STOP, &OUTLET_CV::outletdetect_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 1000);
   
   std_srvs::Empty _emp;
   while(ros::ok()){
      // cout << cc.getRun() << endl;
      if(!cc.src.empty()){
        if(cc.getRun()){
            // cc.MaskThreshold(0,&cc);
        }
        // setMouseCallback("src", mouseEvent, &cc);
        std::string exp="";
        if(cc.mode =="L" || cc.mode == "R"){
            if(cc.getRun())
                exp ="Get Coordinate";
            else 
                exp ="Color checker";
        }
        else if (cc.mode == "M"){
            break;
        }
	clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
	std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));
        putText(cc.src, //target image
          fps, //text
          Point(10, 30), //top-left position
          FONT_HERSHEY_DUPLEX,
          1.0,
          Scalar(118, 185, 0), //font color
          2);
        putText(cc.src, exp, Point(10, 65), FONT_HERSHEY_DUPLEX,1.0,Scalar(0, 0, 255), 2);
//         imshow( "src", cc.src);
        //imshow("hsv", cc.src_hsv);
// 	waitKey(3);
      }
      loop_rate.sleep();
      ros::spinOnce();
      
   }
  destroyAllWindows();
  return 0;
}
