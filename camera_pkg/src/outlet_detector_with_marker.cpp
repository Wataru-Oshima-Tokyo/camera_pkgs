 #include <ros/ros.h>

 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <geometry_msgs/Twist.h>
 #include <mg400_bringup/RobotStatus.h>
 #include "std_msgs/String.h"
 #include "std_msgs/Bool.h"
 #include "std_srvs/Empty.h"
 #include <vector>
//  #include <object/Coordinate.h>
 #include <camera_pkg_msgs/Coordinate.h>
 #include <map>
 #include <math.h>

// #include <camera_pkg/Camera_CV.h>
 #define IMG_HEIGHT (480)
 #define IMG_WIDTH (640)
 #define rep(i,a,b) for(int i=a;i<b;i++)
 #define fore(i,a) for(auto &i:a)
 using namespace std;
 using namespace cv;

struct timespec timer_start, timer_stop;
double fstart, fstop, detect_start, detect_stop, total_time_start, total_time_stop;

class OUTLET_CV{
  public:
    //variables
    Mat src, dst; //for the first camera
    Mat u_src, u_dst ; // for the second camera
    Mat depth; //for the depth cammera
    Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    ros::Publisher coordinate_pub, mg400_cmd_vel_pub, target_cmd_vel_pub;
    ros::Subscriber image_sub, depth_sub, mg400_sub, usbcam_sub, mg400_status;
    ros::NodeHandle nh;
    camera_pkg_msgs::Coordinate coordinate;
    ros::ServiceServer detect_srv_start, detect_srv_stop, detect_srv_reset;
    int lowThreshold;
    int ix,iy,cx,cy;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;

    virtual bool arucodetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool arucodetect_reset_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool arucodetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void mg400_status_callback(const mg400_bringup::RobotStatus&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    virtual void mg400_callback(const std_msgs::Bool&);
    virtual void usbcam_callback(const sensor_msgs::ImageConstPtr&);
    virtual void aruco_marker_detector();
    virtual void adjustArm(double &x, double &y, double &z, double &ang);
    virtual bool correct_position(double &);
    virtual void InitializeValues();
    virtual bool adjust_height(const double &height, const double &depth);
    virtual void putTexts(const double &x, const double &y, const double &z, const double &r);
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    std::string USBCAM_TOPIC;
    std::string TARGET_CMDVELTOPIC;
    double adjust_speed;
    const std::string COORDINATE_PUBLISH_TOPIC = "/outlet/coordinate";
    const std::string MG400_CMD_VEL_TOPIC = "/mg400/cmd_vel";
    const std::string MG400STATUS_TOPIC ="/mg400_bringup/msg/RobotStatus";
    const std::string MG400_TOPIC = "/mg400/working";
    const std::string ARUCO_DETECT_SERVICE_START = "/arucodetect/start";
    const std::string ARUCO_DETECT_SERVICE_RESET = "/arucodetect/reset";
    const std::string ARUCO_DETECT_SERVICE_STOP = "/arucodetect/stop";
    
    OUTLET_CV();
    ~OUTLET_CV();
    bool getRun(); 
    void setRun(bool run);
    int w,h; //heihgt and width for the first cam
    int u_w, u_h; //height and width for the second cam
    bool Drew = false;
    bool drawing = false;
    bool ADJUST=false;
    double offset_x =0; double offset_y=0; double offset_z=0;
    double fixed_x,fixed_y, fixed_z;
    double c_x,c_y;
    bool initial = true;  
    bool final = false;
    double Kp;
    std::string mode ="";
private:
    bool RUN = false; 
    double detect_probability =0.0;
    bool detected=false;
    bool initial_position = true;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
    bool Done_x = false; bool Done_y = false; bool Done_z = false; bool Done_r= false;
    bool Done_depth = false;
    bool Done_height = false;
    bool mg400_running = false;
    const double timer = 1.5;
    const double quit_searching = 120; 
    int offset_x_counter =0;
    int offset_y_counter =0;
    int _counter =0;
    double angle=0;
    double threshold_x, threshold_y, threshold_z;
    std::string CALIBRATION;
    std::vector<double> rvecs_array;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
};


OUTLET_CV::OUTLET_CV(){
  
    ros::NodeHandle private_nh("~");
    private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
    private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/camera/aligned_depth_to_color/image_raw"));
    private_nh.param("usbcam_topic", USBCAM_TOPIC, std::string("/usb_cam/color/image"));
    private_nh.param("target_cmd_vel", TARGET_CMDVELTOPIC, std::string("cmd_vel"));
    private_nh.param("offset_fixed_x", fixed_x, -0.075);
    private_nh.param("offset_fixed_y", fixed_y, -0.04);
    private_nh.param("offset_fixed_z", fixed_z, 0.37);
    private_nh.param("Kp", Kp, 30.0);
    private_nh.param("adjust_speed",adjust_speed, 0.05);
    private_nh.param("calibration_path", CALIBRATION, std::string(""));
    std::cout << "calibration path: " <<  CALIBRATION << std::endl; 
    cv::FileStorage fs;
    fs.open(CALIBRATION, cv::FileStorage::READ); 
    if (!fs.isOpened())
    {
        std::cout << "Failed to open " << CALIBRATION << std::endl;
    }
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
};

OUTLET_CV::~OUTLET_CV(){};

bool OUTLET_CV::getRun(){    
  return RUN;
}

void OUTLET_CV::setRun(bool run){
    RUN = run;
}

/*
In case the target aruco marker is out of range from the robot arm 
If it is holonomic, we can alsot conside adding 
*/
bool OUTLET_CV::correct_position(double &x ){
  // printf("%lf\n", c_x);
  geometry_msgs::Twist twist;
  if(x <w/5){
    twist.linear.x = adjust_speed;
  }else if (x> (4*w/5)){
    twist.linear.x = -adjust_speed;
  }else{
    return true;
  }
  target_cmd_vel_pub.publish(twist);
  return true;
}

void OUTLET_CV::adjustArm(double &x, double &y, double &z, double &ang){
  //expand the ROI to detect how off the MG400 is
    //GaussianBlur( dst, dst, Size(9, 9), 2, 2 );
    geometry_msgs::Twist twist;
    offset_x = (double)fixed_x - x;
    offset_y = (double)fixed_y - y;
    offset_z = (double)fixed_z - z;
    //比率ゲイン
    // double Kp_y = 0.01;
    double Kv = 0.0;
    double _Kp = Kp;
    if (Done_r){
        threshold_x = 0.0005;
        threshold_y = 0.0002;
        threshold_z = 0.002; //0.001
      }else{
        _Kp *= 3;
        threshold_x = 0.01;
        threshold_y = 0.01;
        threshold_z = 0.1;
    }
    //PD control
    double move_x = _Kp*offset_x - Kv*offset_x/1000;
    double move_y = _Kp*offset_y - Kv*offset_y/1000;
    double move_z = _Kp*offset_z - Kv*offset_z/1000;

    if (!mg400_running && (fstop-fstart)>timer && (!Done_x || !Done_y)){
      // if(!Done_z)
      twist.linear.x = -move_z; // depth
      twist.linear.y = move_x; // horizontal 
      twist.linear.z = move_y; // vertical
      
      clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
      // printf("\nOffset_x: %lf, Offset_y: %lf\n", offset_x, offset_y);
      // printf("\nlinear.y: %lf, linear.z: %lf\n", twist.linear.y, twist.linear.z);
      ROS_INFO("\nOffset_x: %lf, Offset_y: %lf, Offset_z: %lf\n", offset_x, offset_y, offset_z);
      ROS_INFO("\nlinear.x: %lf, linear.y: %lf, linear.z: %lf\n", twist.linear.x, twist.linear.y, twist.linear.z);
      mg400_cmd_vel_pub.publish(twist);

      if(std::abs(offset_z)<=threshold_z){
            Done_z = true;
      }
      if(Done_z){
        if(std::abs(offset_x)<=threshold_x){
            Done_x = true;
        }
        if(std::abs(offset_y)<=threshold_y){
            Done_y = true;
        }
      }

    }
    clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
    if(!mg400_running && Done_x && Done_y && (fstop-fstart)>timer && Done_r){
      if (!final){
        coordinate.t ="F";
        coordinate.x = 10;
        coordinate.y = 10;
        coordinate.z = 10;
        coordinate_pub.publish(coordinate);
        final =true;
        RUN = false;
        clock_gettime(CLOCK_MONOTONIC, &timer_start); detect_start=(double)timer_start.tv_sec;
        clock_gettime(CLOCK_MONOTONIC, &timer_stop); detect_stop=(double)timer_stop.tv_sec;
        clock_gettime(CLOCK_MONOTONIC, &timer_start); total_time_stop=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
      }
    clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);

    }else if(!mg400_running && Done_x && Done_y && (fstop-fstart)>timer && !Done_r ){
      coordinate.t ="D";
      coordinate.r = angle;
      coordinate_pub.publish(coordinate);
      Done_r = true;
      Done_x = false;
      Done_y = false;
      Done_z = false;
      clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
    }else if(!mg400_running && !Done_x &&! Done_y && (fstop-fstart)>timer && Done_r && std::abs(ang) > 3 ){
      coordinate.t = "A";
      coordinate.r = ang;
      coordinate_pub.publish(coordinate);
      clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
    }
    
}


bool OUTLET_CV::adjust_height(const double &height, const double &depth){
  geometry_msgs::Twist twist;
  const double diff_height = 0.0114 - height;
  const double diff_depth = 0.13- depth;
  const double _Kp = Kp*3;
  const double move_height = _Kp*diff_height; 
  const double move_depth = _Kp*diff_height; 
  twist.linear.z = move_height; // vertical
  twist.linear.x = -move_depth;
  const double threshold_height = 0.0005;  
  const double threshold_depth = 0.03;         
  if (std::abs(diff_height)<=threshold_height){
    Done_height =true;
  }
  if (std::abs(diff_depth)<=threshold_depth){
    Done_depth = true; 
  }
  if (initial_position){
    clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
      if(Done_height && Done_depth){
        clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
        initial_position =false;
        Done_height = false;
        Done_depth = false;
        return true;  
      }else{
        if(!mg400_running && (fstop-fstart)>timer){
            clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
            mg400_cmd_vel_pub.publish(twist);
            std::cout << "diff_height: " << std::abs(diff_height) << "diff_depth: " << std::abs(diff_depth) << std::endl;
          }
          return false;
      }
  }else{
    return true;
  }
}

  void OUTLET_CV::putTexts(const double &x, const double &y, const double &z, const double &r){
                // This section is going to print the data for all the detected
            // markers. If you have more than a single marker, it is
            // recommended to change the below section so that either you
            // only print the data for a specific marker, or you print the
            // data for each marker separately.
            vector_to_marker.str(std::string());
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "x: " << std::setw(8) << x;
            cv::putText(u_src, vector_to_marker.str(),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "y: " << std::setw(8) << y;
            cv::putText(u_src, vector_to_marker.str(),
                        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "z: " << std::setw(8) << z;
            cv::putText(u_src, vector_to_marker.str(),
                        cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);
            
                        vector_to_marker.str(std::string());

            vector_to_marker << std::setprecision(4)
                                << "angle: " << std::setw(8) << r;
            cv::putText(u_src, vector_to_marker.str(),
                        cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);
  }

  void OUTLET_CV::aruco_marker_detector(){          
    if(initial){
      aruco::detectMarkers(src, dictionary, corners, ids);
      clock_gettime(CLOCK_MONOTONIC, &timer_start); detect_start=(double)timer_start.tv_sec;
      clock_gettime(CLOCK_MONOTONIC, &timer_stop); detect_stop=(double)timer_stop.tv_sec;
    } else{
      clock_gettime(CLOCK_MONOTONIC, &timer_start); detect_start=(double)timer_start.tv_sec;
      //erase it below later
      // clock_gettime(CLOCK_MONOTONIC, &timer_stop); detect_stop=(double)timer_stop.tv_sec;
      // clock_gettime(CLOCK_MONOTONIC, &timer_start); total_time_start=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
      // clock_gettime(CLOCK_MONOTONIC, &timer_stop); total_time_stop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
      //------------------------
      aruco::detectMarkers(u_src, dictionary, corners, ids);
    }

    //if the total time searching for the marker, then quit (120 secs)
    if (std::abs(total_time_stop-total_time_start)>quit_searching){
        std::cout << "Detection time exceeds 120 secs" <<std::endl;
        arucodetect_reset_service(req,res);
    }
        
        
    // if at least one marker detected
    
    if (std::abs(detect_start-detect_stop) >5){
      std::cout << "Detection timeout" <<std::endl;
      arucodetect_reset_service(req, res);
    } 
    if (ids.size() > 0){
        // printf("detected\n");
        aruco::drawDetectedMarkers(src, corners, ids);
        c_x = (corners[0][0].x + corners[0][1].x)/2;
        c_y = (corners[0][0].y + corners[0][3].y)/2;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::Mat rot_mat;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs, rvecs, tvecs);
        if(initial){
            if(correct_position(c_x)){
              std::vector<double> z_array;
              double z=0.0;
              rep(i,0,5)
                rep(j,0,5){
                  z = depth.at<uint16_t>((uint16_t)(c_y+j),(uint16_t)(c_x+i));
                  z_array.push_back(z);
              }
              std::sort(z_array.begin(), z_array.end());
              z = z_array[z_array.size()/2-1]; 
              z = depth.at<uint16_t>((uint16_t)(c_y),(uint16_t)(c_x));
              coordinate.t = "L";
              coordinate.x = c_x;
              coordinate.y = c_y;
              if(coordinate.x !=0 && coordinate.y!=0){
                  coordinate.z = z;
              }else{
                  coordinate.z = 0;
              }
              coordinate.r = 0;
              coordinate_pub.publish(coordinate);
              initial = false;
              destroyAllWindows();
              clock_gettime(CLOCK_MONOTONIC, &timer_start); total_time_start=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
              clock_gettime(CLOCK_MONOTONIC, &timer_stop); total_time_stop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
            }
        }else{
            for(int i=0; i < ids.size(); i++)
            {
              clock_gettime(CLOCK_MONOTONIC, &timer_stop); detect_stop=(double)timer_stop.tv_sec;
              cv::drawFrameAxes(u_src, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.1);
              if (!adjust_height(tvecs[0](1),tvecs[0](2)))
                break;
                  

              if (_counter>40){
                std::sort(rvecs_array.begin(), rvecs_array.end());
                angle = rvecs_array[rvecs_array.size()/2-1]*180/M_PI;               
                _counter = -1;
                break;
              }else if (_counter>=0){
                rvecs_array.push_back(rvecs[0](2));
                _counter++;
                break;
              }
              double _angle = rvecs[0](2)*180/M_PI;
              putTexts(tvecs[0](0),tvecs[0](1),tvecs[0](2),_angle);

            if(!mg400_running)
              adjustArm(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle);
            
            }  
        }
        clock_gettime(CLOCK_MONOTONIC, &timer_stop); total_time_stop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
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
    w = src.size().width;
    h = src.size().height;
    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

  }

  void OUTLET_CV::usbcam_callback(const sensor_msgs::ImageConstPtr& msg){
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

  u_src = cv_ptr->image;
  u_w = u_src.size().width;
  u_h = u_src.size().height;

  }

  void OUTLET_CV::InitializeValues(){
  Done_r = false;
  Done_x = false;
  Done_y = false;
  Done_z = false;
  Done_height = false;
  Done_depth = false;
  clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
  clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
  clock_gettime(CLOCK_MONOTONIC, &timer_start); detect_start=(double)timer_start.tv_sec;
  clock_gettime(CLOCK_MONOTONIC, &timer_stop); detect_stop=(double)timer_stop.tv_sec;            
  clock_gettime(CLOCK_MONOTONIC, &timer_start); total_time_start=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
  clock_gettime(CLOCK_MONOTONIC, &timer_stop); total_time_stop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
  initial= true;
  final = false;
  initial_position = true;
  _counter=0;
  rvecs_array.clear();
}

  void OUTLET_CV::mg400_status_callback(const mg400_bringup::RobotStatus& msg){
    if(msg.robot_status == 7){
      mg400_running = true;
    }else {
      mg400_running = false;
    }
  }


  void OUTLET_CV::mg400_callback(const std_msgs::Bool& msg){
    if(!msg.data){ 
          ADJUST=true;
          // put the cmd_vel control below
    }
  }
bool OUTLET_CV::arucodetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "Detection starts" << endl;
   RUN = true;
   return RUN;

 }

 bool OUTLET_CV::arucodetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
   cout << "Detection stops" << endl;
   RUN = false;
   destroyAllWindows();
  //  InitializeValues();
   return true;

 }
 bool OUTLET_CV::arucodetect_reset_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  cout << "Detection resets" << endl;
  destroyAllWindows();
  coordinate.t = "I";
  if(Done_x && Done_y && Done_z && Done_r){
      coordinate.x = 10;
      coordinate.y = 10;
      coordinate.z = 10;
  }else{
      coordinate.x = 100;
      coordinate.y = 100;
      coordinate.z = 100;
  }
  coordinate_pub.publish(coordinate);
  InitializeValues();
  ros::Rate _rate(10);
  while((fstop-fstart)<5 && !mg400_running){
    _rate.sleep();
    clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
  }
  
  return true;
 }


int main( int argc, char** argv )
{

   ros::init(argc, argv, "mask_detector");
   OUTLET_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(30);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 100, &OUTLET_CV::image_callback, &cc);
   cc.usbcam_sub = cc.nh.subscribe(cc.USBCAM_TOPIC, 100, &OUTLET_CV::usbcam_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 100, &OUTLET_CV::depth_callback, &cc);
   cc.mg400_status = cc.nh.subscribe(cc.MG400STATUS_TOPIC,100, &OUTLET_CV::mg400_status_callback, &cc);
//    cc.darknet_bbox_sub = cc.nh.subscribe(cc.BBOX_TOPIC, 1000, &OUTLET_CV::bbox_callback, &cc);
   cc.detect_srv_start = cc.nh.advertiseService(cc.ARUCO_DETECT_SERVICE_START, &OUTLET_CV::arucodetect_start_service, &cc);
   cc.detect_srv_reset = cc.nh.advertiseService(cc.ARUCO_DETECT_SERVICE_RESET, &OUTLET_CV::arucodetect_reset_service, &cc);
   cc.detect_srv_stop = cc.nh.advertiseService(cc.ARUCO_DETECT_SERVICE_STOP, &OUTLET_CV::arucodetect_stop_service, &cc);
   cc.coordinate_pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.COORDINATE_PUBLISH_TOPIC, 100);
   cc.mg400_cmd_vel_pub = cc.nh.advertise<geometry_msgs::Twist>(cc.MG400_CMD_VEL_TOPIC,100);
   cc.target_cmd_vel_pub = cc.nh.advertise<geometry_msgs::Twist>(cc.TARGET_CMDVELTOPIC,100);
   cc.mg400_sub = cc.nh.subscribe(cc.MG400_TOPIC,100, &OUTLET_CV::mg400_callback, &cc);   
   std_srvs::Empty _emp;
   clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
   clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
   while(ros::ok()){
      if(!cc.src.empty() && !cc.u_src.empty()){
        if(cc.getRun()){
            cc.aruco_marker_detector();
            if(cc.initial){
            namedWindow("src", WINDOW_NORMAL);
            cv::resizeWindow("src", IMG_WIDTH, IMG_HEIGHT);
            imshow("src", cc.src);

            }
            else{
                namedWindow("out", WINDOW_NORMAL);
                cv::resizeWindow("out", IMG_WIDTH, IMG_HEIGHT);
                imshow("out", cc.u_src);
            }
        }
        waitKey(3);      
      }
      ros::spinOnce();
      loop_rate.sleep();
   }
  destroyAllWindows();
  return 0;
}
