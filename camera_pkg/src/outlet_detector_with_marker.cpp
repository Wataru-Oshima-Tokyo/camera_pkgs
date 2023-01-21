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
#include <mg400_bringup/InsertStatus.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include <vector>
#include <camera_pkg_msgs/Coordinate.h>
#include <map>
#include <math.h>
//include action files
#include <actionlib/server/simple_action_server.h>
#include <camera_action/camera_pkgAction.h>

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
    Mat camera_matrix, dist_coeffs; //for calibration parameters
    std::ostringstream vector_to_marker; //for putting text on the image
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50); //the dictionary for aruco marker 
    ros::Time start_time;
    std_srvs::Empty::Request req; // empty request
    std_srvs::Empty::Response res; //empty response

    ros::Publisher coordinate_pub_, mg400_cmd_vel_pub_, target_cmd_vel_pub_;//publish topics
    ros::Subscriber image_sub_, depth_sub_, mg400_sub_, usbcam_sub_, mg400_status_sub_; //subscribe topics
    ros::NodeHandle nh; 
    camera_pkg_msgs::Coordinate coordinate; //coordinate for sending to MG400
    ros::ServiceServer detect_srv_start_, detect_srv_stop_, detect_srv_reset_, insert_result_srv_; //service
    std::vector<int> ids; //ids for markers
    std::vector<std::vector<cv::Point2f> > corners; // corners for markers
    double adjust_speed; //the velociy for a mobile robots

    //actionlib servers
    camera_action::camera_pkgGoalConstPtr current_goal; // instance of a goal
    actionlib::SimpleActionServer<camera_action::camera_pkgAction> server;//make a server

    //functions
    virtual bool arucodetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);  //the service to start the detection
    virtual bool arucodetect_reset_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res); // the service to reset the detection
    virtual bool arucodetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res); // the service to stop the detection
    virtual bool insert_result_service(mg400_bringup::InsertStatus::Request& req, mg400_bringup::InsertStatus::Response& res); // the service to insert_result
    virtual void image_callback(const sensor_msgs::ImageConstPtr&); //to get an image from Realsense
    virtual void mg400_status_callback(const mg400_bringup::RobotStatus&); //to get a status of MG400
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&); //to get a depth image from Realsense
    virtual void usbcam_callback(const sensor_msgs::ImageConstPtr&); // to get a rgb image from USB camera 
    virtual void aruco_marker_detector(); //detecting function inside
    virtual void adjustArm(double &x, double &y, double &z, double &ang); // adjusting the robot arm by the coordinate obtained from the detected marker
    virtual bool correct_position(double &); //if the detected marker is at the edge of the image obtained by Realsense, move the mobile robot to the center 
    virtual void InitializeValues(); //initialzie all variables
    virtual bool adjust_height(const double &height, const double &depth); // to adjust the robot arm to the initial postion where the camera reads the angle of the detected marker 
    virtual void putTexts(const double &x, const double &y, const double &z, const double &r); //puttting some texts
    virtual void initial_detection(); //the function of detecting a marker by Realsense
    virtual void hand_camera_detction(); // the functioin of detecting a marker by USB camera
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    std::string USBCAM_TOPIC;
    std::string TARGET_CMDVELTOPIC;
    
    const std::string COORDINATE_PUBLISH_TOPIC = "/outlet/coordinate";
    const std::string MG400_CMD_VEL_TOPIC = "/mg400/cmd_vel";
    const std::string MG400STATUS_TOPIC ="/mg400_bringup/msg/RobotStatus";
    const std::string ARUCO_DETECT_SERVICE_START = "/arucodetect/start";
    const std::string ARUCO_DETECT_SERVICE_RESET = "/arucodetect/reset";
    const std::string ARUCO_DETECT_SERVICE_STOP = "/arucodetect/stop";
    const std::string INSERT_RESULT_SERVICE_TOPIC = "/insert_result";

    OUTLET_CV();
    ~OUTLET_CV();
    bool getRun(); 
    void setRun(bool run);
    bool setInsert_result(const bool &result);
    int w,h; //heihgt and width for the first cam
    int u_w, u_h; //height and width for the second cam
    double offset_x =0; double offset_y=0; double offset_z=0;
    double fixed_x,fixed_y, fixed_z;
    double c_x,c_y; // the x, y coordinate of center of the marker 
    bool initial = true;  //used to switch an image from realsense to USB camera
    bool _final = false; // to decide if MG400 tries to insert the plug or not
    double Kp; // the proportional coeficient
    std::string mode ="";
    bool insert_result = false; //insert_result  
    ros::Time insert_time; 
    bool show_image;
private:
    bool RUN = false; 
    double detect_probability =0.0;
    bool detected=false;
    bool initial_position = true;
    //They are for deciding if the detection is done or not
    bool Done_x = false; bool Done_y = false; bool Done_z = false; bool Done_r= false;
    //They are for deciding if MG400 is in the initial position
    bool Done_depth = false;
    bool Done_height = false;
    
    bool mg400_running = false; // if the MG400 is still moviing
    const double timer = 1.5; // the interval to send the command to MG400
    const double quit_searching = 120; //timer for the entire operation since it detected the marker
    int _counter =0; // for how many pcitures to get angle
    double angle=0; // the detected angle
    double threshold_x, threshold_y, threshold_z; //thresholds for the marker
    std::string CALIBRATION; //calibration path
    std::vector<double> rvecs_array; //the angele for marker
    std::vector<cv::Vec3d> rvecs, tvecs; //for marker angle and distance
};

//constructor
OUTLET_CV::OUTLET_CV():
server(nh, "charging_station", false)
{

  ros::NodeHandle private_nh("~");
  private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
  private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/camera/aligned_depth_to_color/image_raw"));
  private_nh.param("usbcam_topic", USBCAM_TOPIC, std::string("/usb_cam/color/image"));
  private_nh.param("target_cmd_vel", TARGET_CMDVELTOPIC, std::string("cmd_vel"));
  private_nh.param("offset_fixed_x", fixed_x, -0.075);
  private_nh.param("offset_fixed_y", fixed_y, -0.04);
  private_nh.param("offset_fixed_z", fixed_z, 0.37);
  private_nh.param("show_image", show_image, true);
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





  //Subscribers
  image_sub_ = nh.subscribe(IMAGE_TOPIC, 100, &OUTLET_CV::image_callback, this);
  usbcam_sub_ = nh.subscribe(USBCAM_TOPIC, 100, &OUTLET_CV::usbcam_callback, this);
  depth_sub_ = nh.subscribe(DEPTH_TOPIC, 100, &OUTLET_CV::depth_callback, this);
  mg400_status_sub_ = nh.subscribe(MG400STATUS_TOPIC,100, &OUTLET_CV::mg400_status_callback, this);

  //Services
  detect_srv_start_ = nh.advertiseService(ARUCO_DETECT_SERVICE_START, &OUTLET_CV::arucodetect_start_service, this);
  detect_srv_reset_ = nh.advertiseService(ARUCO_DETECT_SERVICE_RESET, &OUTLET_CV::arucodetect_reset_service, this);
  detect_srv_stop_ = nh.advertiseService(ARUCO_DETECT_SERVICE_STOP, &OUTLET_CV::arucodetect_stop_service, this);
  insert_result_srv_ = nh.advertiseService(INSERT_RESULT_SERVICE_TOPIC, &OUTLET_CV::insert_result_service, this);
  //Publishers
  coordinate_pub_ = nh.advertise<camera_pkg_msgs::Coordinate>(COORDINATE_PUBLISH_TOPIC, 100);
  mg400_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(MG400_CMD_VEL_TOPIC,100);
  target_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(TARGET_CMDVELTOPIC,100);
  
  //start the server
  server.start(); 
};

// destructor
OUTLET_CV::~OUTLET_CV(){};

//reutn the value of RUN
bool OUTLET_CV::getRun(){    
  return RUN;
}

//set the value of RUN
void OUTLET_CV::setRun(bool run){
    RUN = run;
}


/*
In case the target aruco marker is out of range from the robot arm 
If it is holonomic, we can alsot conside adding 
*/
bool OUTLET_CV::correct_position(double &x ){

  geometry_msgs::Twist twist;
  if(x <(3*w/10)){
    twist.linear.x = adjust_speed;
  }else if (x> (7*w/10)){
    twist.linear.x = -adjust_speed;
  }else{
    return true;
  }
  // target_cmd_vel_pub_.publish(twist);
  return true;
}

//this is a crucial function because it is moving the arm to the correct position to insert 
void OUTLET_CV::adjustArm(double &x, double &y, double &z, double &ang){
    geometry_msgs::Twist twist;
    offset_x = (double)fixed_x - x;
    offset_y = (double)fixed_y - y;
    offset_z = (double)fixed_z - z;
    //propotional coeficient
    double _Kp = Kp;
    if (Done_r){
        threshold_x = 0.0005;
        threshold_y = 0.0002;
        threshold_z = 0.002;
      }else{
        _Kp *= 3;
        threshold_x = 0.01;
        threshold_y = 0.01;
        threshold_z = 0.1;
    }
    //P control
    double move_x = _Kp*offset_x;
    double move_y = _Kp*offset_y;
    double move_z = _Kp*offset_z;


    if (!mg400_running && (fstop-fstart)>timer && (!Done_x || !Done_y)){
      twist.linear.x = -move_z; // depth
      twist.linear.y = move_x; // horizontal 
      twist.linear.z = move_y; // vertical
      
      clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
      ROS_INFO("\nOffset_x: %lf, Offset_y: %lf, Offset_z: %lf\n", offset_x, offset_y, offset_z);
      ROS_INFO("\nlinear.x: %lf, linear.y: %lf, linear.z: %lf\n", twist.linear.x, twist.linear.y, twist.linear.z);
      mg400_cmd_vel_pub_.publish(twist);

      if(std::abs(offset_z)<=threshold_z){
            Done_z = true;
      }
      //after the depth adjustment is done
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
      if (!_final){
        _final =true;
        coordinate.t ="F";
        coordinate.x = 10;
        coordinate.y = 10;
        coordinate.z = 10;
        coordinate_pub_.publish(coordinate);
        RUN = false;
        //after the operation is done, then initialize the timers
        clock_gettime(CLOCK_MONOTONIC, &timer_start); detect_start=(double)timer_start.tv_sec;
        clock_gettime(CLOCK_MONOTONIC, &timer_stop); detect_stop=(double)timer_stop.tv_sec;
        clock_gettime(CLOCK_MONOTONIC, &timer_start); total_time_stop=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
        insert_time = ros::Time::now();
      }
    clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
    }else if(!mg400_running && Done_x && Done_y && (fstop-fstart)>timer && !Done_r ){
      //if the angle adjustment is done 
      coordinate.t ="D";
      coordinate.r = angle;
      coordinate_pub_.publish(coordinate);
      Done_r = true;
      Done_x = false;
      Done_y = false;
      Done_z = false;
      clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
    }else if(!mg400_running && !Done_x &&! Done_y && (fstop-fstart)>timer && Done_r && std::abs(ang) > 3 ){
      //if the angle is more than 3, then adjust it
      coordinate.t = "A";
      coordinate.r = ang;
      coordinate_pub_.publish(coordinate);
      clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
    }
    
}

//this is the function for moving the arm to the intial postion to read the angle of a marker
bool OUTLET_CV::adjust_height(const double &height, const double &depth){
  geometry_msgs::Twist twist;
  const double diff_height = 0.0114 - height;
  const double diff_depth = 0.13- depth;
  const double _Kp = Kp*3;
  const double move_height = _Kp*diff_height; 
  const double move_depth = _Kp*diff_height; 
  twist.linear.z = move_height; // vertical
  twist.linear.x = -move_depth; // depth
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
      if(Done_height && Done_depth || std::abs(total_time_stop-total_time_start)>15){
        //if the adjustment is done or the time of operation exceeds 30 seconds, then finalize the position
        clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
        initial_position =false;
        Done_height = false;
        Done_depth = false;
        return true;  
      }else{
        if(!mg400_running && (fstop-fstart)>timer){
            //if the position is still not the correct position
            clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
            mg400_cmd_vel_pub_.publish(twist);
            std::cout << "diff_height: " << std::abs(diff_height) << "diff_depth: " << std::abs(diff_depth) << std::endl;
          }
          return false;
      }
  }else{
    return true;
  }
}

//this is the function to put some comments on the imshow screen
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

//this is the function to control the arm once the marked is detectd by realsense camera
void OUTLET_CV::initial_detection(){
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
            if (coordinate.z > 400){
              server.setPreempted();
              ROS_WARN("Too far away: Preempt Goal\n");
              setRun(false);
            }else{
              coordinate_pub_.publish(coordinate);
            }
            initial = false;
            destroyAllWindows();
            clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
            clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);    
            clock_gettime(CLOCK_MONOTONIC, &timer_start); total_time_start=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
            clock_gettime(CLOCK_MONOTONIC, &timer_stop); total_time_stop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
}

//this is the function to adjust the position of the arm based on the info from the hand eye camera
void OUTLET_CV::hand_camera_detction(){

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

//this is the function to detect a marker
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
      
      
  //if the marker has not been detected for 5 seconds
  if (std::abs(detect_start-detect_stop) >5 && !_final){
    std::cout << "Detection timeout" <<std::endl;
    arucodetect_reset_service(req, res);
  } 
  // if at least one marker detected
  if (ids.size() > 0){
      // printf("detected\n");
      aruco::drawDetectedMarkers(src, corners, ids);
      c_x = (corners[0][0].x + corners[0][1].x)/2;
      c_y = (corners[0][0].y + corners[0][3].y)/2;
      
      cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs, rvecs, tvecs);
      if(initial){
          if(correct_position(c_x)){
            initial_detection(); 
          }
      }else{
            hand_camera_detction();
      }
      //counting the total timer of operation regardless the marker is detected or not
      clock_gettime(CLOCK_MONOTONIC, &timer_stop); total_time_stop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
  }

}

// depth info call back function (Realsnese camera)
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

// rgb info call back function (Realsense camera)
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

// rgb info call back funciton (USB camera)
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


//initialize all the values
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
  _final = false;
  initial_position = true;
  _counter=0;
  rvecs_array.clear();
}

// return true if the robot status is 7 which means it is ready to receive the next command
void OUTLET_CV::mg400_status_callback(const mg400_bringup::RobotStatus& msg){
  if(msg.robot_status == 7){
    mg400_running = true;
  }else {
    mg400_running = false;
  }
}

bool OUTLET_CV::setInsert_result(const bool &result){
  insert_result = result;
  ROS_INFO("Set insert result initialized");
}

bool OUTLET_CV::insert_result_service(mg400_bringup::InsertStatus::Request& req, mg400_bringup::InsertStatus::Response& res){
  insert_result = true;
  ROS_INFO("Set insert result succeeded!");
  return true;
}

//detection starts by this service
bool OUTLET_CV::arucodetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  cout << "Detection starts" << endl;
  RUN = true;
  return RUN;

}

//detection stops by this service
bool OUTLET_CV::arucodetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  cout << "Detection stops" << endl;
  RUN = false;
  destroyAllWindows();
  return true;

}

//detection resets by this service
 bool OUTLET_CV::arucodetect_reset_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  cout << "Detection resets" << endl;
  destroyAllWindows();
  coordinate.t = "I";
  //if this reset is called after the entire operation is done 
  if(Done_x && Done_y && Done_z && Done_r){
      coordinate.x = 10;
      coordinate.y = 10;
      coordinate.z = 10;
      RUN = false;
  }else{ //otherwise;
      coordinate.x = 100;
      coordinate.y = 100;
      coordinate.z = 100;
  }
  //after sending it, the control porgram will change the behavior based on the value sent to it
  coordinate_pub_.publish(coordinate);
  InitializeValues();
  ros::Rate _rate(10);
  //wait for a little bit
  while((fstop-fstart)<5 && !mg400_running){
    _rate.sleep();
    clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
  }
  
  return true;
 }


int main( int argc, char** argv )
{

   ros::init(argc, argv, "outlet_detector");
   OUTLET_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(30);
   //start the timer
   clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
   clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
   while(ros::ok()){
      if(cc.server.isNewGoalAvailable()){
          cc.current_goal = cc.server.acceptNewGoal();
          cc.start_time = ros::Time::now();
          cc.setRun(true);
          cc.setInsert_result(false);
          cc.initial = true;
      }
      if(cc.server.isActive()){
        if(cc.server.isPreemptRequested()){
          cc.server.setPreempted();
          ROS_WARN("Preempt Goal\n");
          cc.setRun(false);
        }else{
          if(cc.start_time + ros::Duration(cc.current_goal->duration) < ros::Time::now()){
            cc.server.setPreempted();
            ROS_WARN("Preempt Goal\n");
            cc.setRun(false);
          }else{
            camera_action::camera_pkgFeedback feedback;
            feedback.rate = (ros::Time::now() - cc.start_time).toSec() / cc.current_goal->duration;
            cc.server.publishFeedback(feedback);
            if(!cc.src.empty() && !cc.u_src.empty()){
              if(cc.getRun()){
                  cc.aruco_marker_detector();
                  if(cc.initial){ //showing the screen received from realsense
                  if(cc.show_image){
                    namedWindow("src", WINDOW_NORMAL);
                    cv::resizeWindow("src", IMG_WIDTH, IMG_HEIGHT);
                    imshow("src", cc.src);
                  }
                  }else{ //showing the screen received from hand eye camera
                    if(cc.show_image){
                      namedWindow("out", WINDOW_NORMAL);
                      cv::resizeWindow("out", IMG_WIDTH, IMG_HEIGHT);
                      imshow("out", cc.u_src);
                    }
                  }
              }else if(cc._final){
                  if (cc.insert_time + ros::Duration(10) <ros::Time::now()){
                      cc.server.setPreempted();
                      ROS_WARN("Preempt Goal\n");
                      cc.setRun(false);
                  }
                  if(cc.insert_result){
                    destroyAllWindows();
                    cc.server.setSucceeded();
                  }
              }
              waitKey(3);      
            }
          }
        }
      }      
      ros::spinOnce();
      loop_rate.sleep();
   }
  
  return 0;
}
