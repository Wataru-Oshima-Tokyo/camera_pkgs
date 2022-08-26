 #include <ros/ros.h>

 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

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

// #include <camera_pkg/Camera_CV.h>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define rep(i,a,b) for(int i=a;i<b;i++)
 #define fore(i,a) for(auto &i:a)
 using namespace std;
 using namespace cv;

struct timespec timer_start, timer_stop;
double fstart, fstop;

class OUTLET_CV{
  public:
    //variables
    Mat src, dst; //for the first camera
    Mat u_src, u_dst ; // for the second camera
    Mat depth; //for the depth cammera
    Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    ros::Publisher pub, cmd_vel_pub;
    ros::Subscriber image_sub, depth_sub, mg400_sub, usbcam_sub, mg400_status;
    ros::NodeHandle nh;
    camera_pkg_msgs::Coordinate coordinate;
    ros::ServiceServer pickup_start, pickup_stop;
    int lowThreshold;
    int ix,iy,cx,cy;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;

    void makeRegion(int, void*);
    void get_circle(int, void*);



    void get_hsv(int event, int x, int y, int flags, void* userdata);
    // Mat getDepth();
    const std::string SRC_WINDOW = "src";
    const std::string ROI_WINDOW = "roi";
    virtual bool arucodetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool arucodetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void mg400_status_callback(const mg400_bringup::RobotStatus&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    virtual void mg400_callback(const std_msgs::Bool&);
    virtual void usbcam_callback(const sensor_msgs::ImageConstPtr&);
    virtual void aruco_marker_detector();
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    std::string USBCAM_TOPIC;
    // const std::string DEPTH_TOPIC = "/camera/depth/color/image_raw";
    const std::string PUBLISH_TOPIC = "/outlet/coordinate";
    const std::string MG400_CMD_VEL_TOPIC = "/mg400/cmd_vel";
    const std::string MG400STATUS_TOPIC ="/mg400_bringup/msg/RobotStatus";
    const std::string MG400_TOPIC = "/mg400/working";
    const std::string PICKUP_SERVICE_START = "/pickup/start";
    const std::string PICKUP_SERVICE_STOP = "/pickup/stop";
    const std::string ARUCO_DETECT_SERVICE_START = "/arucodetect/start";
    const std::string ARUCO_DETECT_SERVICE_STOP = "/arucodetect/stop";
    std::string mode ="";
    OUTLET_CV();
    ~OUTLET_CV();
    bool getRun(); 
    void setRun(bool run);
    const int max_lowThreshold = 100;
    const std::string window_name = "Edge Map";
    int w,h; //heihgt and width for the first cam
    int u_w, u_h; //height and width for the second cam
    bool Drew = false;
    bool drawing = false;
    bool ADJUST=false;
    double offset_x =0; double offset_y=0; double offset_z=0;
    double fixed_x,fixed_y;//88.0;
    double c_x,c_y;
private:
    bool initial = true;    
    bool RUN = false; 
    double detect_probability =0.0;
    bool detected=false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
    bool Done_x = false; bool Done_y = false;
    bool mg400_running = false;
    int timer = 1.5;
    int offset_x_counter =0;
    int offset_y_counter =0;
    std::string CALIBRATION;
};


OUTLET_CV::OUTLET_CV(){
  
    ros::NodeHandle private_nh("~");
    private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
    private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/camera/aligned_depth_to_color/image_raw"));
    private_nh.param("usbcam_topic", USBCAM_TOPIC, std::string("/usb_cam/color/image"));
    private_nh.param("offset_fixed_x", fixed_x, 161.0);
    private_nh.param("offset_fixed_y", fixed_y, 200.0);
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



// void OUTLET_CV::get_circle(int, void*userdata){
//   //expand the ROI to detect how off the MG400 is
//     //GaussianBlur( dst, dst, Size(9, 9), 2, 2 );
//     geometry_msgs::Twist twist;
//     std::vector<cv::Vec3f> circles;
//     cv::HoughCircles(
//          u_dst,                    // 8ビット，シングルチャンネル，グレースケールの入力画像
//          circles,                // 検出された円を出力.配列の [ 0, 1 ] に円の中心座標. [2] に円の半径が格納される
//          cv::HOUGH_GRADIENT,     // cv::HOUGH_GRADIENT メソッドのみ実装されている.
//          1,                      // 画像分解能に対する出力解像度の比率の逆数
//          30,                     // 検出される円の中心同士の最小距離
//          60,                    // Canny() の大きいほうの閾値.勾配がこのパラメータを超えている場合はエッジとして判定
//          30                      // Canny() の小さいほうの閾値.勾配がこのパラメータを下回っている場合は非エッジとして判定
//          );

//       for (auto circle : circles)
//       {
//           cv::circle(u_dst, cv::Point( circle[0], circle[1] ), circle[2], cv::Scalar(0, 0, 255), 2);
//           offset_x = (double)fixed_x - circle[0];
//           offset_y = (double)fixed_y - circle[1];
//           // printf("\nCircle[0]: %lf, Circle[1]: %lf\n", circle[0], circle[1]);
//           circle_detected = true;
//           ROS_INFO("Circles are detected");
//       }

//       //比率ゲイン
//       double Kp = 0.05;
//       // double Kp_y = 0.01;
//       double Kv = 0.0;
//       //PD control
//       double move_x = Kp*offset_x - Kv*offset_x/1000;
//       double move_y = Kp*offset_y - Kv*offset_y/1000;
//       // if(Done_x ){
//       //   twist.linear.y = 0;
//       //   if (offset_x_counter>3)
//       //     Done_x = true;
//       //   offset_x_counter++;
//       // }else{
        
//       // }
//       // if((std::abs(offset_y)<=0.5  || std::abs(offset_y)>20) ||  Done_y){
//       //   twist.linear.z = 0;
//       //   if (offset_y_counter>3)
//       //     Done_y = true;
//       //   offset_y_counter++;
//       // }else{
//       //   twist.linear.z = move_y;
//       // }
      
//       if (!mg400_running && (fstop-fstart)>timer && (!Done_x || !Done_y)){
//         twist.linear.y = move_x;
//         twist.linear.z = move_y;
//         clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
//         // printf("\nOffset_x: %lf, Offset_y: %lf\n", offset_x, offset_y);
//         // printf("\nlinear.y: %lf, linear.z: %lf\n", twist.linear.y, twist.linear.z);
//         ROS_INFO("\nOffset_x: %lf, Offset_y: %lf\n", offset_x, offset_y);
//         ROS_INFO("\nlinear.y: %lf, linear.z: %lf\n", twist.linear.y, twist.linear.z);
//         cmd_vel_pub.publish(twist);
//         if(std::abs(offset_x)<=0.5){
//              Done_x = true;
//         }
//         if(std::abs(offset_y)<=0.5){
//              Done_y = true;
//         }
//       }
//       clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
//       if(!mg400_running && Done_x && Done_y && (fstop-fstart)>timer && circle_detected){
//         coordinate.t ="F";
//         coordinate.x = 10;
//         coordinate.y = 10;
//         coordinate.z = 10;
//         pub.publish(coordinate);
//       }
        
      
//      /*   try{
//           cv::circle(dst, cv::Point( circles[0][0], circles[0][1] ), circles[0][2], cv::Scalar(0, 0, 255), 2);
//           offset_x = (double)coordinate.x - circles[0][0];
//           offset_y = (double)coordinate.y - circles[0][1];
//           printf("Offset_x: %f, Offset_y: %f", offset_x, offset_y);
//         } 
//         catch (std::exception e)
//         {
//             printf("A circle is not found\n");
//         }
//       */
//       // cv::circle(u_dst, cv::Point(c_x,c_y), 5, cv::Scalar(0, 0, 255),-1);

//       cv::namedWindow("dst", 1);
//       imshow("dst", u_dst);

//       cv::waitKey(3);
// }

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
   cout << "detection starts" << endl;
   RUN = true;
   return RUN;

 }

 bool OUTLET_CV::arucodetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
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

void OUTLET_CV::aruco_marker_detector(){         
    Mat imageCopy;
    u_src.copyTo(imageCopy);
    if(initial)
        aruco::detectMarkers(src, dictionary, corners, ids);
    else
        aruco::detectMarkers(u_src, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0){
        printf("detected\n");
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        c_x = (corners[0][0].x + corners[0][1].x)/2;
        c_y = (corners[0][0].y + corners[0][3].y)/2;
        if(initial){
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
            pub.publish(coordinate);
            // I should put actionlib here 
            initial = false;
        }
        
    }
    if(!initial){
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs, rvecs, tvecs);
        for(int i=0; i < ids.size(); i++)
        {
        cv::drawFrameAxes(imageCopy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);

        // This section is going to print the data for all the detected
        // markers. If you have more than a single marker, it is
        // recommended to change the below section so that either you
        // only print the data for a specific marker, or you print the
        // data for each marker separately.
        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4)
                            << "x: " << std::setw(8) << tvecs[0](0);
        cv::putText(imageCopy, vector_to_marker.str(),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 252, 124), 1, CV_AVX);

        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4)
                            << "y: " << std::setw(8) << tvecs[0](1);
        cv::putText(imageCopy, vector_to_marker.str(),
                    cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 252, 124), 1, CV_AVX);

        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4)
                            << "z: " << std::setw(8) << tvecs[0](2);
        cv::putText(imageCopy, vector_to_marker.str(),
                    cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 252, 124), 1, CV_AVX);
        }        
        cv::resizeWindow("out", 640,480);
        imshow("out", imageCopy);
        waitKey(3);
    }

}


int main( int argc, char** argv )
{

   ros::init(argc, argv, "mask_detector");
   OUTLET_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(30);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &OUTLET_CV::image_callback, &cc);
   cc.usbcam_sub = cc.nh.subscribe(cc.USBCAM_TOPIC, 1000, &OUTLET_CV::usbcam_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 1000, &OUTLET_CV::depth_callback, &cc);
   cc.mg400_status = cc.nh.subscribe(cc.MG400STATUS_TOPIC,1000, &OUTLET_CV::mg400_status_callback, &cc);
//    cc.darknet_bbox_sub = cc.nh.subscribe(cc.BBOX_TOPIC, 1000, &OUTLET_CV::bbox_callback, &cc);
   cc.pickup_start = cc.nh.advertiseService(cc.ARUCO_DETECT_SERVICE_START, &OUTLET_CV::arucodetect_start_service, &cc);
   cc.pickup_stop = cc.nh.advertiseService(cc.ARUCO_DETECT_SERVICE_STOP, &OUTLET_CV::arucodetect_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 100);
   cc.cmd_vel_pub = cc.nh.advertise<geometry_msgs::Twist>(cc.MG400_CMD_VEL_TOPIC,100);
   cc.mg400_sub = cc.nh.subscribe(cc.MG400_TOPIC,1000, &OUTLET_CV::mg400_callback, &cc);   
   std_srvs::Empty _emp;
   clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
   clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
   while(ros::ok()){
      // cout << cc.getRun() << endl;
      if(!cc.src.empty() && !cc.u_src.empty()){
        if(cc.getRun()){
            cc.aruco_marker_detector();
        }
      imshow("src", cc.src);
      waitKey(3);      
      }
      ros::spinOnce();
      loop_rate.sleep();
   }
  destroyAllWindows();
  return 0;
}
