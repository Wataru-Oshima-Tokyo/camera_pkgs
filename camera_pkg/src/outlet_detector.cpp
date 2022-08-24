 #include <ros/ros.h>

 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>


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
    Mat src, src_hsv, ROI, dst, mask; //for the first camera
    Mat u_src, u_dst, u_ROI; // for the second camera
    Mat depth; //for the depth cammera
    ros::Publisher pub, cmd_vel_pub;
    ros::Subscriber image_sub, depth_sub, mg400_sub, usbcam_sub, mg400_status;
    ros::NodeHandle nh;
    camera_pkg_msgs::Coordinate coordinate;
    ros::ServiceServer pickup_start, pickup_stop;
    int lowThreshold;
    int ix,iy,cx,cy;
    int low_c[3] = {50, 138, 157};
    int high_c[3] = {100, 255, 197};
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> contour;
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
    void CannyThreshold(int, void*);
    void MaskThreshold(int, void*);
    void DrawCircle(int, void*);
    void makeRegion(int, void*);
    void get_circle(int, void*);

//     void detect_object(int , void* userdata);
    void mouseEvent(int event, int x, int y, int flags, void* userdata);
    void draw_region_of_interest(int event, int x, int y, int flags, void* userdata);
    void get_hsv(int event, int x, int y, int flags, void* userdata);
    // Mat getDepth();
    const std::string SRC_WINDOW = "src";
    const std::string ROI_WINDOW = "roi";
    virtual bool maskdetect_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool maskdetect_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void mg400_status_callback(const mg400_bringup::RobotStatus&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    virtual void mg400_callback(const std_msgs::Bool&);
    virtual void usbcam_callback(const sensor_msgs::ImageConstPtr&);
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    std::string USBCAM_TOPIC;
    // const std::string DEPTH_TOPIC = "/camera/depth/color/image_raw";
    const std::string PUBLISH_TOPIC = "/outlet/coordinate";
    const std::string MG400_CMD_VEL_TOPIC = "/MG400/cmd_vel";
    const std::string MG400STATUS_TOPIC ="/mg400_bringup/msg/RobotStatus";
    const std::string MG400_TOPIC = "/mg400/working";
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
    int w,h; //heihgt and width for the first cam
    int u_w, u_h; //height and width for the second cam
    bool Drew = false;
    bool drawing = false;
    bool ADJUST=false;
    double offset_x =0; double offset_y=0; double offset_z=0;
    double fixed_x = 161.0; double fixed_y = 88.0;
    double c_x,c_y;
private:
    bool RUN = false; 
    double detect_probability =0.0;
    bool detected=false;
    bool start_call = true;
    bool stop_call = false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
    bool Done_x = false; bool Done_y = false;
    bool mg400_running = false;
    int timer = 1.5;
    int offset_x_counter =0;
    int offset_y_counter =0;
};


OUTLET_CV::OUTLET_CV(){
  
  ros::NodeHandle private_nh("~");
  private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
  private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/camera/aligned_depth_to_color/image_raw"));
  private_nh.param("usbcam_topic", USBCAM_TOPIC, std::string("/usb_cam/color/image"));
  lowThreshold = 6;
};

OUTLET_CV::~OUTLET_CV(){};

bool OUTLET_CV::getRun(){    
  return RUN;
}

void OUTLET_CV::setRun(bool run){
    RUN = run;
}

int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function


void OUTLET_CV::get_circle(int, void*userdata){
  //expand the ROI to detect how off the MG400 is
    //GaussianBlur( dst, dst, Size(9, 9), 2, 2 );
    geometry_msgs::Twist twist;
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
         u_dst,                    // 8ビット，シングルチャンネル，グレースケールの入力画像
         circles,                // 検出された円を出力.配列の [ 0, 1 ] に円の中心座標. [2] に円の半径が格納される
         cv::HOUGH_GRADIENT,     // cv::HOUGH_GRADIENT メソッドのみ実装されている.
         1,                      // 画像分解能に対する出力解像度の比率の逆数
         30,                     // 検出される円の中心同士の最小距離
         60,                    // Canny() の大きいほうの閾値.勾配がこのパラメータを超えている場合はエッジとして判定
         30                      // Canny() の小さいほうの閾値.勾配がこのパラメータを下回っている場合は非エッジとして判定
         );

      for (auto circle : circles)
      {
          cv::circle(u_dst, cv::Point( circle[0], circle[1] ), circle[2], cv::Scalar(0, 0, 255), 2);
          offset_x = (double)fixed_x - circle[0];
          offset_y = (double)fixed_y - circle[1];
          // printf("\nCircle[0]: %lf, Circle[1]: %lf\n", circle[0], circle[1]);
      }

      //比率ゲイン
      double Kp = 0.05;
      double Kv = 0.0;
      //PD control
      double move_x = Kp*offset_x - Kv*offset_x/1000;
      double move_y = Kp*offset_y - Kv*offset_y/1000;
      // if(Done_x ){
      //   twist.linear.y = 0;
      //   if (offset_x_counter>3)
      //     Done_x = true;
      //   offset_x_counter++;
      // }else{
        
      // }
      // if((std::abs(offset_y)<=0.5  || std::abs(offset_y)>20) ||  Done_y){
      //   twist.linear.z = 0;
      //   if (offset_y_counter>3)
      //     Done_y = true;
      //   offset_y_counter++;
      // }else{
      //   twist.linear.z = move_y;
      // }
      
      if (!mg400_running && (fstop-fstart)>timer && (!Done_x || !Done_y)){
        twist.linear.y = move_x;
        twist.linear.z = move_y;
        clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
        printf("\nOffset_x: %lf, Offset_y: %lf\n", offset_x, offset_y);
        printf("\nlinear.y: %lf, linear.z: %lf\n", twist.linear.y, twist.linear.z);
        cmd_vel_pub.publish(twist);
        if(std::abs(offset_x)<=0.5){
             Done_x = true;
        }
        if(std::abs(offset_y)<=0.5){
             Done_y = true;
        }
      }
      clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
      if(!mg400_running && Done_x && Done_y && (fstop-fstart)>timer){
        coordinate.t ="F";
        coordinate.x = 10;
        coordinate.y = 10;
        coordinate.z = 10;
        pub.publish(coordinate);
      }
        
      
     /*   try{
          cv::circle(dst, cv::Point( circles[0][0], circles[0][1] ), circles[0][2], cv::Scalar(0, 0, 255), 2);
          offset_x = (double)coordinate.x - circles[0][0];
          offset_y = (double)coordinate.y - circles[0][1];
          printf("Offset_x: %f, Offset_y: %f", offset_x, offset_y);
        } 
        catch (std::exception e)
        {
            printf("A circle is not found\n");
        }
      */
      // cv::circle(u_dst, cv::Point(c_x,c_y), 5, cv::Scalar(0, 0, 255),-1);

      cv::namedWindow("dst", 1);
      imshow("dst", u_dst);

      cv::waitKey(3);
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

void OUTLET_CV::MaskThreshold(int, void*userdata){
   OUTLET_CV *cc = (OUTLET_CV*)userdata;
   cv::inRange(src_hsv, cv::Scalar(low_c[0],low_c[1],low_c[2]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
   printf("made a mask\n");
//    Canny(mask, mask, lowThreshold, lowThreshold*ratio, kernel_size );
   cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  //   printf("got contours\n");
   contour = contours[getMaxAreaContourId(contours)];
  //  printf("got a contour\n");
  //  cv::drawContours(mask, contour, 1, 255);
  //  printf("drew the contour\n");
   cv::Moments M = cv::moments(mask); // get the center of gravity
   printf("got the contour\n");
   if (M.m00 >0){
   		  cc->c_x = int(M.m10/M.m00); //重心のx座標
   		  cc->c_y = int(M.m01/M.m00); //重心のy座標
      std::cout << "Momentum " <<c_x << " " << c_y <<std::endl;
      std::vector<double> z_array;
      double z=0.0;
      cv::circle(cc->ROI, cv::Point(cc->c_x,cc->c_y), 5, cv::Scalar(0, 0, 255),-1);
      rep(i,0,5)
        rep(j,0,5){
          z = depth.at<uint16_t>((uint16_t)(c_y+j),(uint16_t)(c_x+i));
          z_array.push_back(z);
        }
        std::sort(z_array.begin(), z_array.end());
        z = z_array[z_array.size()-1]; 
        z = cc->depth.at<uint16_t>((uint16_t)(c_y),(uint16_t)(c_x));
        cc->coordinate.t = "L";
	cc->coordinate.x = c_x;
        cc->coordinate.y = c_y;
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
   rep(i,0,w){
    rep(j,0,h){
        if((j>=0 && j<=iy) || (i>=0 && i<ix) || (i>cx && i<w) ||(j>cy)){
          cv::Vec3b &color = ROI.at<cv::Vec3b>(j,i); 
          color.val[0] = 0;
          color.val[1] = 0;
          color.val[2] = 0;
        }
      }
    }
   cvtColor(ROI, src_hsv, COLOR_BGR2HSV);
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
    ROI =src;
    cvtColor(src, dst, COLOR_BGR2GRAY);
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
    u_ROI =u_src;
    u_w = u_src.size().width;
    u_h = u_src.size().height;
    //if aruco is used, the ROI should be removed.
    rep(i,0,u_w){
      rep(j,0,u_h){
          if((j>=0 && j<=u_h/2-70) || (i>=0 && i<u_w/2-50) || (i>u_w/2+50 && i<u_w) ||(j>u_h/2+30)){
            cv::Vec3b &color = u_ROI.at<cv::Vec3b>(j,i); 
            color.val[0] = 0;
            color.val[1] = 0;
            color.val[2] = 0;
          }
        }
    }
    cvtColor(u_ROI, u_dst, COLOR_BGR2GRAY);

    // cv::line(u_dst, cv::Point(0, u_h/2), cv::Point(u_w,u_h/2), cv::Scalar(0,255,255),2,4);
    // cv::line(u_dst, cv::Point(u_w/2, 0), cv::Point(u_w/2,u_h), cv::Scalar(0,255,255),2,4);

    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

 }

void get_hsv(int event, int x, int y, int flags, void* userdata){
  OUTLET_CV *cc = (OUTLET_CV*)userdata;
  if (event == EVENT_LBUTTONDOWN )
     {
      Vec3b &color = cc->src_hsv.at<Vec3b>(y,x);
      // std::cout << color[0] << " " << color[1] << " " << color[2] << std::endl;
      cc->low_c[0] = color[0] -10; cc->low_c[1] = color[1] -10; cc->low_c[2] = color[2] -40;
      cc->high_c[0] = color[0] +10; cc->high_c[1] = color[1] +10; cc->high_c[2] = color[2] +40;
      // ROS_INFO_STREAM("The MIN color: %d, %d, %d", low_c[0],low_c[1],low_c[2]);
      // ROS_INFO_STREAM("The MAX color: %d, %d, %d", high_c[0],high_c[1],high_c[2]);
      printf("The MIN color: %d, %d, %d\n", cc->low_c[0],cc->low_c[1],cc->low_c[2]);
      printf("The MAX color: %d, %d, %d\n", cc->high_c[0],cc->high_c[1],cc->high_c[2]);	
      cc->MaskThreshold(0,cc);
	 }
  
}

 

void draw_region_of_interest(int event, int x, int y, int flags, void* userdata)
{
     OUTLET_CV *cc = (OUTLET_CV*)userdata;
    //  ros::Publisher* _pub = cc->pub;
    //  _cc.pub = _cc.nh.advertise<std_msgs::String>(_cc.PUBLISH_TOPIC, 1000);
     if(event == EVENT_LBUTTONDOWN){
      cc->drawing =!cc->drawing;
      if(cc->drawing){
        cc->ix = x; cc->iy=y;
      }else{
        cv::rectangle(cc->src, cv::Point(cc->ix,cc->iy), cv::Point(x,y), cv::Scalar(0,255,255),2,4);
        cc->cx = x; cc->cy = y;
        cc->Drew = true;
        std::cout << cc->ix << " "
        << cc->iy << " "
        << cc->cx << " "
        << cc->cy << std::endl;
        cv::destroyAllWindows(); 
      }
     }else if(event == EVENT_MOUSEMOVE){
      if(cc->drawing){
        cv::rectangle(cc->src, cv::Point(cc->ix,cc->iy), cv::Point(x,y),cv::Scalar(0,255,0),-1,4);
      }
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
   cc.pickup_start = cc.nh.advertiseService(cc.PICKUP_SERVICE_START, &OUTLET_CV::maskdetect_start_service, &cc);
   cc.pickup_stop = cc.nh.advertiseService(cc.PICKUP_SERVICE_STOP, &OUTLET_CV::maskdetect_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 100);
   cc.cmd_vel_pub = cc.nh.advertise<geometry_msgs::Twist>(cc.MG400_CMD_VEL_TOPIC,100);
   cc.mg400_sub = cc.nh.subscribe(cc.MG400_TOPIC,1000, &OUTLET_CV::mg400_callback, &cc);   
   std_srvs::Empty _emp;
   
   cv::namedWindow(cc.SRC_WINDOW,WINDOW_AUTOSIZE);
   setMouseCallback(cc.SRC_WINDOW, draw_region_of_interest, &cc);
   clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);
   clock_gettime(CLOCK_MONOTONIC, &timer_stop); fstop=(double)timer_stop.tv_sec + ((double)timer_stop.tv_nsec/1000000000.0);
   while(ros::ok()){
      // cout << cc.getRun() << endl;
      if(!cc.src.empty() && !cc.u_src.empty()){
          if(!cc.Drew){
              imshow(cc.SRC_WINDOW, cc.src);
          }else{
            cv::rectangle(cc.ROI, cv::Point(cc.ix,cc.iy),  cv::Point(cc.cx,cc.cy), cv::Scalar(0,255,255), 2,4);
            //make the region of interest
            cc.makeRegion(0, &cc);
            cv::namedWindow(cc.ROI_WINDOW,WINDOW_AUTOSIZE);
            cv::setMouseCallback(cc.ROI_WINDOW, get_hsv,&cc);
	    if (cc.ADJUST)
            	cc.get_circle(0, &cc);
            imshow(cc.ROI_WINDOW, cc.ROI);
            // imshow("hsv", cc.src_hsv);
          }
      waitKey(3);      
      }
      ros::spinOnce();
      loop_rate.sleep();
   }
  destroyAllWindows();
  return 0;
}
