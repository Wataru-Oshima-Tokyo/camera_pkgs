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

class CAMERA_CV{
  public:
    Mat src, src_gray, src_hsv, dst, detected_edges, mask;
    Mat depth;
    ros::Publisher pub;
    ros::Subscriber image_sub, depth_sub;
    ros::NodeHandle nh;
    ros::ServiceServer imshow_start, imshow_stop;
    ros::ServiceClient calibration_start, calibration_stop;
    int lowThreshold;
    // int low_c[3] = {17, 123, 121};
    // int high_c[3] ={37, 143, 201};
    int low_c[3] = {0, 0, 0};
    int high_c[3] = {0, 0, 0};
    bool calibration = false;
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
    void CannyThreshold(int, void*);
    void MaskThreshold(int, void*);
    void DrawCircle(int, void*);
    void mouseEvent(int event, int x, int y, int flags, void* userdata);
    camera_pkg_msgs::Coordinate MaskThreshold(int, int);
    // Mat getDepth();
    const std::string OPENCV_WINDOW = "Image window";
    virtual bool calibration_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool calibration_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    // const std::string DEPTH_TOPIC = "/camera/depth/color/image_raw";
    const std::string PUBLISH_TOPIC = "/camera_pkg/coordinate";
    const std::string IMSHOW_SERVICE_START = "/imshow/start";
    const std::string IMSHOW_SERVICE_STOP = "/imshow/stop";
    const std::string CALIB_SERVICE_START = "/calibration/start";
    const std::string CALIB_SERVICE_STOP = "/calibration/stop";
    std::string mode="L";
    CAMERA_CV();
    ~CAMERA_CV();
    bool getRun(); 
    const int max_lowThreshold = 100;
    const std::string window_name = "Edge Map";
    std::string status ="";
private:
    bool RUN = false;
    bool start_call = true;
    bool stop_call = false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
};


CAMERA_CV::CAMERA_CV(){
  ros::NodeHandle private_nh("~");
  private_nh.param("image_topic", IMAGE_TOPIC, std::string("/color/image_rect_raw"));
  private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/depth/image_rect_raw"));
  lowThreshold = 6;
};

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

camera_pkg_msgs::Coordinate CAMERA_CV::MaskThreshold(int x, int y){
  camera_pkg_msgs::Coordinate rvalue;
  rvalue.t="e";
  Vec3b &color = src_hsv.at<Vec3b>(Point(y,x));
  // 			upper=  np.array([pixel[0] + 10, pixel[1] + 10, pixel[2] + 40])
	// lower=  np.array([pixel[0] -10, pixel[1] -10, pixel[2] -40])
  rep(i,0,3){
    int val=10;
    if (i==2){
      val=40;
    }
    low_c[i] = color[i] -val;
    high_c[i] = color[i] +val;
  }

   cv::inRange(src_hsv, cv::Scalar(low_c[0],low_c[1],low_c[2]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
   
   cv::Moments M = cv::moments(mask); // get the center of gravity
   if (M.m00 >0){
   			int cx = int(M.m10/M.m00); //the center of mass for x
   			int cy = int(M.m01/M.m00); //the cneter of mass for y
      
      cv::circle(src, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
      rvalue.t="f";
      rvalue.x = cx;
      rvalue.y = cy;
      return rvalue;
   }
   return rvalue;

}



// void CAMERA_CV::CannyThreshold(int, void*)
// {
//     // cout << "start canny threashold" <<endl;
//     // removeing the noise and ap since the kernel size is 3, set the size 3 by 3
//     blur(mask, detected_edges, Size(3,3) );
//     // //detecing an edege by Canny with the lowThreashold and maxThreshold which is 3 times thant the lower one.
//     Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
//     // //set the dist image all black so that you can put the deteceted edges on the black background
//     dst = Scalar::all(0);
//     // src.copyTo( dst, detected_edges);
//     imshow( "Edge Map", detected_edges);
//     waitKey(3);
// }

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


void CAMERA_CV::depth_callback(const sensor_msgs::ImageConstPtr& msg){
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

 void CAMERA_CV::image_callback(const sensor_msgs::ImageConstPtr& msg){
    clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
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



void mouseEvent(int event, int x, int y, int flags, void* userdata)
{
     CAMERA_CV *cc = (CAMERA_CV*)userdata;
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
      bool clicked= false;
     if  ( event == EVENT_LBUTTONDOWN )
     {
	  	cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
      cc->mode = "L";
      clicked = true;
// 		z = cc->depth.at<u_int16_t>(x,y);
// 	  }
         
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
          cc->mode = "R";
          clicked = true;
          cc->calibration = true;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ", " << z << ")" << endl;
          cc->mode = "M";
          clicked = true;
          if (cc->calibration)
            cc->calibration = false;
          else
            cc->calibration = true;
     }
     if(clicked){
       if(z>0 && z <1200){
          coordinate.t = cc->mode;
          coordinate.x = x;
          coordinate.y = y;
          coordinate.z = z;
          cc->pub.publish(coordinate);

       }else{
         cout << "z value is not valid please try again." << endl;
       }

     }

}


int main( int argc, char** argv )
{

   ros::init(argc, argv, "work_with_camera_start");
   CAMERA_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(20);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &CAMERA_CV::image_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 1000, &CAMERA_CV::depth_callback, &cc);
   cc.imshow_start = cc.nh.advertiseService(cc.IMSHOW_SERVICE_START, &CAMERA_CV::calibration_start_service, &cc);
   cc.imshow_stop = cc.nh.advertiseService(cc.IMSHOW_SERVICE_STOP, &CAMERA_CV::calibration_stop_service, &cc);
   cc.calibration_start = cc.nh.serviceClient<std_srvs::Empty>(cc.CALIB_SERVICE_START);
   cc.calibration_stop = cc.nh.serviceClient<std_srvs::Empty>(cc.CALIB_SERVICE_STOP);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 1000);
   std_srvs::Empty _emp;
   while(ros::ok()){
      // cout << cc.getRun() << endl;
       
       
      if(cc.getRun()){
            cc.DrawCircle(0,0);
      }
      if(!cc.src.empty()){
        setMouseCallback("src", mouseEvent, &cc);
        clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));
        
        std::string explain ="Left clip for robot coordinate: right for image coordinate: center for finishing calibrtion";
        std::string cmd_exp="L:Move R: xy_calibration M: z_calibration";
        if(cc.mode =="L"){
            if(!cc.calibration)
              cc.status ="Executing";
        }else if (cc.mode == "R"){
            cc.status ="xy_calibration";
        }else if (cc.mode == "M"){
            cc.status ="z_calibration";
        }
        if(cc.calibration){
          putText(cc.src, status, Point(10, 30), FONT_HERSHEY_DUPLEX,1.0,Scalar(0, 0, 255), 2);
          putText(cc.src, explain, Point(10, 55), FONT_HERSHEY_DUPLEX,0.7, Scalar(118, 185, 0), 1);
        }else{
            putText(cc.src, fps, Point(10, 30), FONT_HERSHEY_DUPLEX, 1.0,Scalar(118, 185, 0), 2);
            putText(cc.src, status, Point(10, 55), FONT_HERSHEY_DUPLEX, 0.7,Scalar(0, 0, 255), 2);
            putText(cc.src, cmd_exp, Point(10, 70), FONT_HERSHEY_DUPLEX,0.7,Scalar(0, 255, 255), 2);
        }
        
        

      
        imshow( "src", cc.src);
        waitKey(3);
      }

      ros::spinOnce();
      loop_rate.sleep();
   }
  destroyAllWindows();
  return 0;
}
