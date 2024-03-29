 #include <ros/ros.h>

 // Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
//  #include <opencv2/core/types.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <pthread.h>
 #include <geometry_msgs/Twist.h>
 #include <std_srvs/Empty.h>
 #include <std_msgs/String.h>
#include <std_msgs/Int8.h>
 #include <sensor_msgs/LaserScan.h>
 #include <vector>
 #include <camera_pkg_msgs/Coordinate.h>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define NUM_THREADS 4
 #define RANGE_MAX 5.6;
  // OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
static const std::string PUBLISH_TOPIC = "/image_converter/output_video";
static const std::string SCAN_TOPIC ="/scan";
static const std::string QRSTATUS_TOPIC ="/visp_auto_tracker/status";
struct timespec fps_start, fps_stop, interval_start, interval_stop;
double fstart, fstop, istart, istop;
class LINETRACE{
 
 public:
    //init node
    ros::NodeHandle nh;
    // Publisher
    ros::Publisher cmd_vel_pub, ditance_pub, message_pub;
    ros::Subscriber rgb_sub, scan_sub, qr_sub;
    cv::Mat ir;
    ros::ServiceServer linetrace_start, linetrace_stop;
    ros::ServiceClient mg400_work_start, mg400_work_stop;
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void qrstatus_callback(const std_msgs::Int8::ConstPtr& msg);
    double null_check(double target);
    std::vector<double> meanWithoutInf(std::vector<double> vec);
    virtual bool linetrace_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool linetrace_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    const std::string LINETRACE_SERVICE_START = "/linetrace/start";
    const std::string LINETRACE_SERVICE_STOP = "/linetrace/stop";
    const std::string MG400_PICKUP_SERVICE_START = "/pickup/start";
    const std::string MG400_PICKUP_SERVICE_STOP = "/pickup/stop";
    const std::string DISTANCE_TOPIC = "/linetrace/distance";
    
    bool RUN = false;
    bool STOP = false;
    bool QR =false;
    bool MG_WORK =false;
    double velocity =0.2;
    double angular = 0.0;
    const int kernel_size = 3;
    int lowThreshold = 100;
    const int ratio = 3;
    std_srvs::Empty _emp;
    sensor_msgs::LaserScan _scan;
    std::vector<double> stop_threashold;
    LINETRACE();
    ~LINETRACE();
};

LINETRACE::LINETRACE(){

}

LINETRACE::~LINETRACE(){
  
}


 bool LINETRACE::linetrace_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  //  cout << "start calibration" << endl;
   RUN = true;
   return RUN;

 }

 bool LINETRACE::linetrace_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  //  cout << "stop calibration" << endl;
   RUN = false;
   return RUN;
 }


struct arg_struct {
    int index;
    // int num_threads;
    int from;
    int to;
    cv::Mat masked;
};


void *makeBlack(void *arguments){
  int idx = ((struct arg_struct*)arguments)->index;
  int start = ((struct arg_struct*)arguments)->from;
  int end = ((struct arg_struct*)arguments)->to;
  cv::Mat masked= ((struct arg_struct*)arguments)->masked;
  int fheight = masked.cols, fwidth = masked.rows;
  int height_start = (idx*(fheight/NUM_THREADS))+1;
  int height_end = (idx*(NUM_THREADS)) + (fheight/NUM_THREADS);
  // for(int i = height_end; i<height_end; i++){
  //   for(int j = 0; j<fwidth; j++){
  //    cv::Vec3b &color = masked.at<cv::Vec3b>(cv::Point(j,i)); 
  //       color.val[0] = 0;
  //       color.val[1] = 0;
  //       color.val[2] = 0;
  //   }
  // }

}

std::vector<double> LINETRACE::meanWithoutInf(std::vector<double> vec){
        std::vector<double> result;
        if(!vec.empty()){
         for (int i = 0; i < vec.size(); i++)
         {
             if(vec[i]<10 && vec[i] > 0.4){
                 result.push_back(vec[i]);
             }
         }
        }
        return result;
    }
double LINETRACE::null_check(double target){
      if(!(target >0)){
          target=(double)RANGE_MAX;
      }

      return target;
  }


void LINETRACE::qrstatus_callback(const std_msgs::Int8::ConstPtr& msg){
   if(msg->data==1){
     QR=false;
   }else{
     QR=true;
   }
}


void LINETRACE::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

      try
        {
            double center_number = (-msg->angle_min)/msg->angle_increment;
            double angle_min = (msg->angle_min)/msg->angle_increment;
            double angle_max = (msg->angle_max)/msg->angle_increment;
            double center=msg->ranges[179];
//             double center=msg->ranges[angle_min];
            std_msgs::String msg_data;
            double index=180;
            //40 degree front
            if (center <=0) center =5.0;
            for(int i=140; i<220; i++){
              double temp = msg->ranges[i];
              double temp_cent = center;
              if(temp <=0) temp =100.0;
              center = std::min(std::min(center, temp), 5.0);
              if(center < temp_cent) index = i;
            }
	   
            if(center<=0.35){
                stop_threashold.push_back(1);
	    }else if(center<=0.5){
                velocity =0.1;
		
            }else{
                stop_threashold.clear();
                velocity =0.2;
		if (STOP){
	          STOP = false;
		  RUN=true;
		}
		
            }
           
            if(!MG_WORK && (stop_threashold.size()>5) && QR){
                clock_gettime(CLOCK_MONOTONIC, &interval_start); istart=(double)interval_start.tv_sec + ((double)interval_start.tv_nsec/1000000000.0);
                RUN=false;
                MG_WORK =true;
		sleep(2);
                mg400_work_start.call(_emp);
            }else if(stop_threashold.size()>2){
	    	RUN=false;
		STOP = true;
	    }
		    
            std::stringstream _center;
            _center << " center: " << center << " index: " << index;
             msg_data.data = _center.str();
             message_pub.publish(msg_data);

            

        }
        catch(const std::exception& e)
        {
           ROS_ERROR("exception: %s", e.what());
//            std::cout << e.what() <<std::endl;
        }
        if(MG_WORK)
         clock_gettime(CLOCK_MONOTONIC, &interval_stop); istop=(double)interval_stop.tv_sec + ((double)interval_stop.tv_nsec/1000000000.0);
        //comunicate with MG400 would be better 
        if(MG_WORK && (istop-istart)>30){
            MG_WORK=false;
            RUN=true;
            mg400_work_stop.call(_emp);
        }
}



void LINETRACE::image_callback(const sensor_msgs::ImageConstPtr& msg){
   clock_gettime(CLOCK_MONOTONIC, &fps_start); fstart=(double)fps_start.tv_sec + ((double)fps_start.tv_nsec/1000000000.0);
   std_msgs::Header msg_header = msg->header;
   geometry_msgs::Twist cmd_msg;
   std::string frame_id = msg_header.frame_id.c_str();
   cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);

   ROS_INFO_STREAM("New Image from " << frame_id);

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
   
   int low_c[3] = {14, 131, 189};
   int high_c[3] ={34, 151, 269};
   cv::Mat frame = cv_ptr->image, frame_HSV, mask; 
   cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
   int fheight = frame.size().height, fwidth = frame.size().width;
   int search_top = (fheight/4)*3;
   int search_bot = search_top + 20;




   //erase unnessesary pixles by blacking

   // -------------- use pthread but not working yet -----------//
   /*
   pthread_t threads[NUM_THREADS];
   int rc;
   int i;
   for(i=0; i<NUM_THREADS; i++){
     struct arg_struct *args = (struct arg_struct *)malloc(sizeof(struct arg_struct));
     std::cout << "cretae a thread no." << i << std::endl;
     args->index =i;
     args->from = search_top;
     args->to = search_bot;
     args->masked = frame_HSV;
     rc = pthread_create(&threads[i], NULL, makeBlack, args);

     if(rc){
       std::cout << "Error: unable to create thread, " << rc <<std::endl;
       exit(-1);
     }
   }
   for(int i=0; i<NUM_THREADS;i++)
         pthread_join(threads[i], NULL);
  */
  for(int i = 0; i<search_top; i++){
    for(int j = 0; j<fwidth; j++){
     cv::Vec3b &color = frame_HSV.at<cv::Vec3b>(cv::Point(j,i)); 
        color.val[0] = 0;
        color.val[1] = 0;
        color.val[2] = 0;
    }
  }

      //mask the image in yellow
   cv::inRange(frame_HSV, cv::Scalar(low_c[0],low_c[1],low_c[1]), cv::Scalar(high_c[0],high_c[1],high_c[2]),mask);
   blur( mask, mask, cv::Size(3,3) );
   Canny( mask, mask, lowThreshold, lowThreshold*ratio, kernel_size );
   cv::Moments M = cv::moments(mask); // get the center of gravity
   if (M.m00 >0){
        int cx = int(M.m10/M.m00); //重心のx座標
        int cy = int(M.m01/M.m00); //重心のy座標

      cv::circle(frame, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
      double err = (double)cx - (double)(fwidth/2);  //黄色の先の重心座標(x)と画像の中心(x)との差
      cmd_msg.linear.x =velocity;
      angular = -(double)(err/400);
      cmd_msg.angular.z = angular;
   }else{
      cmd_msg.linear.x =0.0;
      cmd_msg.angular.z = 0.0;
   }
   if(RUN) cmd_vel_pub.publish(cmd_msg);
   clock_gettime(CLOCK_MONOTONIC, &fps_stop); fstop=(double)fps_stop.tv_sec + ((double)fps_stop.tv_nsec/1000000000.0);
   std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));
        cv::putText(frame, //target image
          fps, //text
          cv::Point(10, 30), //top-left position
          cv::FONT_HERSHEY_DUPLEX,
          1.0,
          cv::Scalar(118, 185, 0), //font color
          2);
   cv::imshow("original", frame);
   cv::imshow("mask", mask);
   cv::waitKey(3);
}


 int main(int argc, char** argv)
 {
   // Initialize the ROS Node "roscpp_example"
   ros::init(argc, argv, "roscpp_example");
   LINETRACE lt;
   // Instantiate the ROS Node Handler as nh
   

   // Print "Hello ROS!" to the terminal and ROS log file
   ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());
   lt.rgb_sub = lt.nh.subscribe(IMAGE_TOPIC, 1000, &LINETRACE::image_callback, &lt);
   lt.scan_sub = lt.nh.subscribe(SCAN_TOPIC, 1000, &LINETRACE::scan_callback, &lt);
   lt.qr_sub = lt.nh.subscribe(QRSTATUS_TOPIC, 1000, &LINETRACE::qrstatus_callback, &lt);
   lt.message_pub = lt.nh.advertise<std_msgs::String>("/scan/angle", 1000);
   lt.linetrace_start = lt.nh.advertiseService(lt.LINETRACE_SERVICE_START, &LINETRACE::linetrace_start_service, &lt);
   lt.linetrace_stop =lt.nh.advertiseService(lt.LINETRACE_SERVICE_STOP, &LINETRACE::linetrace_stop_service, &lt);
   lt.mg400_work_start = lt.nh.serviceClient<std_srvs::Empty>(lt.MG400_PICKUP_SERVICE_START);
   lt.mg400_work_stop = lt.nh.serviceClient<std_srvs::Empty>(lt.MG400_PICKUP_SERVICE_STOP);
   
   // Program succesful
   ros::spin();
   return 0;
 }
