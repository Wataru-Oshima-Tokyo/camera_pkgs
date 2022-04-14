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
 #include "std_srvs/Empty.h"
 #include <vector>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define NUM_THREADS 4
  // OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
static const std::string PUBLISH_TOPIC = "/image_converter/output_video";
static const std::string IR_TOPIC ="/camera/ir/image";
class LINETRACE{
 
 public:
    //init node
    ros::NodeHandle nh;
    // Publisher
    ros::Publisher cmd_vel_pub, ditance_pub;
    cv::Mat ir;
    ros::ServiceServer linetrace_start, linetrace_stop;
    ros::ServiceClient mg400_work_start, mg400_work_stop;
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void ir_callback(const sensor_msgs::ImageConstPtr& msg);
    virtual bool linetrace_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool linetrace_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    const std::string LINETRACE_SERVICE_START = "/linetrace/start";
    const std::string LINETRACE_SERVICE_STOP = "/linetrace/stop";
    const std::string MG400_PICKUP_SERVICE_START = "/mg400/pickup_start";
    const std::string MG400_PICKUP_SERVICE_STOP = "/mg400/pickup_stop";
    const std::string DISTANCE_TOPIC = "/linetrace/distance";
    bool RUN = false;
    bool MG_WORK =false;
    std_srvs::Empty _emp;
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

void LINETRACE::ir_callback(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);
    camera_pkg::Coordinate _distance;
    ditance_pub = nh.advertise<camera_pkg::Coordinate>(DISTANCE_TOPIC, 10, true);
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

    ir = cv_ptr->image;
    std::vector<int> z_arr;
    int fheight = ir.size().height, fwidth = ir.size().width;

    // only see the front of robot
    for(int i = fwidth/3 ; i<(2*fwidth)/3; i++){
      for(int j = fheight/3; j<(2*fheight)/3; j++){
          int z = ir.at<uint16_t>((uint16_t)j,(uint16_t)i);
          z_arr.push_back(z);
      }
    }
    // get the mean of z_arr
    int z = z_arr[z_arr.size()/2];
    _distance.z =z;
    //start the picking behavior
    if(z<100 && RUN){
      MG_WORK =true;
      if(MG_WORK && RUN){
        mg400_work_start.call(_emp); //calling the mg400_work_start service
      }
      RUN=false;
    }else{
      RUN=true;
      if(MG_WORK && RUN){
        mg400_work_stop.call(_emp); //calling the mg400_work_stop service
      }
      MG_WORK=false;
    }
    ditance_pub.publish(_distance);

}



void LINETRACE::image_callback(const sensor_msgs::ImageConstPtr& msg){
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
   
   int low_c[3] = {17, 123, 121};
   int high_c[3] ={37, 143, 201};
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
   
   cv::Moments M = cv::moments(mask); // get the center of gravity
   if (M.m00 >0){
        int cx = int(M.m10/M.m00); //重心のx座標
        int cy = int(M.m01/M.m00); //重心のy座標

      cv::circle(frame, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
      double err = (double)cx - (double)(fwidth/2);  //黄色の先の重心座標(x)と画像の中心(x)との差
      cmd_msg.linear.x =0.2;
      cmd_msg.angular.z = -(double)(err/800);
   }else{
      cmd_msg.linear.x =0.0;
      cmd_msg.angular.z = 0.2;
   }
   if(RUN) cmd_vel_pub.publish(cmd_msg);

   cv::imshow("original", frame);
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
   ros::Subscriber rgb_sub = lt.nh.subscribe(IMAGE_TOPIC, 1000, &LINETRACE::image_callback, &lt);
   ros::Subscriber ir_sub = lt.nh.subscribe(IR_TOPIC, 1000, &LINETRACE::ir_callback, &lt);
   lt.linetrace_start = lt.nh.advertiseService(lt.LINETRACE_SERVICE_START, &LINETRACE::linetrace_start_service, &lt);
   lt.linetrace_stop =lt.nh.advertiseService(lt.LINETRACE_SERVICE_STOP, &LINETRACE::linetrace_stop_service, &lt);
   lt.mg400_work_start = lt.nh.serviceClient<std_srvs::Empty>(lt.MG400_PICKUP_SERVICE_START);
   lt.mg400_work_stop = lt.nh.serviceClient<std_srvs::Empty>(lt.MG400_PICKUP_SERVICE_STOP);
   
   // Program succesful
   ros::spin();
   return 0;
 }
