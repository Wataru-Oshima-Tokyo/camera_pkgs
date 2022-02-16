 #include <ros/ros.h>

 // Include opencv2
 #include <opencv2/imgproc.hpp>
 #include <opencv2/highgui.hpp>
//  #include <opencv2/core.hpp>
//  #include "roscpp_tutorials/TwoInts.h"
//  #include <opencv2/core/types.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include "roscpp_tutorials/TwoInts.h"
 #include <vector>
 #include <map>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define rep(i,a,b) for(int i=a;i<b;i++)
 #define fore(i,a) for(auto &i:a)
 using namespace std;
 using namespace cv;


class CAMERA_CV{

public:
    Mat src, src_gray, dst, detected_edges;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    ros::ServiceServer start, stop;
    void CannyThreshold(int, void*);
    const std::string OPENCV_WINDOW = "Image window";
    virtual bool clbk_start_service(roscpp_tutorials::TwoInts::Request& req,roscpp_tutorials::TwoInts::Response& res);
    virtual bool clbk_stop_service(roscpp_tutorials::TwoInts::Request& req, roscpp_tutorials::TwoInts::Response& res);
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    // Topics
    const std::string IMAGE_TOPIC = "/camera/color/image_raw";
    const std::string PUBLISH_TOPIC = "/image_converter/output_video";
    const std::string SERVICE_START = "/canny/start";
    const std::string SERVICE_STOP = "/canny/stop";
    CAMERA_CV();
    ~CAMERA_CV();
    bool getRun();
private:
    bool RUN = false;
    int lowThreshold = 6;
    const int max_lowThreshold = 100;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
    const char* window_name = "Edge Map";
};

CAMERA_CV::CAMERA_CV(){

};
CAMERA_CV::~CAMERA_CV(){};

bool CAMERA_CV::getRun(){
  return RUN;
}

void CAMERA_CV::CannyThreshold(int, void*)
{
    // cout << "start canny threashold" <<endl;
    // removeing the noise and ap since the kernel size is 3, set the size 3 by 3
    blur(src_gray, detected_edges, Size(3,3) );
    // //detecing an edege by Canny with the lowThreashold and maxThreshold which is 3 times thant the lower one.
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    // //set the dist image all black so that you can put the deteceted edges on the black background
    dst = Scalar::all(0);
    // src.copyTo( dst, detected_edges);
    imshow( window_name, src);
    waitKey(3);
}

 bool CAMERA_CV::clbk_start_service(roscpp_tutorials::TwoInts::Request& req,roscpp_tutorials::TwoInts::Response& res){
   cout << "start canny_img" << endl;
   RUN = true;
   return RUN;

 }

 bool CAMERA_CV::clbk_stop_service(roscpp_tutorials::TwoInts::Request& req,roscpp_tutorials::TwoInts::Response& res){
   cout << "stop canny_img" << endl;
   RUN = false;
   destroyAllWindows();
   return RUN;
 }

 void CAMERA_CV::image_callback(const sensor_msgs::ImageConstPtr& msg){
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    src = cv_ptr->image;
    dst.create(src.size(), src.type());
    // cvtColor(src, src_gray, COLOR_BGR2GRAY);
    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

 }



int main( int argc, char** argv )
{
//   CommandLineParser parser( argc, argv, "{@input | ../data/fruits.jpg | input image}" );
//   src = imread( parser.get<String>( "@input" ), IMREAD_COLOR ); // Load an image
    
//     //call back
//     Mat src = imread("./ppms/frame200.ppm");
//     imshow("frame100", src);
//   if( src.empty() )
//   {
//     std::cout << "Could not open or find the image!\n" << std::endl;
//     std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
//     return -1;
//   }

  /* you can set the value of Threshold for the canny function. 
    It can be used for ajusting how much we want to detect the edge
  */
//   createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
  //call canny Threashold

   ros::init(argc, argv, "roscpp_example");
   CAMERA_CV cc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(100);
   
   cc.sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &CAMERA_CV::image_callback, &cc);
   cc.start = cc.nh.advertiseService(cc.SERVICE_START, &CAMERA_CV::clbk_start_service, &cc);
   cc.stop = cc.nh.advertiseService(cc.SERVICE_STOP, &CAMERA_CV::clbk_stop_service, &cc);
   while(ros::ok()){
      // cout << cc.getRun() << endl;
      if(cc.getRun()){
        cc.CannyThreshold(0, 0);
      }
      ros::spinOnce();
      loop_rate.sleep();
   }

    // ros::spin();
   // Instantiate the ROS Node Handler as nh

   // Print "Hello ROS!" to the terminal and ROS log file

   // Program succesful
   
  return 0;
}