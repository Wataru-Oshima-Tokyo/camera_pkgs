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
 #include <geometry_msgs/Twist.h>
 #include <std_srvs/Empty.h>
 #include <std_msgs/String.h>
 #include <sensor_msgs/LaserScan.h>
 #include <vector>
 #include <camera_pkg/Coordinate.h>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define NUM_THREADS 4
 #define RANGE_MAX 5.6; 
 using namespace cv;
 using namespace std;
// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";
//static const std::string video_name="outcpp.avi";


// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
class RECORD{
    public:
        ros::NodeHandle nh;
        ros::Subscriber image;
        const cv::String video_name="outcpp.avi";
        // int ex = static_cast<int>(inputVideo.get(CAP_PROP_FOURCC));
        VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(640,480));
        // VideoWriter video.open(video_name,ex,30, Size(640,480, true);
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        RECORD();
        ~RECORD();
};

RECORD::RECORD(){}
RECORD::~RECORD(){}
void RECORD::image_callback(const sensor_msgs::ImageConstPtr& msg){
   std_msgs::Header msg_header = msg->header;
   std::string frame_id = msg_header.frame_id.c_str();

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
    cv::Mat frame = cv_ptr->image;
    int frame_width = frame.size().width;
    int frame_height = frame.size().height;
   
    video.write(frame);

    cv::imshow("original", frame);
    cv::waitKey(3);
}
   int main(int argc, char** argv)
 {
    ros::init(argc, argv, "roscpp_example");
    RECORD rc;
    rc.image = rc.nh.subscribe(IMAGE_TOPIC, 1000, &RECORD::image_callback, &rc);
    ros::spin();
    destroyAllWindows();
	return 0;
 }
