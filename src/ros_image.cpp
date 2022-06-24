
  #include <memory>
  #include "rclcpp/rclcpp.hpp"
 // Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
 #include <opencv2/videoio.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/msg/image.h>
    using namespace cv;
    using std::placeholders::_1;
    struct timespec start, stop;
    double fstart, fstop;
    static const std::string IMAGE_TOPIC = "/color/image_rect_raw";
    Mat src, src_hsv, dst;

class imageSubscriber : public rclcpp::Node{

   public:  
        imageSubscriber() : Node("image_subscriber"){
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC,1000, std::bind(&imageSubscriber::image_callback, this, _1));

        }
    private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;


        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        	clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
        	std_msgs::msg::Header msg_header = msg->header;
        	std::string frame_id = msg_header.frame_id.c_str();
        	// ROS_INFO_STREAM("New Image from " << frame_id);

        	cv_bridge::CvImageConstPtr cv_ptr;
        	try
        	{
            		cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        	}
        	catch (cv_bridge::Exception& e)
        	{
            		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            		return;
        	}

        	src = cv_ptr->image;
        	dst.create(src.size(), src.type());
        	//cvtColor(src, src_gray, COLOR_BGR2GRAY);
        	cvtColor(src, src_hsv, COLOR_BGR2HSV);
        
        	// namedWindow(window_name, WINDOW_AUTOSIZE );
        	// CannyThreshold(0, 0);

        	clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        	std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));

        	putText(src, //target image
            		fps, //text
            		Point(10, 30), //top-left position
            		FONT_HERSHEY_DUPLEX,
            		1.0,
            		Scalar(118, 185, 0), //font color
            		2);
        	cv::imshow("src", src);
        	cv::waitKey(3);   
    	}
};


 int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imageSubscriber>());
    rclcpp::shutdown();
    destroyAllWindows();
    return 0;
 }
