
#include <ros/ros.h>
#include <iostream>
 // Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
 #include <opencv2/videoio.hpp>
 #include <opencv2/aruco.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>

using namespace cv;


class Image{
   public:	 
   Image(){
        ros::NodeHandle private_nh("~");
        private_nh.param("image_topic", IMAGE_TOPIC, std::string("/usb_cam/image_raw"));  
        private_nh.param("video_path", VIDEO_PATH, std::string("")); 
     };
	 ~Image(){};
    ros::NodeHandle nh;
    ros::Subscriber image = nh.subscribe("/usb_cam/image_raw", 1000, &Image::image_callback, this);



    void image_callback(const sensor_msgs::ImageConstPtr& msg){
        clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
        std_msgs::Header msg_header = msg->header;
        std::string frame_id = msg_header.frame_id.c_str();

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
        cvtColor(src, src_hsv, COLOR_BGR2HSV);

        clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));

        putText(src, fps,Point(10, 30), FONT_HERSHEY_DUPLEX,1.0,Scalar(118, 185, 0), 2);
        cv::imshow("src", src);
        cv::waitKey(3);
    }

    void aruco_marker_detector(){
        VideoCapture cap(VIDEO_PATH);
        if(!cap.isOpened()){
            std::cout << "Error opening video stream or file" << std::endl;
        }
        while(1){
            Mat frame;
            cap >> frame;
            Mat imageCopy;
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners;
            aruco::detectMarkers(frame, dictionary, corners, ids);
            // if at least one marker detected
            if (ids.size() > 0)
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            imshow("original", imageCopy);
            imshow("out", imageCopy);
            waitKey(3);
        }
        

    }

    struct timespec start, stop;
    double fstart, fstop;
    std::string IMAGE_TOPIC;
    std::string VIDEO_PATH;
    Mat src, src_hsv, dst;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
};






 int main(int argc, char* argv[]){
    ros::init(argc, argv, "roscpp_example");
    Image img;
    // if(!img.src.empty()){
    //     img.aruco_marker_detector();
    // }
    img.aruco_marker_detector();
    ros::spin();
    destroyAllWindows();
    return 0;
 }
