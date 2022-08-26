
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
                fs.release();
            };
	//  ~Image(){};
    



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
        // cv::imshow("src", src);
        // cv::waitKey(3);
    }

    void aruco_marker_detector(){
        if(initial){
            printf("start detecting\n"); 
            initial = false;
        }
         
        Mat imageCopy;
        src.copyTo(imageCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        aruco::detectMarkers(src, dictionary, corners, ids);
        
        // if at least one marker detected
        if (ids.size() > 0){
            printf("detected\n");
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            std::cout << "top left: " << corners[0][0].x << ", " << corners[0][0].y<< std::endl;
        }
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
        imshow("original", src);
        if(!imageCopy.empty())
            imshow("out", imageCopy);
        waitKey(3);
        

    }

    bool initial = true;
    struct timespec start, stop;
    double fstart, fstop;
    std::string IMAGE_TOPIC;
    std::string CALIBRATION;
    Mat src, src_hsv, dst, camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
};






 int main(int argc, char* argv[]){
    ros::init(argc, argv, "roscpp_example");
    Image img;
    ros::NodeHandle nh;
    ros::Subscriber image = nh.subscribe(img.IMAGE_TOPIC, 1000, &Image::image_callback, &img);
    while(ros::ok()){
        if(!img.src.empty())
            img.aruco_marker_detector();
        ros::spinOnce();
    }

    destroyAllWindows();
    return 0;
 }
