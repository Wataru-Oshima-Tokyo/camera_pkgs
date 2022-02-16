 #include <ros/ros.h>

 // Include opencv2
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
//  #include "roscpp_tutorials/TwoInts.h"
//  #include <opencv2/core/types.hpp>

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <vector>
 #include <map>
 #define IMG_HEIGHT (240)
 #define IMG_WIDTH (320)
 #define rep(i,a,b) for(int i=a;i<b;i++)
 #define fore(i,a) for(auto &i:a)
 using namespace std;


 // Publisher


class AUTOCALIB{
  public:
    AUTOCALIB();
    ~AUTOCALIB();
    const std::string OPENCV_WINDOW = "Image window";

    // Topics
    const std::string IMAGE_TOPIC = "/camera/color/image_raw";
    const std::string PUBLISH_TOPIC = "/image_converter/output_video";
    void image_callback(const sensor_msgs::ImageConstPtr&);
    void clbk_start_service();
    void clbk_stop_service();
    void start_autocalib();
    void setDefault();
    void setOpenCVPrams();
    void setColorRange(int&, int&, int&);
    void getCircle(int);
    void mouseEvent();
    void drawMark();

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    ros::ServiceServer start, stop;
    // OpenCV Window Name

  private:
    bool RUN = false;
    int low_c[3] = {17, 123, 121};
    int high_c[3] ={37, 143, 201};
    int SCREEN_HEGHT, SCREEN_WIDTH;
    int AVERAGE_COUNT =1;
    int arm_x=0, arm_y=0, arm_r = 0;
    int obj_x=0, obj_y=0, obj_r = 0;
    bool isSetArm =false, isSetObj=false;
    //#################################################################
		//#  カメラの高さ = 500 mm  [台からカメラレンズの下側まで]        #
		//#################################################################
		const int HOUGH_1      = 30;  //# 手法依存の 1 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
						//# Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値を表す
		const int HOUGH_2      = 10;  //# 円の中心を検出する際の投票数の閾値を表す。これが小さくなるほど，
						//# より多くの誤検出が起こる可能性がある。
						//# より多くの投票を獲得した円が，最初に出力される。
		
		const int ARM_SIZE_MIN = 12;   //# アームの円形のマークの最小半径  (5)
		const int ARM_SIZE_MAX = 25;   //# アームの円形のマークの最大半径  (20)
		
		const int OBJ_SIZE_MIN = 30;  //# 円形のピッキング・オブジェクトの最小半径  (20)
		const int OBJ_SIZE_MAX = 42;  //# 円形のピッキング・オブジェクトの最大半径  (50)
		//#################################################################
    map<string, int[4]> CIRCLE_PARAMS[2];


    cv::Mat frame,frame_HSV, frame_threshold, mask;


};

AUTOCALIB::AUTOCALIB(){}
AUTOCALIB::~AUTOCALIB(){}
 
 void AUTOCALIB::clbk_start_service(){
   cout << "start autocalib" << endl;
   RUN = true;

 }

 void AUTOCALIB::clbk_stop_service(){
   cout << "stop autocalib" << endl;
   RUN = false;
 }


void AUTOCALIB::setDefault(){
  if (!(frame.empty())){
    SCREEN_HEGHT = frame.cols;
    SCREEN_WIDTH = frame.rows;
  }
}

void AUTOCALIB::setOpenCVPrams(){
  //arm
  CIRCLE_PARAMS[0]["HOUGH"][0] = HOUGH_1;
  CIRCLE_PARAMS[0]["HOUGH"][1] = HOUGH_2;
  CIRCLE_PARAMS[0]["HOUGH"][2] = ARM_SIZE_MIN;
  CIRCLE_PARAMS[0]["HOUGH"][3] = ARM_SIZE_MAX;
  //obj
  CIRCLE_PARAMS[1]["HOUGH"][0] = HOUGH_1;
  CIRCLE_PARAMS[1]["HOUGH"][1] = HOUGH_2;
  CIRCLE_PARAMS[1]["HOUGH"][2] = OBJ_SIZE_MIN;
  CIRCLE_PARAMS[1]["HOUGH"][3] = OBJ_SIZE_MAX;

  rep(i,0,2){
      rep(q,0, 4){
        CIRCLE_PARAMS[i]["MIN"][q] =0;
        CIRCLE_PARAMS[i]["MAX"][q] =0;
      }
  }

}

void AUTOCALIB::setColorRange(int &index, int &x, int &y){
  cout << "POS: " << x << y << endl;
  cv::Mat _bgr = frame(cv::Range(x,y),cv::Range(x+1,y+1)); // Slicing to crop the image
  cv::Vec3b &_bgr00 = _bgr.at<cv::Vec3b>(0,0);

  cout << "RGB: " << _bgr00[0] << endl;
  cv::Mat _hsv;
  cv::cvtColor(_bgr, _hsv, cv::COLOR_BGR2HSV);
  cv::Vec3b &_hsv00 = _hsv.at<cv::Vec3b>(0,0);
  cout << "HSV: " << _hsv00[0] <<endl;
  //#################################################################
  //#  HSV 空間における、指定色から最小値と最大値                   #
  //#################################################################
  int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
  int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
  //#################################################################

  	CIRCLE_PARAMS[index]["MIN"][0] = max(0, _hsv00[0] - _MIN_DH);
		CIRCLE_PARAMS[index]["MIN"][1] = max(0, _hsv00[1] - _MIN_DS);
		CIRCLE_PARAMS[index]["MIN"][2] = max(0, _hsv00[2] - _MIN_DV);
		// print("MIN: ", np.array(self.CIRCLE_PARAMS[index]["MIN"]))
    printf("MIN: ");
    fore(i, CIRCLE_PARAMS[index]["MIN"])
      cout << i << " ";
    cout <<endl;


		
		CIRCLE_PARAMS[index]["MAX"][0] = min(255, _hsv00[0] + _MAX_DH);
		CIRCLE_PARAMS[index]["MAX"][1] = min(255, _hsv00[1] + _MAX_DS);
	  CIRCLE_PARAMS[index]["MAX"][2] = min(255, _hsv00[2] + _MAX_DV);
    printf("MAX: ");
    fore(i, CIRCLE_PARAMS[index]["MAX"])
      cout << i << " ";
    cout <<endl;

		// print("MAX: ", np.array(self.CIRCLE_PARAMS[index]["MAX"]))

  
}


 void AUTOCALIB::image_callback(const sensor_msgs::ImageConstPtr& msg){
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

   frame = cv_ptr->image;
   int fheight = frame.cols, fwidth = frame.rows;

   //change the image from BGR to HSV
   cv::cvtColor(AUTOCALIB::frame, AUTOCALIB::frame_HSV, cv::COLOR_BGR2HSV);
   //mask the image in yellow
   cv::inRange(AUTOCALIB::frame_HSV, cv::Scalar(low_c[0],low_c[1],low_c[1]), cv::Scalar(high_c[0],high_c[1],high_c[2]),AUTOCALIB::mask);
   //resize the image
  //  cv::resize(mask, resized, cv::Size(), fwidth/3, fheight/3);
   int search_top = fheight/4*3; 
   int search_bot = search_top+20;//focus on just the front of cam <- need to know only 20 rows
  //  mask[0:search_top, 0:w] =0;
  //  mask[search_bot:fheight, 0:w] = 0;
  //  for(int i=1; i<mask.rows-1; i++){
  //    for(int j=1; j<mask.cols-1; j++){
  //      cv::Vec3b &color = mask.at<cv::Vec3b>(cv::Point(j,i));
  //      if(j< search_top || (j>search_bot && j<fheight)){
  //         color.val[0] =0;
  //         color.val[1] =0;
  //         color.val[2] =0;
  //      }
  //    }
  //  }

   cv::Moments M = cv::moments(AUTOCALIB::mask); // get the center of gravity
   if (M.m00 >0){
   			int cx = int(M.m10/M.m00); //重心のx座標
   			int cy = int(M.m01/M.m00); //重心のy座標
      
      cv::circle(AUTOCALIB::frame, cv::Point(cx,cy), 5, cv::Scalar(0, 0, 255));
   }
   
   cv::imshow("original", AUTOCALIB::frame);
   //cv::imshow("Masked", mask);
   cv::waitKey(3);

}


 int main(int argc, char** argv)
 {
   // Initialize the ROS Node "roscpp_example"
   ros::init(argc, argv, "roscpp_example");
   AUTOCALIB atcb;
   atcb.sub = atcb.nh.subscribe(atcb.IMAGE_TOPIC, 1000, &AUTOCALIB::image_callback, &atcb);
   // Instantiate the ROS Node Handler as nh

   // Print "Hello ROS!" to the terminal and ROS log file

   // Program succesful
   ros::spin();
   return 0;
}
