#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include <signal.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";
bool g_caught_sigint = false;

using namespace cv;

//sig handler
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
   //  ROS_INFO("entered constructor");
    image_sub_ = it_.subscribe("/xtion_camera/rgb/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
     ROS_INFO("destoryed");
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	// ROS_INFO("entered show");
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
        ROS_INFO("entered image cb");
	 
	  cv::Mat src, src_filtered, src_gray;
	  /// Read the image
	  src = cv_ptr->image.clone();
	  src_filtered = src.clone();
	  int rows = src.rows;
	  int cols = src.cols;
	   int rowsf = src_filtered.rows;
	  int colsf = src_filtered.cols;
	  cv::inRange(src, cv::Scalar(10, 10, 10), cv::Scalar(180, 180, 180), src_filtered);
       ROS_INFO("src rows %d", rows);
        ROS_INFO("src cols %d", cols);
	    ROS_INFO("src filtered rows %d", rowsf);
	    ROS_INFO("src filtered cols %d", colsf);
         int count = 0;
		 for( unsigned int row = 0; row < src_filtered.rows; row++){	
			  for ( unsigned int col = 0; col < src_filtered.cols; col++){
				  count ++;
				    //ROS_INFO("entered");
					//if(src_filtered.at<uchar>(row,col) < 255){
						  // ROS_INFO("entered2");
							src.at<uchar>(row,col) = 0;
					//}	
			 
			  /*Vec3f pixel = src_filtered.at<Vec3f>(row, col);
			  int b = pixel[0];
			  int g = pixel[1];
			  int r = pixel[2];
			  if(!(b < 0 || b > 10 || g < 180 || g > 10 || r < 180 || r > 10)){
				   src.at<uchar>(row,col) = 255;
			 
		      }		*/
		  }
			
	  }
	    ROS_INFO("count %d", count);
	
	/*	
	  /// Convert it to gray
	  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
	  
	  /// Reduce the noise so we avoid false circle detection
	  cv::GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
	
	  vector<Vec3f> circles;
	
	  /// Apply the Hough Transform to find the circles
	  cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 50, 20, 0, 30 );

	  /// Draw the circles detected
	  for( size_t i = 0; i < circles.size(); i++ )
	  {
	      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	      int radius = cvRound(circles[i][2]);
	      // circle center
	      cv::circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
	      // circle outline
	      cv::circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
	     
	   }
	     ROS_INFO("drew x circles %d", circles.size());

	  /// Show your results*/
	  cv::imshow( "Hough Circle Transform Demo", src );
	  cv::imshow( "in range", src_filtered );	
    
	  //pause for 3 ms
      cv::waitKey(3000);
	 
      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
  
    }
    
};


/** @function main */
int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "");
  signal(SIGINT, sig_handler);
  ImageConverter ic;
  while (ros::ok())
  {   
    ros::spinOnce();
    //set the header
  }
 
  return 0;
}
