#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#define SHOWIMAGES

class ImageSelector
{
  	ros::NodeHandle nh;
	image_transport::ImageTransport it;
  	image_transport::CameraSubscriber sub;
	image_transport::CameraPublisher  pub;

  public:
    	ImageSelector(): it(nh)
  	{
    
    	sub = it.subscribeCamera("camera/image_raw", 1, &ImageSelector::imageCallback, this);
	    pub = it.advertiseCamera("camera/sel/image_raw", 1);
#ifdef SHOWIMAGES
	    cv::namedWindow("original");
 	    cv::namedWindow("edges");
 	    cv::namedWindow("noiseless");
 	    cv::startWindowThread();
#endif
    	}
	
	void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
  
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

    		cv::Mat edges;
    		cv::Canny(cv_ptr->image, edges, 50, 100);
    

    		std::vector<cv::Vec4i> lines;
    		cv::HoughLinesP(edges, lines, 1, CV_PI/180, 25, 25, 10); // Detect lines
    		int count=0;
    		
    		if (lines.size() > 1000)     // Lots of lines indicate a high noise
       			count=500;
    		else			    // Looks for horizontal lines close to the borders	
       			for (size_t i=0; i<lines.size(); i++) {
          			cv::Vec4i l = lines[i];
          			cv::Point a(l[0], l[1]);
          			cv::Point b(l[2], l[3]);
          			//line(cv_ptr->image, a, b, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
          			float slope=float(float(b.y - a.y) / float(b.x - a.x));
          			//ROS_INFO("%f %d %d %d %d", slope, a.x, a.y, b.x, b.y);
          			if ((fabs(slope) < 0.0001) && ((a.x<10) || b.x > 630))  // horizontal lines that start at righ borders or finish at the left border
	      				count++;
        		}
#ifdef SHOWIMAGES
    		cv::imshow("original", cv_ptr->image);
    		cv::imshow("edges", edges);
#endif

    		
    		if ((count<1)){
#ifdef SHOWIMAGES
       			cv::imshow("noiseless", cv_ptr->image);
#endif
			pub.publish(msg, info_msg);
		}
	}

}; // class ImageSelector

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ImageSelector IS;
  ros::spin();
  cv::destroyWindow("view");
}
