#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //cv::imshow("original", cv_ptr->image);	
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat edges;
    cv::Canny(cv_ptr->image, edges, 50, 100);
    

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 40, 40, 10);
    int count=0;
    //std::cout<<lines.size()<<std::endl;
    if (lines.size() > 500)
       count=500;
    else
       for (size_t i=0; i<lines.size(); i++) {
          cv::Vec4i l = lines[i];
          cv::Point a(l[0], l[1]);
          cv::Point b(l[2], l[3]);
          line(cv_ptr->image, a, b, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
          float slope=float(float(b.y - a.y) / float(b.x - a.x));
          //ROS_INFO("%f %d %d %d %d", slope, a.x, a.y, b.x, b.y);
          if ((fabs(slope) < 0.0001) && ((a.x<10) || b.x > 630)) 
	      count++;
        }
    cv::imshow("original", cv_ptr->image);
    cv::imshow("edges", edges);

    //ROS_INFO("************** %d", (int)count);
    if ((count<1))
       cv::imshow("noiseless", cv_ptr->image);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("original");
  cv::namedWindow("edges");
  cv::namedWindow("noiseless");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/scout_1/camera/left/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
