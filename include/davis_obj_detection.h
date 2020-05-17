#ifndef DAVIS_OBJ_DETECTION_H
#define DAVIS_OBJ_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;


class DavisObjDetection
{
public:
  DavisObjDetection();
  ~DavisObjDetection();

private:
  // ROS interface
  ros::NodeHandle nh_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher event_image_pub_;

  ros::Subscriber image_sub_, event_image_sub_;
  void imageCallback(const sensor_msgs::Image::ConstPtr& image);
  void eventImageCallback(const sensor_msgs::Image::ConstPtr& image);

  const int K;
  int point_count_;
  cv::Point center_point_;
};

#endif //DAVIS_OBJ_DETECTION_H