#include <iostream>
#include <ros/ros.h>
#include "davis_obj_detection.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "davis_obj_detection");

  DavisObjDetection davis_obj_detection;

  ros::spin();
  return 0;
}
