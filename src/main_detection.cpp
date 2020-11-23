#include <ros/ros.h>
#include "../include/cloudprocess.h"

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "main_detection");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  cloudprocess(nh, private_nh);
  return 0;

}