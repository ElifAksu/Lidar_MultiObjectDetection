#ifndef MY_PCL_TUTORIAL_RVIZ_VISUALIZER_H
#define MY_PCL_TUTORIAL_RVIZ_VISUALIZER_H

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Int32MultiArray.h"
#include "ros/node_handle.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "Eigen/Dense"

class rvizVis
{
public:
  ros::NodeHandle nh;
  ros::NodeHandle nh_;
  ros::Subscriber subs;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markerarray;
  std::vector<visualization_msgs::Marker> marker_vector;

  rvizVis(pcl::PointCloud<pcl::PointXYZ> vis_points, ros::Publisher vis_pub ,double r, double g);


};


#endif
