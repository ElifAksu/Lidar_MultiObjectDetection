#ifndef CLOUDPROCESS_H
#define CLOUDPROCESS_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <assert.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"
// #include <pcl_ros/transforms.h>


#include <vector>
#include <iostream>
#include <math.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "../include/ground_removal.h"
#include "../include/component_clustering.h"
#include "../include/box_fitting.h"

#include "../include/rviz_visualizer.h"
#include "std_msgs/UInt32.h"

#include "../include/road_removal.h"

#include "../include/ground_removal.h"

using namespace std;

class cloudprocess
{
private:
   std::string  topicname_in;
   std::string  framename_in;
   int selection;
   int counta;

  ros::Publisher  filtered_cloud_pub;
  ros::Publisher  boxline_pub;
  ros::Publisher  distance_pub;
  ros::Publisher  pointnumber_pub;
  ros::Publisher  clusterid_pub; 
  ros::Publisher  objects_pub;
  ros::Publisher  vis_pub;

  visualization_msgs::Marker line_strip;
  visualization_msgs::MarkerArray marker_;
  visualization_msgs::MarkerArray pointnumber_marker;
  visualization_msgs::MarkerArray clusterid_marker;
public:
    cloudprocess(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr input);
    ~cloudprocess();
};

#endif