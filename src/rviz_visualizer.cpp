#include <ros/ros.h>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Int32MultiArray.h"
#include "ros/node_handle.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "../include/rviz_visualizer.h"
#include "Eigen/Dense"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



 rvizVis::rvizVis(pcl::PointCloud<pcl::PointXYZ> vis_points, ros::Publisher vis_pub, double r, double g)
 {
   marker_vector.clear();
   
  int objnum= vis_points.size();
  ROS_INFO(" visualized obj number = %d",objnum);

  for(int i =0; i<objnum; i++){
    
 
   marker.header.frame_id = "vehicle_frame";
   marker.header.stamp = ros::Time::now();
   marker.ns = "";
   marker.id = i;
   marker.type = visualization_msgs::Marker::CUBE;
   marker.action = visualization_msgs::Marker::ADD;
   marker.pose.position.x = vis_points[i].x;
   marker.pose.position.y = vis_points[i].y;
  // ROS_INFO(" box center x , y %f %f",vis_points[i].x,vis_points[i].y);
   // ROS_INFO_STREAM("Marker x: " << marker.pose.position.x  << " y: " << marker.pose.position.y);
   marker.pose.position.z = 0;
   marker.lifetime = ros::Duration(0.2);
   marker.pose.orientation.x = 0;
   marker.pose.orientation.y =0;
   marker.pose.orientation.z =0;
   marker.pose.orientation.w = 1.0;
   marker.scale.x = 1.0;
   marker.scale.y = 1.0;
   marker.scale.z = 1.0;
   marker.color.a = 1.0; // Don't forget to set the alpha!
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;
   //markerarray.markers.push_back(marker); 
   marker_vector.push_back(marker);
  
  }
  // vis_pub.publish( markerarray);
  for(int jj=0; jj<objnum; jj++)
  {
    vis_pub.publish( marker_vector[jj]);
  }

 

 
};





