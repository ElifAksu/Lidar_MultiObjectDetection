#ifndef MY_PCL_TUTORIAL_IMMUKF_H
#define MY_PCL_TUTORIAL_IMMUKF_H
#include <ros/ros.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "rviz_visualizer.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

void tracking_immUkf(vector<vector<double>> bBoxes,  double timestamp, 
	PointCloud<PointXYZ>& targets, vector<vector<double>>& targetVandYaw, 
	vector<int>& trackManage, vector<bool>& isStaticVec,
	vector<bool>& isVisVec, vector<PointCloud<PointXYZ>>& visBB, ros::Publisher vis_pub, ros::Publisher pub, std_msgs::Float32MultiArray msg_array1, double r, double g,std_msgs::Float32MultiArray pp_objects,vector<PointCloud<PointXYZ>> Boxes);


#endif