//
// Created by kosuke on 11/29/17.
//

#ifndef MY_PCL_TUTORIAL_BOX_FITTING_H
#define MY_PCL_TUTORIAL_BOX_FITTING_H

#include <array>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <iostream>

#include "component_clustering.h"

using namespace std;
using namespace pcl;

extern float picScale; // picScale * roiM = 30 * 30
//const float picScale = 30;
extern int ramPoints;
extern int lSlopeDist;
extern int lnumPoints;

extern float tHeightMin;
extern float tHeightMax;
extern float tWidthMin;
extern float tWidthMax;
extern float tLenMin;
extern float tLenMax;
extern float tAreaMax;
extern float tRatioMin;
extern float tRatioMax;
extern float minLenRatio;
extern float tPtPerM3;
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

vector<PointCloud<PointXYZ>> boxFitting(PointCloud<PointXYZ>::Ptr elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        int numCluster , ros::Publisher pointcloudPub, ros::Publisher clusteridPub, string frame ,visualization_msgs::MarkerArray pointnumber_marker, visualization_msgs::MarkerArray clusterid_marker);


#endif //MY_PCL_TUTORIAL_BOX_FITTING_H
