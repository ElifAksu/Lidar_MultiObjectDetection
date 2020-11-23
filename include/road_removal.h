//
// Created by kosuke on 11/26/17.
//

#ifndef MY_PCL_TUTORIAL_ROAD_REMOVAL_H
#define MY_PCL_TUTORIAL_ROAD_REMOVAL_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//#include "gaus_blur.h"

using namespace std;
using namespace pcl;


void roadRemove(PointCloud<pcl::PointXYZ> elevatedCloud, 
                  PointCloud<pcl::PointXYZ>::Ptr nonRoadCloud, 
                  PointCloud<pcl::PointXYZ>::Ptr RoadCloud); 
// void groundRemove(PointCloud<pcl::PointXYZ> cloud, 
//                   PointCloud<pcl::PointXYZ>::Ptr elevatedCloud); 

#endif //TEST1_ROAD_REMOVAL_H
