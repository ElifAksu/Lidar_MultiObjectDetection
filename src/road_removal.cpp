//
// Created by kosuke on 11/26/17.
//

// #include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <math.h>
//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


#include "../include/road_removal.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

//int numChannel = 80;
//int numBin = 120;
//const int numMedianKernel = 1;
float roadMin = -15;
float roadMax = 17;

void roadRemove(PointCloud<pcl::PointXYZ>   elevatedCloud,
              PointCloud<pcl::PointXYZ>::Ptr  nonRoadCloud,
              PointCloud<pcl::PointXYZ>::Ptr  RoadCloud){

    for (int i = 0; i < elevatedCloud.size(); i++) {
        float x = elevatedCloud.points[i].x;
        float y = elevatedCloud.points[i].y;
        float z = elevatedCloud.points[i].z;
            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;

        if(x<= roadMin || x>= roadMax ) {

            nonRoadCloud->push_back(o);
        }
        else{

            RoadCloud->push_back(o);

            
        }
    }


}
