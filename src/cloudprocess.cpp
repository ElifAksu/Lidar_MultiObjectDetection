#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <assert.h>

#include "../include/ground_removal.h"
#include "../include/cloudprocess.h"

using namespace std;

cloudprocess::cloudprocess(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{

  private_nh.param<std::string>("pointname", topicname_in, "/velodyne_points");
  private_nh.param<std::string>("framename", framename_in, "/velodyne");
  private_nh.param<int>("tracking_selection", selection, 0);
   
  counta=0;

  filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_output", 1);
  boxline_pub = nh.advertise<visualization_msgs::Marker>("visualization_lines", 10);
  distance_pub = nh.advertise<visualization_msgs::MarkerArray>("distance", 10);
  pointnumber_pub = nh.advertise<visualization_msgs::MarkerArray>("pointnum", 10);
  clusterid_pub = nh.advertise<visualization_msgs::MarkerArray>("clusterid", 10);
  objects_pub= nh.advertise<std_msgs::Float32MultiArray>("/Objects_merge", 1000);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  visualization_msgs::Marker line_strip;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (topicname_in, 1000,  boost::bind(&cloudprocess::cloud_callback, this,_1));
  ros::spin ();
   
}

cloudprocess::~cloudprocess()
{
}
void cloudprocess::cloud_callback(const sensor_msgs::PointCloud2ConstPtr input)
{
  PointCloud<pcl::PointXYZ> cloud;
  PointCloud<pcl::PointXYZ>::Ptr elevatedCloud (new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr groundCloud   (new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr roadCloud (new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr nonroadCloud   (new pcl::PointCloud<pcl::PointXYZ>());
  ros::Time begin = ros::Time::now();
  // Convert from ros msg to PCL::PointCloud data type
  fromROSMsg (*input, cloud);
  groundRemove(cloud, elevatedCloud, groundCloud);
  int numCluster = 0; // global variable?
  array<array<int, numGrid>, numGrid> cartesianData{};
  //road remove can be added here as: 
  //roadRemove(cloud, roadCloud, nonroadCloud);
  //componentClustering(roadCloud, cartesianData, numCluster);
  componentClustering(elevatedCloud, cartesianData, numCluster);

  // for visualization
  PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  makeClusteredCloud(elevatedCloud, cartesianData, clusteredCloud);

  // Convert from PCL::PointCloud to ROS data type
  clusteredCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  elevatedCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  roadCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  nonroadCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  sensor_msgs::PointCloud2 output;
  toROSMsg(*elevatedCloud, output);
  //toROSMsg(*roadCloud, output);
  
  // Publish the data.
  filtered_cloud_pub.publish(output);

  counta ++;
  cout << "Frame: "<<counta << "----------------------------------------"<< endl;

  vector<PointCloud<PointXYZ>> bBoxes = boxFitting(elevatedCloud, cartesianData, numCluster , pointnumber_pub, clusterid_pub, framename_in, pointnumber_marker, clusterid_marker);
  //assert(bBoxes.size() != 0);
  cout << "number of clusters" << numCluster<<endl;
  cout << "Bboxes size " <<bBoxes.size()<< endl;

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = framename_in;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  //LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.1;
  // Points are green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  int id = 0;string ids;
  for(int objectI = 0; objectI < bBoxes.size(); objectI ++){
    for(int pointI = 0; pointI < 4; pointI++){
      assert((pointI+1)%4 < bBoxes[objectI].size());
      assert((pointI+4) < bBoxes[objectI].size());
      assert((pointI+1)%4+4 < bBoxes[objectI].size());
      id ++; ids = to_string(id);
      geometry_msgs::Point p;
      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4].x;
      p.y = bBoxes[objectI][(pointI+1)%4].y;
      p.z = bBoxes[objectI][(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4+4].x;
      p.y = bBoxes[objectI][(pointI+1)%4+4].y;
      p.z = bBoxes[objectI][(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }

  //line list end
  boxline_pub.publish(line_list);


};



