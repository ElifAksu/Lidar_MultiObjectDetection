#pragma once

#include <string>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"



#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#define W 5
#define H 10

/// \brief Receiver socket FD 
int senderSocket;
/// \brief A structure for sender  
struct sockaddr_in sender, receiver;

/// \brief Data to be sent over UDP
int tracking_data[(H) * W];
int temp[320];

/// \brief A callback fuction to subscribe object messages and send them over UDP
/// \param[out] A buffer to be filled with object messages
void trackingCallback(const std_msgs::Float32MultiArray::ConstPtr& array);