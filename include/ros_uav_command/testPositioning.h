
#pragma once

// ROS Headers
#include "ros/ros.h"

#include "std_msgs/String.h"

#include "geometry_msgs/PoseStamped.h"  // Setpoint publish, current position
#include "geometry_msgs/Quaternion.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/HomePosition.h"

#include "geographic_msgs/GeoPoint.h" // Home Position
#include "geographic_msgs/GeoPointStamped.h"
#include <tf2/LinearMath/Quaternion.h>

#include "mavros_msgs/SetMode.h" 
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/MessageInterval.h"

#include <mavlink/v2.0/common/common.hpp>

// Standard C++ headers
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <stdlib.h>
#include <time.h>

//======= ROS Stuff

// Callbacks
void homePosition_callback(const mavros_msgs::HomePosition::ConstPtr& msg);

// Publishers
extern ros::Publisher globalPositionTarget_pub;
extern ros::Publisher localPositionTarget_pub;

// Subscribers
extern ros::Subscriber homePositionSub;


//======= Functions
void publish_global_target(double lat, double lon, double alt);
void publish_local_target (double x, double y, double alt);

// Initialisation
void init_publishers (ros::NodeHandle n);
void init_subscribers(ros::NodeHandle n);


void init_services(ros::NodeHandle n);
bool setMessageRate(uint32_t msg_id, float rate);
bool cmdSetSpeed(float newSpeed);
bool setMode_AUTO();
bool setMode_GUIDED();
bool setMode_RTL();


// Helpers
void XY_to_latlon(double   X, double   Y, double *lat, double *lon);
void latlon_to_XY(double lat, double lon, double   *X, double   *Y);

//======= Variables

extern geographic_msgs::GeoPoint home_;

// Debugging (main)
extern uint _counter;
