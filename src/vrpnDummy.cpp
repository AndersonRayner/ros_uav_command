
// Script to provide dummy VRPN (MOCAP) data for testing
//
// Matt Anderson, 2022

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <math.h>

ros::Publisher vrpnDummy_pub;

// Node for handling navigation around fumaroles
int main(int argc, char **argv) {

  // Initialise ROS
  ros::init(argc, argv, "vrpn_dummy");
  ros::NodeHandle n;

  // Initialise Publishers
  vrpnDummy_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
 
  // Set up loop rates
  ros::Rate loop_rate(10);

  ROS_INFO("==== VRPN Dummy Node ====");

  // ROS Loop
  while (ros::ok()) {

    static uint counter = 0;

    // Create the message
    geometry_msgs::PoseStamped msg;

    // Header
    msg.header.stamp    = ros::Time::now();
    msg.header.seq      = counter++;
    msg.header.frame_id = "base_link";

    // Pose
    msg.pose.position.x = cos(counter)*0.5;
    msg.pose.position.y = sin(counter)*0.5;
    msg.pose.position.z = sin(counter)*0.5;
    
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    
    // Publish the message
    vrpnDummy_pub.publish(msg);

    // Give ROS time to do it's thing
    ros::spinOnce();
    loop_rate.sleep();
    counter++;

  }

  return 0;
  
}
