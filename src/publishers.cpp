#include "ros_uav_command/testPositioning.h"

// Publishers
ros::Publisher globalPositionTarget_pub;
ros::Publisher localPositionTarget_pub;

void init_publishers(ros::NodeHandle n) {

  // Init all the publishers
  globalPositionTarget_pub = n.advertise<mavros_msgs::GlobalPositionTarget>("uav1/mavros/setpoint_raw/global", 5);
  localPositionTarget_pub  = n.advertise<mavros_msgs::PositionTarget>      ("uav1/mavros/setpoint_raw/local" , 5);

  return;

}

void publish_global_target(double lat, double lon, double alt) {

    // Publishes a global position target and ignores the requested RPY
    // http://docs.ros.org/en/api/mavros_msgs/html/msg/GlobalPositionTarget.html

    // Create the message
    mavros_msgs::GlobalPositionTarget msg;

    // Fill in the header        
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    // Co-ordinate Frame
    msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_TERRAIN_ALT;
    
    // Type Mask
    msg.type_mask = 
          mavros_msgs::GlobalPositionTarget::IGNORE_VX // Velocity vector ignore flags
        + mavros_msgs::GlobalPositionTarget::IGNORE_VY
        + mavros_msgs::GlobalPositionTarget::IGNORE_VZ
        + mavros_msgs::GlobalPositionTarget::IGNORE_AFX // Acceleration/Force vector ignore flags
        + mavros_msgs::GlobalPositionTarget::IGNORE_AFY
        + mavros_msgs::GlobalPositionTarget::IGNORE_AFZ
        + mavros_msgs::GlobalPositionTarget::FORCE      // Force in af vector flag
        + mavros_msgs::GlobalPositionTarget::IGNORE_YAW
        + mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;

    // Position
    msg.latitude  = lat;
    msg.longitude = lon;
    msg.altitude  = alt;

    // Velocity
    msg.velocity.x = 0.0;
    msg.velocity.y = 0.0;
    msg.velocity.z = 0.0;

    // Acceleration
    msg.acceleration_or_force.x = 0.0;
    msg.acceleration_or_force.y = 0.0;
    msg.acceleration_or_force.z = 0.0;

    // Yaw
    msg.yaw      = 0.0;
    msg.yaw_rate = 0.0;

    // Publish the command
    globalPositionTarget_pub.publish(msg);

    // Debug
    //printf("Targeting: (%6.2f, %6.2f, %6.2f)\n",
    //    msg.latitude,msg.longitude,msg.altitude);

    // All done
    return;

}

void publish_local_target(double x, double y, double alt) {
   
    // Publishes a local target.  Requires a heading
    static uint32_t seq = 0;

    mavros_msgs::PositionTarget msg;

    // Fill in the header
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq++;
    msg.header.frame_id = "world";

    // Co-ordinate Frame
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    // Type Mask
    msg.type_mask = 
        //   mavros_msgs::PositionTarget::IGNORE_PX // Position ignore flags
        // + mavros_msgs::PositionTarget::IGNORE_PY
        // + mavros_msgs::PositionTarget::IGNORE_PZ
        + mavros_msgs::PositionTarget::IGNORE_VX // Velocity vector ignore flags
        + mavros_msgs::PositionTarget::IGNORE_VY
        + mavros_msgs::PositionTarget::IGNORE_VZ
        + mavros_msgs::PositionTarget::IGNORE_AFX // Acceleration/Force vector ignore flags
        + mavros_msgs::PositionTarget::IGNORE_AFY
        + mavros_msgs::PositionTarget::IGNORE_AFZ
        + mavros_msgs::PositionTarget::FORCE      // Force in acceleration / force vector flag
        + mavros_msgs::PositionTarget::IGNORE_YAW
        + mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // Position
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = alt;

    // Velocity
    msg.velocity.x = 0.0;
    msg.velocity.y = 0.0;
    msg.velocity.z = 0.0;

    // Acceleration
    msg.acceleration_or_force.x = 0.0;
    msg.acceleration_or_force.y = 0.0;
    msg.acceleration_or_force.z = 0.0;

    // Yaw
    msg.yaw      = 0.0;
    msg.yaw_rate = 0.0;

    // Publish the message
    localPositionTarget_pub.publish(msg);

    // Debugging
    // printf("Commanded Position: (%6.2f, %6.2f, %6.2f)\n",
    //      msg.position.x,msg.position.y,msg.position.z);

    return;
}
