
// ROS Headers
#include "ros/ros.h"
#include "mavros_msgs/MessageInterval.h"

#include <mavlink/v2.0/common/common.hpp>

//======= Forward declarations
bool setMessageRate(uint32_t msg_id, float rate);
ros::ServiceClient setMessageRateClient;

// Node for setting UAV data rates

int main(int argc, char **argv) {

  // Initialise ROS
  ros::init(argc, argv, "setRates");
  ros::NodeHandle n;

  // Initialise Services
  setMessageRateClient = n.serviceClient<mavros_msgs::MessageInterval>("uav1/mavros/set_message_interval");

  // Set up loop rates
  ros::Rate loop_rate(1);

  // Give other systems time to boot
  ros::Duration(5.0).sleep();

  ROS_INFO("==== Set Aircraft Streaming Rates ====");

  while (ros::ok()) {

    bool rateSet_failed = 0;

    // Set data rates
    if (setMessageRate(mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID,   1.0f) == 0) rateSet_failed = 1; // mavros/global_postion/gp_offset
    if (setMessageRate(mavlink::common::msg::ATTITUDE::MSG_ID,           50.0f) == 0) rateSet_failed = 1; // mavros/imu/data
    if (setMessageRate(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 50.0f) == 0) rateSet_failed = 1; // mavros/local_postion/pose
    
    if (!rateSet_failed)
    {
        // All done
        ROS_INFO("Aircraft Data Rates Set");

        // Exit the program
        return 0;
    }

    // This round of setting rates failed, try again in a little bit
    ros::spinOnce();
    loop_rate.sleep();

  }

  // Exit the program
  return 0;
}


bool setMessageRate(uint32_t msg_id, float rate) {
  mavros_msgs::MessageInterval cmd;

  cmd.request.message_id = msg_id;
  cmd.request.message_rate = rate;

  setMessageRateClient.call(cmd);

  if (cmd.response.success) {
    ROS_INFO("Set message rate (#%d) successful",msg_id);
  } else {
    ROS_INFO("!!! Set message rate (#%d) failed !!!",msg_id);
  }

  return (cmd.response.success);

}

