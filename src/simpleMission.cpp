
// Script to demonstrate different ways of commanding an ArduPilot 
// through ROS.
//
// Matt Anderson, 2021

#include "ros_uav_command/ros_uav_command.h"

uint _counter = 0;

enum missionStateEnum {takeoff, circuit, landing} missionState;

// Node for handling navigation around fumaroles
int main(int argc, char **argv) {

  // Initialise variables
  missionState = missionStateEnum::takeoff;

  // Initialise ROS
  ros::init(argc, argv, "simpleMission");
  ros::NodeHandle n;

  // Initialise Publishers
  init_publishers(n);
  init_subscribers(n);

  init_services(n);

  // Set up loop rates
  ros::Rate loop_rate(20);

  ROS_INFO("==== Simple Mission Test Node ====");

  setMessageRate(mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID,   1.0f); // GPS_GLOBAL_ORIGIN (mavros/global_postion/gp_offset)
  setMessageRate(mavlink::common::msg::ATTITUDE::MSG_ID,           50.0f); // ATTITUDE (mavros/imu/data) 
  setMessageRate(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 25.0f); // LOCAL_POSITION_NED (mavros/local_postion/pose)

  // ROS Loop
  while (ros::ok()) {

    _counter++;

    switch (missionState)
    {
        case (missionStateEnum::takeoff) : 
        break;
                case (missionStateEnum::circuit) : 
        break;
                case (missionStateEnum::landing) : 
        break;

    }

    // Position Targets
    double X = cos(_counter/100.0)*(30.0);
    double Y = sin(_counter/100.0)*(30.0);
    double Z = 15.0 + 5.0*sin(3*_counter/100.0);

    // Attitude Targets
    double roll = 5.0/57.3, pitch = 0;
    double yaw = (double) _counter/100.0;
    double thrust = 0.50;

    // Give ROS time to do it's thing
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
