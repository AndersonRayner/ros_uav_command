
#include "ros_uav_command/testPositioning.h"

uint _counter = 0;

enum positionModeEnum {local, global} positionMode;

// Node for handling navigation around fumaroles
int main(int argc, char **argv) {

  // Initialise variables
  positionMode = positionModeEnum::global;

  // Initialise ROS
  ros::init(argc, argv, "aircraft_positioning");
  ros::NodeHandle n;

  // Initialise Publishers
  init_publishers(n);
  init_subscribers(n);

  init_services(n);

  // Set up loop rates
  ros::Rate loop_rate(10);

  ROS_INFO("==== Aircraft Positioning Node Test ====");

  setMessageRate(mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID,   1.0f); // GPS_GLOBAL_ORIGIN (mavros/global_postion/gp_offset)
  setMessageRate(mavlink::common::msg::ATTITUDE::MSG_ID,           50.0f); // ATTITUDE (mavros/imu/data) 
  setMessageRate(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 25.0f); // LOCAL_POSITION_NED (mavros/local_postion/pose)

  // ROS Loop
  while (ros::ok()) {

    _counter++;

    // Position Targets
    double X = cos(_counter/100.0)*(30.0);
    double Y = sin(_counter/100.0)*(30.0);
    double Z = 15.0 + 5.0*sin(3*_counter/100.0);

    // Convert to Lat/Lon
 	double lat, lon, alt;
 	XY_to_latlon(X, Y, &lat, &lon);
    alt = Z;

    // Send the command
    //publish_local_target(X,Y,Z);
    //publish_global_target(lat, lon, alt);

   
    if (positionMode == positionModeEnum::global)
    {
        publish_global_target(lat, lon, alt);
    }
    else
    {
        publish_local_target(X,Y,Z);
    }


    if ((_counter % 200) == 0) {
        switch (positionMode)
        {
        case (positionModeEnum::local) :
            positionMode = global;
            ROS_INFO("Switching to Global Positioning Mode");
            break;

            case (positionModeEnum::global) :
            positionMode = local;
            ROS_INFO("Switching to Local  Positioning Mode");
            break;
        
        default:
            break;
        }
    }

    

    // Debugging
    //printf("Commanded Position: (Lat: %6.2f, Lon: %6.2f, Alt: %6.2f)\n",
    //   lat, lon, alt);

    // Give ROS time to do it's thing
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
