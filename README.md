# ros_uav_command
A collection of ROS nodes to test mavros interfacing between ROS and a UAV.
Designed for Ardupilot, but will work with PX4 with minimal change.

```
https://github.com/AndersonRayner/ros_uav_command.git
```

## Dependencies
Requires mavros and mavros-extras
```
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
```

## Building the Code
### Cloning the Package
Add this repo to any pre-exisiting ros workspace either as a static version (using `git clone`) or as a submodule (using `git submodule add`).

### Building the Code
The code should build exactly the same as any regular ros package, though has only been tested with catkin_tools and ros noetic.

## Nodes
### Set Rates
Demonstration of how to write a script to set the telemetry rates for a flight controller.
The node sends the commands via the `COMMAND_LONG` interface, waits for confirmation that it was successful, then exists the code.
```
roslaunch ros_uav_command setRates.launch
```

### Test Positioning
Demonstration of controlling a UAV using ROS.
Sends the setpoints via the `setpoint_raw` command, allowing a deeper level of control.
Cycles through control via `MAV_FRAME_LOCAL_NED`, `MAV_FRAME_GLOBAL_RELATIVE_ALT`, and raw attitude commands.
```
roslaunch ros_uav_command testPositioning.launch
```

### MOCAP Dummy
A dummy script for generating fake motion capture data, emulating a VICON/Optitrack system.
This allows testing of codes to make sure that flight controller EKFs accept mocap data.
```
roslaunch ros_uav_command mocapDummy.launch
```

## Contributions
As always, PRs welcome!
