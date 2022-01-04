cmake_minimum_required(VERSION 3.0.2)
project(ros_uav_command)

# Dependendices
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# Declare a Catkin Package
catkin_package(
  CATKIN_DEPENDS roscpp
)

# Includes
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  include
)

# Libraries 
#   additional functions used in main code
#   not sure why I can only have one here though...
add_library(pkg_libraries
  src/publishers.cpp
  src/subscribers.cpp
  src/services.cpp
  src/helpers.cpp
)

#===== Executables to generate
# Positioning Test
add_executable(testPositioning
  src/testPositioning.cpp
) 

target_link_libraries(testPositioning
  pkg_libraries
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Set rates test
add_executable(setRates
  src/setRates.cpp
) 

target_link_libraries(setRates
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
)

# VRPN dummy test
add_executable(mocapDummy
  src/mocapDummy.cpp
) 

target_link_libraries(mocapDummy
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${catkin_LIBRARIES}
)
