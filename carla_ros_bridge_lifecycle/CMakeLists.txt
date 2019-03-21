cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_bridge_lifecycle)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  carla_ros_bridge
)

catkin_package()

include_directories(
   ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   src/carla_ros_bridge_lifecycle/carla_ros_bridge_lifecycle.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   launch/challenge.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

