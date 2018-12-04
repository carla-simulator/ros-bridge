cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_bridge)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

include_directories(
   ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   src/carla_ros_bridge/client.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   client.launch
   client_with_rviz.launch
   test/ros_bridge_client.test
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
   config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

