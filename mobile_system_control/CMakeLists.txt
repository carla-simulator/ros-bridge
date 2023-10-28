cmake_minimum_required(VERSION 3.0.2)
project(mobile_system_control)

find_package(catkin REQUIRED COMPONENTS
  carla_msgs
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mobile_system_control
 CATKIN_DEPENDS carla_msgs roscpp std_msgs geometry_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mobile_system_control 
  src/mobile_system_control/main.cpp
  src/mobile_system_control/mobile_system_control.cpp
)
target_link_libraries(mobile_system_control 
  ${catkin_LIBRARIES}
)