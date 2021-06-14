cmake_minimum_required(VERSION 2.8.3)
project(ros_compatibility)

# Update the policy setting to avoid an error when loading the ament_cmake
# package at the current cmake version level
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy)

  catkin_python_setup()

  catkin_package(CATKIN_DEPENDS rospy)

  include_directories()

endif()
