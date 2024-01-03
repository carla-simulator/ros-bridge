cmake_minimum_required(VERSION 3.5)
project(carla_ad_demo)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  ament_export_dependencies(rclpy)

  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

  ament_package()

endif()
