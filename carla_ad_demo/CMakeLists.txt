cmake_minimum_required(VERSION 2.8.3)
project(carla_ad_demo)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS roslaunch)

  roslaunch_add_file_check(launch)

  catkin_package()

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  install(DIRECTORY config/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

elseif(${ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)

  # Install launch files.
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

  ament_package()

endif()
