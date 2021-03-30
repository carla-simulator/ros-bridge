cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_bridge)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs
                                          derived_object_msgs tf roslaunch)

  catkin_python_setup()

  catkin_package()

  if(CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch)
    roslaunch_add_file_check(test)
  endif()

  include_directories(${catkin_INCLUDE_DIRS})

  install(PROGRAMS src/carla_ros_bridge/bridge.py
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(FILES src/carla_ros_bridge/CARLA_VERSION
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(FILES test/ros_bridge_client.test
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  if(CATKIN_ENABLE_TESTING AND NOT $ENV{CI})
    find_package(rostest REQUIRED)
    add_rostest(test/ros_bridge_client.test)
  endif()

endif()
