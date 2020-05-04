cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_bridge)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs
                                        derived_object_msgs tf roslaunch)

catkin_python_setup()

catkin_package()

roslaunch_add_file_check(launch)
roslaunch_add_file_check(test)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/carla_ros_bridge/bridge.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES test/ros_bridge_client.test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING AND NOT $ENV{CI})
  find_package(rostest REQUIRED)
  add_rostest(test/ros_bridge_client.test)
endif()
