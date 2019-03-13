cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_bridge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  tf
  dynamic_reconfigure
)

catkin_python_setup()

add_message_files(
  FILES
  CarlaVehicleControl.msg
  EgoVehicleControlCurrent.msg
  EgoVehicleControlInfo.msg
  EgoVehicleControlMaxima.msg
  EgoVehicleControlState.msg
  EgoVehicleControlTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
   config/EgoVehicleControlParameter.cfg
)

catkin_package()


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

