cmake_minimum_required(VERSION 2.8.3)
project(carla_ackermann_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
  ackermann_msgs
  dynamic_reconfigure
  carla_ros_bridge
)

add_message_files(
  FILES
  EgoVehicleControlCurrent.msg
  EgoVehicleControlInfo.msg
  EgoVehicleControlMaxima.msg
  EgoVehicleControlState.msg
  EgoVehicleControlTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  carla_ros_bridge
)

generate_dynamic_reconfigure_options(
   config/EgoVehicleControlParameter.cfg
)

catkin_package()

install(PROGRAMS
  src/carla_ackermann_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
