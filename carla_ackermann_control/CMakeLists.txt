cmake_minimum_required(VERSION 2.8.3)
project(carla_ackermann_control)

# Find catkin macros and libraries
find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             rospy
             std_msgs
             ackermann_msgs
             dynamic_reconfigure
             carla_msgs
             roslaunch)

catkin_python_setup()

add_message_files(
  FILES EgoVehicleControlCurrent.msg EgoVehicleControlInfo.msg
  EgoVehicleControlMaxima.msg EgoVehicleControlStatus.msg
  EgoVehicleControlTarget.msg)

generate_messages(DEPENDENCIES std_msgs carla_msgs)

generate_dynamic_reconfigure_options(config/EgoVehicleControlParameter.cfg)

roslaunch_add_file_check(launch)

catkin_package()

install(PROGRAMS src/carla_ackermann_control/carla_ackermann_control_node.py
                 src/carla_ackermann_control/carla_control_physics.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
