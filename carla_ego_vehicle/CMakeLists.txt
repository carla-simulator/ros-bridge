cmake_minimum_required(VERSION 2.8.3)
project(carla_ego_vehicle)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        rospy
)

catkin_install_python(PROGRAMS
  src/carla_ego_vehicle/carla_example_ego_vehicle.py
  src/carla_ego_vehicle/carla_ego_vehicle_base.py
  src/carla_ego_vehicle/carla_ros_manual_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
