cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_scenario_runner)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_python_setup()

roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)

catkin_package()

catkin_install_python(
  PROGRAMS
  src/carla_ros_scenario_runner/carla_ros_scenario_runner.py
  src/carla_ros_scenario_runner/application_runner.py
  src/carla_ros_scenario_runner/scenario_runner_runner.py
  src/carla_ros_scenario_runner/ros_vehicle_control.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
