cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_scenario_runner)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS roslaunch)

  catkin_python_setup()

  if(CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)
  endif()

  catkin_package()

  catkin_install_python(
    PROGRAMS
    src/carla_ros_scenario_runner/carla_ros_scenario_runner_node.py
    src/carla_ros_scenario_runner/application_runner.py
    src/carla_ros_scenario_runner/scenario_runner_runner.py
    src/carla_ros_scenario_runner/ros_vehicle_control.py
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

elseif(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  ament_export_dependencies(rclpy)

  # Install launch files.
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

  ament_package()

endif()
