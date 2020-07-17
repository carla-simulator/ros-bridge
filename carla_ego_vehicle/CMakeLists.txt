cmake_minimum_required(VERSION 2.8.3)
project(carla_ego_vehicle)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

  catkin_python_setup()

  roslaunch_add_file_check(launch)

  catkin_package(CATKIN_DEPENDS rospy)

  catkin_install_python(PROGRAMS src/carla_ego_vehicle/carla_ego_vehicle.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  install(DIRECTORY config/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

elseif(${ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  ament_export_dependencies(rclpy)

  # Install launch files.
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

  ament_package()

endif()
