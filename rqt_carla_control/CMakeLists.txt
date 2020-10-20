cmake_minimum_required(VERSION 2.8.3)
project(rqt_carla_control)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy rqt_gui_py roslaunch)

  catkin_python_setup()

  catkin_package(CATKIN_DEPENDS rospy rqt_gui_py)

  catkin_install_python(PROGRAMS src/rqt_carla_control/rqt_carla_control.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY resource/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource)

  install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif(${ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  ament_export_dependencies(rclpy)

  install(DIRECTORY resource/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource)

  install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  ament_package()
endif()
