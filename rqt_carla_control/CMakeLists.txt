cmake_minimum_required(VERSION 2.8.3)
project(rqt_carla_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
ament_export_dependencies(rclpy)

install(DIRECTORY resource/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource)

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_package()
