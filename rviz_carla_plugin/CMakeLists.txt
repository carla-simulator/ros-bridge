cmake_minimum_required(VERSION 3.5)
project(rviz_carla_plugin)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(nav_msgs COMPONENTS)
find_package(carla_ros_scenario_runner_types COMPONENTS)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(SRC_FILES src/drive_widget.cpp src/indicator_widget.cpp
              src/carla_control_panel_ROS2.cpp)

qt5_add_resources(SRC_FILES rviz_carla_plugin.qrc)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES}
                      rviz_ogre_vendor::OgreMain)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

pluginlib_export_plugin_description_file(rviz_common
                                          plugin_description_ros2.xml)

ament_target_dependencies(
  rviz_carla_plugin 
  rclcpp 
  carla_msgs 
  nav_msgs 
  carla_ros_scenario_runner_types 
  rviz_common 
  tf2)

ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin_description_ros2.xml DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY icons/ DESTINATION "share/${PROJECT_NAME}")

ament_package()