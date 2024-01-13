cmake_minimum_required(VERSION 3.5)
project(pcl_recorder)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs COMPONENTS)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(
  Boost
  COMPONENTS system
  REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/PclRecorderROS2.cpp src/mainROS2.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2 
  tf2_ros
  tf2_eigen)

target_link_libraries(${PROJECT_NAME}_node ${Boost_SYSTEM_LIBRARY}
                      ${PCL_LIBRARIES})

target_compile_features(
  ${PROJECT_NAME}_node
  PRIVATE cxx_inheriting_constructors cxx_lambdas cxx_auto_type
          cxx_variadic_templates cxx_variable_templates)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()