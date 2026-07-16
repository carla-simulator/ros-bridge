cmake_minimum_required(VERSION 3.5)
project(pcl_recorder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0074)
  # let find_package() honor <PackageName>_ROOT (e.g. PCL_ROOT), instead of a warning + silently
  # ignoring it; this project's minimum required CMake version predates the policy's introduction.
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  # same as CMP0074, but for the upper-case <PACKAGENAME>_ROOT spelling that pcl_conversions'
  # own FindFLANN.cmake (pulled in transitively below) uses for FLANN_ROOT.
  cmake_policy(SET CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs COMPONENTS)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(
  Boost
  COMPONENTS system
  REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/PclRecorder.cpp src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs
                          pcl_conversions tf2 tf2_ros tf2_eigen)

target_link_libraries(${PROJECT_NAME}_node ${Boost_SYSTEM_LIBRARY}
                      ${PCL_LIBRARIES})

target_compile_features(
  ${PROJECT_NAME}_node
  PRIVATE cxx_inheriting_constructors cxx_lambdas cxx_auto_type
          cxx_variadic_templates cxx_variable_templates)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
