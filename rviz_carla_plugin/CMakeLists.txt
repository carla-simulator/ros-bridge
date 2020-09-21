cmake_minimum_required(VERSION 3.1)
project(rviz_carla_plugin)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS rviz carla_msgs nav_msgs
                                        carla_ros_scenario_runner_types)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(SRC_FILES src/drive_widget.cpp src/indicator_widget.cpp
              src/carla_control_panel.cpp)

qt5_add_resources(SRC_FILES rviz_carla_plugin.qrc)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
