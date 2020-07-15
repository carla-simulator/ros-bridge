cmake_minimum_required(VERSION 3.1)
project(pcl_recorder)

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp
                                        sensor_msgs roslaunch)

catkin_package()

roslaunch_add_file_check(launch DEPENDENCIES ${PROJECT_NAME}_node)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/PclRecorder.cpp src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_14)

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
