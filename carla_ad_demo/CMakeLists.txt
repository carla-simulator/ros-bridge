cmake_minimum_required(VERSION 2.8.3)
project(carla_ad_demo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

roslaunch_add_file_check(launch)

catkin_package()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
