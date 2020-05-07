cmake_minimum_required(VERSION 2.8.3)
project(carla_ego_vehicle)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/carla_ego_vehicle/carla_ego_vehicle.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
