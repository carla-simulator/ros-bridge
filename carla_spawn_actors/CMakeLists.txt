cmake_minimum_required(VERSION 2.8.3)
project(carla_spawn_actors)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/carla_spawn_actors/carla_spawn_actors.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/carla_spawn_actors/set_initial_pose.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
