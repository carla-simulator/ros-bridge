cmake_minimum_required(VERSION 2.8.3)
project(carla_ad_agent)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(
  PROGRAMS
  src/carla_ad_agent/carla_ad_agent.py
  src/carla_ad_agent/basic_agent.py
  src/carla_ad_agent/agent.py
  src/carla_ad_agent/local_planner.py
  src/carla_ad_agent/vehicle_pid_controller.py
  src/carla_ad_agent/misc.py
  src/carla_ad_agent/__init__.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
