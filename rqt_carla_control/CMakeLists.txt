cmake_minimum_required(VERSION 2.8.3)
project(rqt_carla_control)

find_package(catkin REQUIRED COMPONENTS rospy rqt_gui_py roslaunch)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy rqt_gui_py)

catkin_install_python(PROGRAMS src/rqt_carla_control/rqt_carla_control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY resource/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource)

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
