cmake_minimum_required(VERSION 2.8.3)
project(carla_common)

find_package(catkin)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS src/carla_common/transforms.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
