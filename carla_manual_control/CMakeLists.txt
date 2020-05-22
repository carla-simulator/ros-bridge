cmake_minimum_required(VERSION 2.8.3)
project(carla_manual_control)

find_package(ament_cmake QUIET)
if ( ament_cmake_FOUND )
	find_package(rclpy REQUIRED)

	ament_export_dependencies(rclpy)
	ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
	
	find_package(catkin REQUIRED COMPONENTS
	  rospy
	  roslaunch
	)

	catkin_python_setup()

	roslaunch_add_file_check(launch)

	catkin_package(
	  CATKIN_DEPENDS
	  rospy
	)

	catkin_install_python(PROGRAMS
	  src/carla_manual_control/carla_manual_control.py
	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

	install(DIRECTORY launch/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	)
endif()
