project(carla_waypoint_types)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 2.8.3)

  find_package(catkin REQUIRED COMPONENTS message_generation nav_msgs)

  add_service_files(DIRECTORY srv FILES GetWaypoint.srv GetActorWaypoint.srv)

  add_message_files(DIRECTORY msg FILES CarlaWaypoint.msg)

  generate_messages(DEPENDENCIES nav_msgs)

  catkin_package(CATKIN_DEPENDS nav_msgs)

elseif(${ROS_VERSION} EQUAL 2)

  cmake_minimum_required(VERSION 3.5)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    msg/CarlaWaypoint.msg
    srv/GetWaypoint.srv
    srv/GetActorWaypoint.srv
    DEPENDENCIES
    nav_msgs
    ADD_LINTER_TESTS)

  ament_export_dependencies(rosidl_default_runtime)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

endif()
