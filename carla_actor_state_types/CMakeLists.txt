project(carla_actor_state_types)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 2.8.3)

  find_package(catkin REQUIRED COMPONENTS message_generation nav_msgs)

  add_service_files(DIRECTORY srv FILES GetActorState.srv)

  add_message_files(DIRECTORY msg FILES CarlaActorState.msg)

  generate_messages(DEPENDENCIES nav_msgs carla_msgscarla_waypoint_types)

  catkin_package(CATKIN_DEPENDS nav_msgs carla_msgs carla_waypoint_types)

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
  find_package(carla_msgs REQUIRED)
  find_package(carla_waypoint_types REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    msg/CarlaActorState.msg
    srv/GetActorState.srv
    DEPENDENCIES
    nav_msgs
    carla_msgs
    carla_waypoint_types
    ADD_LINTER_TESTS)

  ament_export_dependencies(rosidl_default_runtime)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

endif()
