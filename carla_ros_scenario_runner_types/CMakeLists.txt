project(carla_ros_scenario_runner_types)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 2.8.3)

  find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

  add_service_files(DIRECTORY srv FILES ExecuteScenario.srv)

  add_message_files(DIRECTORY msg FILES CarlaScenario.msg CarlaScenarioList.msg
                    CarlaScenarioRunnerStatus.msg)

  generate_messages(DEPENDENCIES geometry_msgs)

  catkin_package()

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
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    msg/CarlaScenario.msg
    msg/CarlaScenarioList.msg
    msg/CarlaScenarioRunnerStatus.msg
    srv/ExecuteScenario.srv
    DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    ADD_LINTER_TESTS)

  ament_export_dependencies(rosidl_default_runtime)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

endif()
