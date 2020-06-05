project(ainstein_radar_msgs)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

set(CMAKE_CXX_STANDARD 11)

set(MSG_FILES
    # RadarInfo.msg
    RadarAlarm.msg
    RadarAlarmArray.msg
    RadarAlarmStamped.msg
    RadarTarget.msg
    RadarTargetArray.msg
    RadarTargetStamped.msg
    )

if (${ROS_VERSION} EQUAL 1)
    cmake_minimum_required(VERSION 3.0.0)

    find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
    )

    add_message_files(
        DIRECTORY msg
        FILES
        ${MSG_FILES}
        RadarInfo.msg
    )

    generate_messages(
    DEPENDENCIES
    std_msgs
    )

    catkin_package(
    CATKIN_DEPENDS message_runtime
    )

    include_directories(
    ${catkin_INCLUDE_DIRS}
    )
    
    install(TARGETS
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

elseif (${ROS_VERSION} EQUAL 2)
    cmake_minimum_required(VERSION 3.5)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    
    # Apend "msg/" to each file name
    set(TEMP_LIST "")
    foreach(MSG_FILE ${MSG_FILES})
        list(APPEND TEMP_LIST "msg/${MSG_FILE}")
    endforeach()
    set(MSG_FILES ${TEMP_LIST})

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        "msg/RadarInfo2.msg"
        DEPENDENCIES builtin_interfaces std_msgs
        ADD_LINTER_TESTS
    )

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()
    
    ament_export_dependencies(rosidl_default_runtime)

    ament_package()
endif()