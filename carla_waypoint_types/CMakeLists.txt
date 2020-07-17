cmake_minimum_required(VERSION 2.8.3)
project(carla_waypoint_types)

find_package(catkin REQUIRED COMPONENTS message_generation nav_msgs)

add_service_files(DIRECTORY srv FILES GetWaypoint.srv GetActorWaypoint.srv)

add_message_files(DIRECTORY msg FILES CarlaWaypoint.msg)

generate_messages(DEPENDENCIES nav_msgs)

catkin_package(CATKIN_DEPENDS nav_msgs)
