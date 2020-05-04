cmake_minimum_required(VERSION 2.8.3)
project(carla_ros_scenario_runner_types)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_service_files(DIRECTORY srv FILES ExecuteScenario.srv)

add_message_files(DIRECTORY msg FILES CarlaScenario.msg CarlaScenarioList.msg
                  CarlaScenarioRunnerStatus.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()
