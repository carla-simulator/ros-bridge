#!/bin/bash
autopep8 carla_ros_bridge/src/carla_ros_bridge/*.py --in-place --max-line-length=100
autopep8 carla_ackermann_control/src/carla_ackermann_control/*.py --in-place --max-line-length=100
autopep8 carla_ego_vehicle/src/carla_ego_vehicle/*.py --in-place --max-line-length=100
autopep8 carla_waypoint_publisher/src/carla_waypoint_publisher/*.py --in-place --max-line-length=100
autopep8 rqt_carla_control/src/rqt_carla_control/*.py --in-place --max-line-length=100
autopep8 carla_ad_agent/src/carla_ad_agent/*.py --in-place --max-line-length=100
autopep8 carla_ros_scenario_runner/src/carla_ros_scenario_runner/*.py --in-place --max-line-length=100
autopep8 carla_twist_to_control/src/carla_twist_to_control/*.py --in-place --max-line-length=100
autopep8 carla_spectator_camera/src/carla_spectator_camera/*.py --in-place --max-line-length=100
autopep8 carla_infrastructure/src/carla_infrastructure/*.py --in-place --max-line-length=100

pylint --rcfile=.pylintrc carla_ackermann_control/src/carla_ackermann_control/ carla_ros_bridge/src/carla_ros_bridge/ carla_ego_vehicle/src/carla_ego_vehicle/ carla_waypoint_publisher/src/carla_waypoint_publisher/ rqt_carla_control/src/rqt_carla_control/ carla_ad_agent/src/carla_ad_agent/ carla_ros_scenario_runner/src/carla_ros_scenario_runner/ carla_twist_to_control/src/carla_twist_to_control/ carla_spectator_camera/src/carla_spectator_camera/ carla_infrastructure/src/carla_infrastructure/
