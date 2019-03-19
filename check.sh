#!/bin/bash
autopep8 carla_ros_bridge/src/carla_ros_bridge/*.py --in-place --max-line-length=100
autopep8 carla_ros_bridge_lifecycle/src/*.py --in-place --max-line-length=100
autopep8 carla_ackermann_control/src/*.py --in-place --max-line-length=100

pylint --rcfile=.pylintrc carla_ackermann_control/src/ carla_ros_bridge/src/carla_ros_bridge/ carla_ros_bridge_lifecycle/src/
