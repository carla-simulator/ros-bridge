#!/bin/bash
autopep8 src/carla_ros_bridge/*.py --in-place --max-line-length=100
pylint --rcfile=../.pylintrc src/carla_ros_bridge/
