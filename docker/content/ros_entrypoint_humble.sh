#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
source "/opt/carla-ros-bridge/install/setup.bash"

exec "$@"
