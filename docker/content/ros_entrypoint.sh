#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"
source "/opt/carla/setup.bash"

exec "$@"
