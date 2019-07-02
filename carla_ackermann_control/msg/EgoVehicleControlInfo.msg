#
# Copyright (c) 2018-2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This represents an info message of the ego vehicle

Header header

# the restrictions
EgoVehicleControlMaxima restrictions

# the target values
EgoVehicleControlTarget target

# the currently measured values
EgoVehicleControlCurrent current

# the current control status
EgoVehicleControlStatus status

# the current control output to CARLA
carla_msgs/CarlaEgoVehicleControl output
