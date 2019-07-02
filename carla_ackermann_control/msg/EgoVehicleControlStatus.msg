#
# Copyright (c) 2018-2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This represents the control status variables of the ego vehicle control

# the current control status
string status

# speed controller
uint8   speed_control_activation_count
float32 speed_control_accel_delta
float32 speed_control_accel_target

# acceleration controller
float32 accel_control_pedal_delta
float32 accel_control_pedal_target

# borders for lay off pedal
float32 brake_upper_border
float32 throttle_lower_border
