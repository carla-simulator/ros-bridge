# 
# Copyright (c) 2018 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
# This represents the control state variables of the ego vehicle control

# the current control state
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
