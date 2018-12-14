# 
# Copyright (c) 2018 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
# This represents a vehicle control message sent to CARLA simulator

Header header

# The CARLA vehicle control data

# 0. <= throttle <= 1.
float32 throttle

# -1. <= steer <= 1.
float32 steer

# 0. <= brake <= 1.
float32 brake

# hand_brake 0 or 1
bool hand_brake

# reverse 0 or 1
bool reverse