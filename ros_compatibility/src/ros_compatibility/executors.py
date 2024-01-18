#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

from ros_compatibility.core import get_ros_version
ROS_VERSION  = get_ros_version()

import rclpy.executors

class SingleThreadedExecutor(rclpy.executors.SingleThreadedExecutor):
    pass

class MultiThreadedExecutor(rclpy.executors.MultiThreadedExecutor):
    pass

