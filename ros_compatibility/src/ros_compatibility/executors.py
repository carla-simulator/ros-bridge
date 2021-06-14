#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

from ros_compatibility.core import get_ros_version
ROS_VERSION  = get_ros_version()

if ROS_VERSION == 1:

    class Executor(object):
        def add_node(self, node):
            pass

    class SingleThreadedExecutor(Executor):
        pass

    class MultiThreadedExecutor(Executor):
        pass


elif ROS_VERSION == 2:

    import rclpy.executors

    class SingleThreadedExecutor(rclpy.executors.SingleThreadedExecutor):
        pass

    class MultiThreadedExecutor(rclpy.executors.MultiThreadedExecutor):
        pass

