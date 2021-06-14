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

    class CallbackGroup(object):
        pass

    class ReentrantCallbackGroup(CallbackGroup):
        pass

    class MutuallyExclusiveCallbackGroup(CallbackGroup):
        pass


elif ROS_VERSION == 2:

    import rclpy.callback_groups

    class ReentrantCallbackGroup(rclpy.callback_groups.ReentrantCallbackGroup):
        pass

    class MutuallyExclusiveCallbackGroup(rclpy.callback_groups.MutuallyExclusiveCallbackGroup):
        pass
