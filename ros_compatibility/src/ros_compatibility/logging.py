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

    import rospy

    def logdebug(msg):
        rospy.logdebug(msg)

    def loginfo(msg):
        rospy.loginfo(msg)

    def logwarn(msg):
        rospy.logwarn(msg)

    def logerr(msg):
        rospy.logerr(msg)

    def logfatal(msg):
        rospy.logfatal(msg)

elif ROS_VERSION == 2:

    import rclpy

    def logdebug(msg):
        rclpy.logging.get_logger("default").debug(msg)

    def loginfo(msg):
        rclpy.logging.get_logger("default").info(msg)

    def logwarn(msg):
        rclpy.logging.get_logger("default").warn(msg)

    def logerr(msg):
        rclpy.logging.get_logger("default").error(msg)

    def logfatal(msg):
        rclpy.logging.get_logger("default").fatal(msg)
