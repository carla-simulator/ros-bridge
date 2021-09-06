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

    class ROSException(rospy.ROSException):
        pass

    class ROSInterruptException(rospy.ROSInterruptException):
        pass

    class ServiceException(rospy.ServiceException):
        pass


elif ROS_VERSION == 2:

    import rclpy.exceptions

    class ROSException(Exception):
        pass

    class ROSInterruptException(rclpy.exceptions.ROSInterruptException):
        pass

    class ServiceException(Exception):
        pass

