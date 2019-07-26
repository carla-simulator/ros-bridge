#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
report the carla status
"""

import rospy

from carla_msgs.msg import CarlaStatus


class CarlaStatusPublisher(object):

    """
    report the carla status
    """

    def __init__(self, synchronous_mode, fixed_delta_seconds):
        """
        Constructor

        """
        self.synchronous_mode = synchronous_mode
        self.synchronous_mode_running = True
        self.fixed_delta_seconds = fixed_delta_seconds
        self.frame = 0
        self.carla_status_publisher = rospy.Publisher(
            "/carla/status", CarlaStatus, queue_size=10, latch=True)
        self.publish()

    def publish(self):
        """
        publish the current status

        """
        status_msg = CarlaStatus()
        status_msg.frame = self.frame
        status_msg.synchronous_mode = self.synchronous_mode
        status_msg.synchronous_mode_running = self.synchronous_mode_running
        status_msg.fixed_delta_seconds = self.fixed_delta_seconds
        self.carla_status_publisher.publish(status_msg)

    def set_synchronous_mode_running(self, running):
        """
        set the value 'synchronous_mode_running'
        """
        if self.synchronous_mode_running != running:
            self.synchronous_mode_running = running
            self.publish()

    def set_frame(self, frame):
        """
        set the value 'synchronous_mode_running'
        """
        if self.frame != frame:
            self.frame = frame
            self.publish()
