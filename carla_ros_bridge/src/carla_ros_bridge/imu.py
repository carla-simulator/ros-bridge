#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla IMU
"""

from sensor_msgs.msg import Imu

from scipy.spatial.transform import Rotation as R

import math

from carla_ros_bridge.sensor import Sensor


class IMU(Sensor):

    """
    Actor implementation details of Carla IMU
    """

    def __init__(self, carla_actor, parent, communication, synchronous_mode):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(IMU, self).__init__(carla_actor=carla_actor,
                                   parent=parent,
                                   communication=communication,
                                   synchronous_mode=synchronous_mode,
                                   prefix="imu/" + carla_actor.attributes.get('role_name'))

    def sensor_data_updated(self, carla_imu_event):
        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(timestamp=carla_imu_event.timestamp)
        imu_msg.linear_acceleration.x = carla_imu_event.accelerometer.x
        imu_msg.linear_acceleration.y = carla_imu_event.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_event.accelerometer.z
        imu_msg.angular_velocity.x = carla_imu_event.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_event.gyroscope.y
        imu_msg.angular_velocity.z = carla_imu_event.gyroscope.z

        imu_rpy = carla_imu_event.transform.rotation
        r = R.from_rotvec([math.radians(imu_rpy.roll), math.radians(imu_rpy.pitch), math.radians(imu_rpy.yaw)]).as_quat()

        imu_msg.orientation.x = r[0]
        imu_msg.orientation.y = r[1]
        imu_msg.orientation.z = r[2]
        imu_msg.orientation.w = r[3]
        self.publish_message( self.get_topic_prefix() + "/imu", imu_msg)

