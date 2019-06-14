#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy

import tf

from sensor_msgs.point_cloud2 import create_cloud_xyz32

from carla_ros_bridge.sensor import Sensor
import carla_ros_bridge.transforms as trans


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        if topic_prefix is None:
            topic_prefix = 'lidar'
        super(Lidar, self).__init__(carla_actor=carla_actor,
                                    parent=parent,
                                    topic_prefix=topic_prefix,
                                    append_role_name_topic_postfix=append_role_name_topic_postfix)

    def get_tf_msg(self):
        """
        Function (override) to modify the tf messages sent by this lidar.

        The lidar transformation has to be altered:
        for some reasons lidar sends already a rotated cloud,
        so herein, we need to ignore pitch and roll

        :return: the filled tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Lidar, self).get_tf_msg()
        rotation = tf_msg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        dummy_roll, dummy_pitch, yaw = tf.transformations.euler_from_quaternion(
            quat)
        # set roll and pitch to zero
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(
            quat)
        return tf_msg

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(use_parent_frame=False)

        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 3), 3))
        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        # we need a copy here, because the data are read only in carla numpy
        # array
        lidar_data = -lidar_data
        # we also need to permute x and y
        lidar_data = lidar_data[..., [1, 0, 2]]
        point_cloud_msg = create_cloud_xyz32(header, lidar_data)
        self.publish_ros_message(
            self.topic_name() + "/point_cloud", point_cloud_msg)
