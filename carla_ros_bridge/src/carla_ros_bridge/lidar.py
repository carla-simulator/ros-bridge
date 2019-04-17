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

from carla_ros_bridge.sensor import Sensor


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, carla_actor, parent, binding):
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
        super(Lidar, self).__init__(carla_actor=carla_actor,
                                    parent=parent,
                                    binding=binding,
                                    topic_prefix='lidar/' + carla_actor.attributes.get('role_name'))

    def publish_transform(self):
        """
        The lidar transformation has to be altered:
        for some reasons lidar sends already a rotated cloud,
        so herein, we need to ignore pitch and roll

        """
        transform = self.current_sensor_data.transform
        transform.roll = 0
        transform.pitch = 0
        self.get_binding().publish_transform(
            self.get_topic_prefix(),
            self.current_sensor_data.transform)

    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        self.get_binding().publish_lidar(
            self.get_topic_prefix(),
            carla_lidar_measurement)
