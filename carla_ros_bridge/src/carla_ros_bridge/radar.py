#!/usr/bin/env python

#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla Radar
"""

import rospy

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField

from sensor_msgs.point_cloud2 import create_cloud

from carla_ros_bridge.sensor import Sensor


class Radar(Sensor):

    """
    Actor implementation details of Carla RADAR
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Radar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.radar_publisher = rospy.Publisher(self.get_topic_prefix(),
                                               PointCloud2,
                                               queue_size=10)
        self.listen()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_radar_measurement):
        """
        Function to transform the a received Radar measurement into a ROS message
        :param carla_radar_measurement: carla Radar measurement object
        :type carla_radar_measurement: carla.RadarMeasurement
        """
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('Range', 12, PointField.FLOAT32, 1),
                  PointField('Velocity', 16, PointField.FLOAT32, 1),
                  PointField('AzimuthAngle', 20, PointField.FLOAT32, 1),
                  PointField('ElevationAngle', 28, PointField.FLOAT32, 1)]
        points = []
        for detection in carla_radar_measurement:
            points.append([detection.depth * np.cos(-detection.azimuth) * np.cos(detection.altitude),
                           detection.depth * np.sin(-detection.azimuth) *
                           np.cos(detection.altitude),
                           detection.depth * np.sin(detection.altitude),
                           detection.depth, detection.velocity, detection.azimuth, detection.altitude])
        radar_msg = create_cloud(self.get_msg_header(
            timestamp=carla_radar_measurement.timestamp), fields, points)
        self.radar_publisher.publish(radar_msg)
