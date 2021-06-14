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

import numpy as np

from carla_ros_bridge.sensor import Sensor, create_cloud

from sensor_msgs.msg import PointCloud2, PointField


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
        :type node: CompatibleNode
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

        self.radar_publisher = node.new_publisher(PointCloud2, self.get_topic_prefix(), qos_profile=10)
        self.listen()

    def destroy(self):
        super(Radar, self).destroy()
        self.node.destroy_publisher(self.radar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_radar_measurement):
        """
        Function to transform the a received Radar measurement into a ROS message
        :param carla_radar_measurement: carla Radar measurement object
        :type carla_radar_measurement: carla.RadarMeasurement
        """
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='Range', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='Velocity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='AzimuthAngle', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='ElevationAngle', offset=28, datatype=PointField.FLOAT32, count=1)]
        points = []
        for detection in carla_radar_measurement:
            points.append([detection.depth * np.cos(detection.azimuth) * np.cos(-detection.altitude),
                           detection.depth * np.sin(-detection.azimuth) *
                           np.cos(detection.altitude),
                           detection.depth * np.sin(detection.altitude),
                           detection.depth, detection.velocity, detection.azimuth, detection.altitude])
        radar_msg = create_cloud(self.get_msg_header(
            timestamp=carla_radar_measurement.timestamp), fields, points)
        self.radar_publisher.publish(radar_msg)
