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

from sensor_msgs.msg import PointCloud2, PointField

import sys
import ctypes
import os
import struct
import numpy as np

# from sensor_msgs.point_cloud2 import create_cloud

_DATATYPES = {}
_DATATYPES[PointField.FLOAT32] = ('f', 4)

from carla_ros_bridge.sensor import Sensor


class Radar(Sensor):
    """
    Actor implementation details of Carla RADAR
    """
    # pylint: disable=too-many-arguments

    def __init__(self, carla_actor, parent, node, synchronous_mode, sensor_name="Radar"):
        """
        Constructor
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Radar, self).__init__(carla_actor=carla_actor, parent=parent,
                                    node=node, synchronous_mode=synchronous_mode,
                                    prefix="radar/" + carla_actor.attributes.get('role_name'),
                                    sensor_name=sensor_name)
        self.radar_publisher = node.new_publisher(
            PointCloud2, self.get_topic_prefix() + "/radar")
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


# http://docs.ros.org/indigo/api/sensor_msgs/html/point__cloud2_8py_source.html

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset)
                  if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

# https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.p

def create_cloud(header, fields, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message.
    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for 
                   that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)
