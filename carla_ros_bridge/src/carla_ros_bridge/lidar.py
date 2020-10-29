#!/usr/bin/env python
#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

from __future__ import print_function

import sys
import ctypes
import os
import struct
import numpy

from sensor_msgs.msg import PointCloud2, PointField  # pylint: disable=import-error
import carla_common.transforms as trans
from carla_ros_bridge.sensor import Sensor

from ros_compatibility import quaternion_from_euler, euler_from_quaternion

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    # pylint: disable=import-error,ungrouped-imports
    from sensor_msgs.point_cloud2 import create_cloud

_DATATYPES = {}
_DATATYPES[PointField.FLOAT32] = ('f', 4)


class Lidar(Sensor):
    """
    Actor implementation details for lidars
    """
    # pylint: disable=too-many-arguments

    def __init__(self, carla_actor, parent, node, synchronous_mode, sensor_name="Lidar"):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        """
        super(Lidar, self).__init__(carla_actor=carla_actor, parent=parent,
                                    node=node, synchronous_mode=synchronous_mode,
                                    prefix='lidar/' + carla_actor.attributes.get('role_name'),
                                    sensor_name=sensor_name)

        self.lidar_publisher = node.new_publisher(PointCloud2, self.get_topic_prefix() +
                                                  "/point_cloud")
        self.listen()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header()

        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data[:, 1] *= -1

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        if ROS_VERSION == 1:
            point_cloud_msg = create_cloud(header, fields, lidar_data)

        # -- taken from
        # http://docs.ros.org/indigo/api/sensor_msgs/html/point__cloud2_8py_source.html
        elif ROS_VERSION == 2:
            cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

            buff = ctypes.create_string_buffer(cloud_struct.size * len(lidar_data))

            point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
            offset = 0
            for p in lidar_data:
                pack_into(buff, offset, *p)
                offset += point_step

            point_cloud_msg = PointCloud2(header=header,
                                          height=1,
                                          width=len(lidar_data),
                                          is_dense=False,
                                          is_bigendian=False,
                                          fields=fields,
                                          point_step=cloud_struct.size,
                                          row_step=cloud_struct.size * len(lidar_data),
                                          data=buff.raw)

        # --

        self.lidar_publisher.publish(point_cloud_msg)


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, carla_actor, parent, node, synchronous_mode):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """
        super(SemanticLidar, self).__init__(carla_actor=carla_actor,
                                            parent=parent,
                                            node=node,
                                            synchronous_mode=synchronous_mode,
                                            prefix='semantic_lidar/' + carla_actor.attributes.get('role_name'))

        self.semantic_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix() + "/point_cloud")
        self.listen()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        header = self.get_msg_header()
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('CosAngle', 12, PointField.FLOAT32, 1),
            PointField('ObjIdx', 16, PointField.UINT32, 1),
            PointField('ObjTag', 20, PointField.UINT32, 1),
        ]

        lidar_data = numpy.fromstring(carla_lidar_measurement.raw_data,
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
        self.semantic_lidar_publisher.publish(point_cloud_msg)

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
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt
