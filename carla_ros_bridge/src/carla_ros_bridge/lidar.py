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

from __future__ import print_function
import numpy

import ctypes
import struct

import os
ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
elif ROS_VERSION == 2:
    from transformations.transformations import euler_from_quaternion, quaternion_from_euler
else:
    raise NotImplementedError("Make sure you have a valid ROS_VERSION env variable set.")

from sensor_msgs.msg import PointCloud2, PointField

from carla_ros_bridge.sensor import Sensor
import carla_ros_bridge.transforms as trans

_DATATYPES = {}
_DATATYPES[PointField.FLOAT32] = ('f', 4)


class Lidar(Sensor):
    """
    Actor implementation details for lidars
    """

    def __init__(self, carla_actor, parent, communication, synchronous_mode, sensor_name="Lidar"):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        """
        super(Lidar, self).__init__(carla_actor=carla_actor, parent=parent,
                                    communication=communication, synchronous_mode=synchronous_mode,
                                    prefix='lidar/' + carla_actor.attributes.get('role_name'),
                                    sensor_name=sensor_name)

    def get_ros_transform(self, transform=None, frame_id=None, child_frame_id=None):
        """
        Function (override) to modify the tf messages sent by this lidar.

        The lidar transformation has to be altered:
        for some reasons lidar sends already a rotated cloud,
        so herein, we need to ignore pitch and roll

        :return: the filled tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Lidar, self).get_ros_transform(transform, frame_id, child_frame_id)

        rotation = tf_msg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        dummy_roll, dummy_pitch, yaw = euler_from_quaternion(quat)
        # set roll and pitch to zero
        quat = quaternion_from_euler(0, 0, yaw)
        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(quat)
        return tf_msg

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header()

        lidar_data = numpy.frombuffer(carla_lidar_measurement.raw_data, dtype=numpy.float32)
        lidar_data = numpy.reshape(lidar_data, (int(lidar_data.shape[0] / 3), 3))
        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        # we need a copy here, because the data are read only in carla numpy
        # array
        lidar_data = -lidar_data
        # we also need to permute x and y
        lidar_data = lidar_data[..., [1, 0, 2]]

        # -- taken from http://docs.ros.org/indigo/api/sensor_msgs/html/point__cloud2_8py_source.html

        point_field_x_msg = PointField()
        point_field_x_msg.name = "x"
        point_field_x_msg.offset = 0
        point_field_x_msg.datatype = PointField.FLOAT32
        point_field_x_msg.count = 1

        point_field_y_msg = PointField()
        point_field_y_msg.name = "y"
        point_field_y_msg.offset = 4
        point_field_y_msg.datatype = PointField.FLOAT32
        point_field_y_msg.count = 1

        point_field_z_msg = PointField()
        point_field_z_msg.name = "z"
        point_field_z_msg.offset = 8
        point_field_z_msg.datatype = PointField.FLOAT32
        point_field_z_msg.count = 1

        fields = [point_field_x_msg, point_field_y_msg, point_field_z_msg]

        cloud_struct = struct.Struct(_get_struct_fmt(False, fields))
        buff = ctypes.create_string_buffer(cloud_struct.size * len(lidar_data))

        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into

        offset = 0
        for pt in lidar_data:
            pack_into(buff, offset, *pt)
            offset += point_step

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(lidar_data)
        point_cloud_msg.is_dense = False
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.fields = fields
        point_cloud_msg.point_step = cloud_struct.size
        point_cloud_msg.row_step = cloud_struct.size * len(lidar_data)
        point_cloud_msg.data = buff.raw

        # --

        self.publish_message(self.get_topic_prefix() + "/point_cloud", point_cloud_msg)


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