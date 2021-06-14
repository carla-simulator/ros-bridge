#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla sensors
"""

from __future__ import print_function

import ctypes
import os
try:
    import queue
except ImportError:
    import Queue as queue
import struct
import sys
from abc import abstractmethod
from threading import Lock

import carla_common.transforms as trans
import ros_compatibility as roscomp
import tf2_ros

from carla_ros_bridge.actor import Actor

from sensor_msgs.msg import PointCloud2, PointField

ROS_VERSION = roscomp.get_ros_version()

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


class Sensor(Actor):

    """
    Actor implementation details for sensors
    """

    def __init__(self,  # pylint: disable=too-many-arguments
                 uid,
                 name,
                 parent,
                 relative_spawn_pose,
                 node,
                 carla_actor,
                 synchronous_mode,
                 is_event_sensor=False,  # only relevant in synchronous_mode:
                 # if a sensor only delivers data on special events,
                 # do not wait for it. That means you might get data from a
                 # sensor, that belongs to a different frame
                 ):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        super(Sensor, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     node=node,
                                     carla_actor=carla_actor)

        self.relative_spawn_pose = relative_spawn_pose
        self.synchronous_mode = synchronous_mode
        self.queue = queue.Queue()
        self.next_data_expected_time = None
        self.sensor_tick_time = None
        self.is_event_sensor = is_event_sensor
        self._callback_active = Lock()
        try:
            self.sensor_tick_time = float(carla_actor.attributes["sensor_tick"])
            node.logdebug("Sensor tick time is {}".format(self.sensor_tick_time))
        except (KeyError, ValueError):
            self.sensor_tick_time = None

        if ROS_VERSION == 1:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        elif ROS_VERSION == 2:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    def get_ros_transform(self, pose, timestamp):
        if self.synchronous_mode:
            if not self.relative_spawn_pose:
                self.node.logwarn("{}: No relative spawn pose defined".format(self.get_prefix()))
                return
            pose = self.relative_spawn_pose
            child_frame_id = self.get_prefix()
            if self.parent is not None:
                frame_id = self.parent.get_prefix()
            else:
                frame_id = "map"

        else:
            child_frame_id = self.get_prefix()
            frame_id = "map"

        transform = tf2_ros.TransformStamped()
        transform.header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w

        return transform

    def publish_tf(self, pose, timestamp):
        transform = self.get_ros_transform(pose, timestamp)
        try:
            self._tf_broadcaster.sendTransform(transform)
        except roscomp.exceptions.ROSException:
            if roscomp.ok():
                self.node.logwarn("Sensor {} failed to send transform.".format(self.uid))

    def listen(self):
        self.carla_actor.listen(self._callback_sensor_data)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Stop listening to the carla.Sensor actor.
        Finally forward call to super class.

        :return:
        """
        self._callback_active.acquire()
        if self.carla_actor.is_listening:
            self.carla_actor.stop()
        super(Sensor, self).destroy()

    def _callback_sensor_data(self, carla_sensor_data):
        """
        Callback function called whenever new sensor data is received

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        if not self._callback_active.acquire(False):
            # if acquire fails, sensor is currently getting destroyed
            return
        if self.synchronous_mode:
            if self.sensor_tick_time:
                self.next_data_expected_time = carla_sensor_data.timestamp + \
                    float(self.sensor_tick_time)
            self.queue.put(carla_sensor_data)
        else:
            self.publish_tf(trans.carla_transform_to_ros_pose(
                carla_sensor_data.transform), carla_sensor_data.timestamp)
            try:
                self.sensor_data_updated(carla_sensor_data)
            except roscomp.exceptions.ROSException:
                if roscomp.ok():
                    self.node.logwarn(
                        "Sensor {}: Error while executing sensor_data_updated().".format(self.uid))
        self._callback_active.release()

    @abstractmethod
    def sensor_data_updated(self, carla_sensor_data):
        """
        Pure-virtual function to transform the received carla sensor data
        into a corresponding ROS message

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        raise NotImplementedError(
            "This function has to be implemented by the derived classes")

    def _update_synchronous_event_sensor(self, frame, timestamp):
        while True:
            try:
                carla_sensor_data = self.queue.get(block=False)
                if carla_sensor_data.frame != frame:
                    self.node.logwarn("{}({}): Received event for frame {}"
                                      " (expected {}). Process it anyways.".format(
                                          self.__class__.__name__, self.get_id(),
                                          carla_sensor_data.frame, frame))
                self.node.logdebug("{}({}): process {}".format(
                    self.__class__.__name__, self.get_id(), frame))
                self.publish_tf(trans.carla_transform_to_ros_pose(
                    carla_sensor_data.transform), timestamp)
                self.sensor_data_updated(carla_sensor_data)
            except queue.Empty:
                return

    def _update_synchronous_sensor(self, frame, timestamp):
        while not self.next_data_expected_time or \
            (not self.queue.empty() or
             self.next_data_expected_time and
             self.next_data_expected_time < timestamp):
            while True:
                try:
                    carla_sensor_data = self.queue.get(timeout=1.0)
                    if carla_sensor_data.frame == frame:
                        self.node.logdebug("{}({}): process {}".format(self.__class__.__name__,
                                                                       self.get_id(), frame))
                        self.publish_tf(trans.carla_transform_to_ros_pose(
                            carla_sensor_data.transform), timestamp)
                        self.sensor_data_updated(carla_sensor_data)
                        return
                    elif carla_sensor_data.frame < frame:
                        self.node.logwarn("{}({}): skipping old frame {}, expected {}".format(
                            self.__class__.__name__,
                            self.get_id(),
                            carla_sensor_data.frame,
                            frame))
                except queue.Empty:
                    if roscomp.ok():
                        self.node.logwarn("{}({}): Expected Frame {} not received".format(
                            self.__class__.__name__, self.get_id(), frame))
                    return

    def update(self, frame, timestamp):
        if self.synchronous_mode:
            if self.is_event_sensor:
                self._update_synchronous_event_sensor(frame, timestamp)
            else:
                self._update_synchronous_sensor(frame, timestamp)

        super(Sensor, self).update(frame, timestamp)


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
            print('Skipping unknown PointField datatype [{}]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


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
