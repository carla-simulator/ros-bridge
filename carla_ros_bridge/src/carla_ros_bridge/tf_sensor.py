#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a tf sensor
"""

import os

import tf2_ros

import ros_compatibility as roscomp

from carla_ros_bridge.pseudo_actor import PseudoActor

from geometry_msgs.msg import TransformStamped

ROS_VERSION = roscomp.get_ros_version()


class TFSensor(PseudoActor):

    """
    Pseudo tf sensor
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(TFSensor, self).__init__(uid=uid,
                                       name=name,
                                       parent=parent,
                                       node=node)

        if ROS_VERSION == 1:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        elif ROS_VERSION == 2:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.tf"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        self.parent.get_prefix()

        transform = None
        try:
            transform = self.parent.get_current_ros_transform()
        except AttributeError:
            # parent actor disappeared, do not send tf
            self.node.logwarn(
                "TFSensor could not publish transform. Actor {} not found".format(self.parent.uid))
            return

        self._tf_broadcaster.sendTransform(TransformStamped(
            header=self.get_msg_header("map", timestamp=timestamp),
            child_frame_id=self.parent.get_prefix(),
            transform=transform))
