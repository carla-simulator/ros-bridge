#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Class to handle Pseudo Actors (that are not existing in Carla world)
"""

import numpy as np

import ros_compatibility as roscomp

from std_msgs.msg import Header


class PseudoActor(object):

    """
    Generic base class for Pseudo actors (that are not existing in Carla world)
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.PseudoActor
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        :param node: node-handle
        :type node: CompatibleNode
        """
        self.uid = uid
        self.name = name
        self.parent = parent
        self.node = node

        if self.uid is None:
            raise TypeError("Actor ID is not set")

        if self.uid > np.iinfo(np.uint32).max:
            raise ValueError("Actor ID exceeds maximum supported value '{}'".format(self.uid))

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.parent = None

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        raise NotImplementedError(
            "The pseudo actor is missing a blueprint name")

    def get_msg_header(self, frame_id=None, timestamp=None):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        if frame_id:
            header.frame_id = frame_id
        else:
            header.frame_id = self.get_prefix()

        if not timestamp:
            timestamp = self.node.get_time()
        header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)
        return header

    def get_prefix(self):
        """
        get the fully qualified prefix of object
        :return: prefix
        :rtype: string
        """
        if self.parent is not None:
            return self.parent.get_prefix() + "/" + self.name
        else:
            return self.name

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/carla/" + self.get_prefix()

    def update(self, frame, timestamp):
        """
        Function to update this object. Derived classes can add code.
        """
        pass
