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

from std_msgs.msg import Header
import rospy
import numpy as np


class PseudoActor(object):

    """
    Generic base class for Pseudo actors (that are not existing in Carla world)
    """

    def __init__(self, uid, parent, node, prefix=None):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param parent: the parent of this
        :type parent: carla_ros_bridge.PseudoActor
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """
        self.uid = uid
        if self.uid is None:
            raise TypeError("Actor ID is not set")

        if self.uid > np.iinfo(np.uint32).max:
            raise ValueError("Actor ID exceeds maximum supported value '{}'".format(self.uid))

        self.parent = parent
        if self.parent:
            self.parent_id = parent.get_id()
        else:
            self.parent_id = None

        self.node = node

        # Concatenate the onw prefix to the the parent topic name if not empty.
        self.prefix = ""
        if parent:
            self.prefix = parent.get_prefix()
        if prefix:
            if self.prefix:
                self.prefix += "/"
            self.prefix += prefix

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
        if timestamp:
            header.stamp = rospy.Time.from_sec(timestamp)
        else:
            header.stamp = self.node.ros_timestamp
        return header

    def get_parent_id(self):
        """
        Getter for the carla_id of the parent.
        :return: unique carla_id of the parent of this child
        :rtype: int64
        """
        return self.parent_id

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/carla/" + self.prefix

    def get_prefix(self):
        """
        get the fully qualified prefix of object
        :return: prefix
        :rtype: string
        """
        return self.prefix

    def update(self, frame, timestamp):
        """
        Function to update this object. Derived classes can add code.
        """
        pass
