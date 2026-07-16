#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import ros_compatibility as roscomp
import numpy as np

from std_msgs.msg import Header

"""
Base Classes to handle Actor objects
"""

class Actor(object):

    """
    Generic base class for all carla actors
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_client.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.uid = uid
        self.name = name
        self.parent = parent
        self.node = node
        self.carla_actor = carla_actor
        self.carla_actor_id = carla_actor.id

        if self.uid is None:
            raise TypeError("Actor ID is not set")

        if self.uid > np.iinfo(np.uint32).max:
            raise ValueError("Actor ID exceeds maximum supported value '{}'".format(self.uid))
            

    def destroy(self):
        """
        Function (override) to destroy this object.
        Remove the reference to the carla.Actor object.
        :return:
        """
        self.carla_actor = None
        self.parent = None

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

    def get_id(self):
        """
        Getter for the carla_id of this.
        :return: unique carla_id of this object
        :rtype: int64
        """
        return self.carla_actor_id
