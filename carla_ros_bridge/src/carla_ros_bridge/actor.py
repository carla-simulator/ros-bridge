#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Classes to handle Actor objects
"""

import numpy as np
from geometry_msgs.msg import TransformStamped

from carla_ros_bridge.pseudo_actor import PseudoActor
import carla_common.transforms as trans


class Actor(PseudoActor):

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
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(Actor, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    node=node)
        self.carla_actor = carla_actor
        self.carla_actor_id = carla_actor.id

    def destroy(self):
        """
        Function (override) to destroy this object.
        Remove the reference to the carla.Actor object.
        :return:
        """
        self.carla_actor = None
        super(Actor, self).destroy()

    def get_current_ros_pose(self):
        """
        Function to provide the current ROS pose

        :return: the ROS pose of this actor
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_ros_pose(
            self.carla_actor.get_transform())

    def get_current_ros_twist_rotated(self):
        """
        Function to provide the current ROS twist rotated

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity(),
            self.carla_actor.get_transform().rotation)

    def get_current_ros_twist(self):
        """
        Function to provide the current ROS twist

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity())

    def get_current_ros_accel(self):
        """
        Function to provide the current ROS accel

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_acceleration_to_ros_accel(
            self.carla_actor.get_acceleration())

    def get_id(self):
        """
        Getter for the carla_id of this.
        :return: unique carla_id of this object
        :rtype: int64
        """
        return self.carla_actor_id

    def get_ros_transform(self, transform=None, frame_id=None, child_frame_id=None):
        """
        Function to provide the current ROS transform

        :return: the ROS transfrom
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = TransformStamped()
        if frame_id:
            tf_msg.header = self.get_msg_header(frame_id)
        else:
            tf_msg.header = self.get_msg_header("map")
        if child_frame_id:
            tf_msg.child_frame_id = child_frame_id
        else:
            tf_msg.child_frame_id = self.get_prefix()
        if transform:
            tf_msg.transform = transform
        else:
            tf_msg.transform = trans.carla_transform_to_ros_transform(
                self.carla_actor.get_transform())
        return tf_msg

    def publish_transform(self, ros_transform_msg):
        """
        Helper function to send a ROS tf message of this child

        :return:
        """
        self.node.publish_tf_message(ros_transform_msg)
