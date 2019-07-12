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
import carla_ros_bridge.transforms as trans


class Actor(PseudoActor):

    """
    Generic base class for all carla actors
    """

    def __init__(self, carla_actor, parent, communication, prefix=None):
        """
        Constructor
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        super(Actor, self).__init__(parent=parent,
                                    prefix=prefix,
                                    communication=communication)
        self.carla_actor = carla_actor

        if carla_actor.id > np.iinfo(np.uint32).max:
            raise ValueError("Actor ID exceeds maximum supported value '{}'".format(carla_actor.id))

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

    def get_current_ros_twist(self):
        """
        Function to provide the current ROS twist

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity(),
            self.carla_actor.get_transform().rotation)

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

    def get_ros_transform(self, transform=None):
        """
        Function to provide the current ROS transform

        :return: the ROS transfrom
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = TransformStamped()
        tf_msg.header = self.get_msg_header("map")
        tf_msg.child_frame_id = self.get_prefix()
        if transform:
            tf_msg.transform = trans.carla_transform_to_ros_transform(transform)
        else:
            tf_msg.transform = trans.carla_transform_to_ros_transform(
                self.carla_actor.get_transform())
        return tf_msg

    def publish_transform(self, ros_transform_msg):
        """
        Helper function to send a ROS tf message of this child

        :return:
        """
        self.publish_message('tf', ros_transform_msg)
