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
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from carla_ros_bridge.pseudo_actor import PseudoActor
import carla_common.transforms as trans


class Actor(PseudoActor):

    """
    Generic base class for all carla actors
    """

    def __init__(self, carla_actor, parent, node, prefix=None):
        """
        Constructor
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        super(Actor, self).__init__(parent=parent, prefix=prefix, node=node)
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

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a walkers marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 0
        color.b = 255
        return color

    def get_marker(self):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the actor

        :return:
        visualization_msgs.msg.Marker
        """
        marker = Marker(
            header=self.get_msg_header(frame_id=str(self.get_id())))
        marker.color = self.get_marker_color()
        marker.color.a = 0.3
        marker.id = self.get_id()
        marker.text = "id = {}".format(marker.id)
        return marker

    def publish_marker(self):
        """
        Function to send marker messages of this walker.

        :return:
        """
        marker = self.get_marker()
        marker.type = Marker.CUBE

        marker.pose = trans.carla_location_to_pose(
            self.carla_actor.bounding_box.location)
        marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
        marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
        marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
        self.node.marker_publisher.publish(marker)
