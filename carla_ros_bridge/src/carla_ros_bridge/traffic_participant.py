#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic participants
"""

import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor

from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class TrafficParticipant(Actor):

    """
    actor implementation details for traffic participant
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
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.classification_age = 0
        super(TrafficParticipant, self).__init__(uid=uid,
                                                 name=name,
                                                 parent=parent,
                                                 node=node,
                                                 carla_actor=carla_actor)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update vehicles send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        super(TrafficParticipant, self).update(frame, timestamp)

    def get_object_info(self):
        """
        Function to send object messages of this traffic participant.

        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'

        :return:
        """
        obj = Object(header=self.get_msg_header("map"))
        # ID
        obj.id = self.get_id()
        # Pose
        obj.pose = self.get_current_ros_pose()
        # Twist
        obj.twist = self.get_current_ros_twist()
        # Acceleration
        obj.accel = self.get_current_ros_accel()
        # Shape
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])

        # Classification if available in attributes
        if self.get_classification() != Object.CLASSIFICATION_UNKNOWN:
            obj.object_classified = True
            obj.classification = self.get_classification()
            obj.classification_certainty = 255
            obj.classification_age = self.classification_age

        return obj

    def get_classification(self):  # pylint: disable=no-self-use
        """
        Function to get object classification (overridden in subclasses)
        """
        return Object.CLASSIFICATION_UNKNOWN

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: default color used by traffic participants
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0.
        color.g = 0.
        color.b = 255.
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for traffic participants.

        :return: the pose of the traffic participant.
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_ros_pose(self.carla_actor.get_transform())

    def get_marker(self, timestamp=None):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the actor

        :return:
        visualization_msgs.msg.Marker
        """
        marker = Marker(header=self.get_msg_header(frame_id="map", timestamp=timestamp))
        marker.color = self.get_marker_color()
        marker.color.a = 0.3
        marker.id = self.get_id()
        marker.type = Marker.CUBE

        marker.pose = self.get_marker_pose()
        marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
        marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
        marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
        return marker
