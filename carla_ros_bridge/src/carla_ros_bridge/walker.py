#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla pedestrians
"""

from std_msgs.msg import ColorRGBA
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

from carla_ros_bridge.actor import Actor
import carla_ros_bridge.transforms as transforms


class Walker(Actor):

    """
    Actor implementation details for pedestrians
    """

    def __init__(self, carla_actor, parent, communication, prefix=None):
        """
        Constructor

        :param carla_actor: carla walker actor object
        :type carla_actor: carla.Walker
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        if not prefix:
            prefix = "walker/{:03}".format(carla_actor.id)

        super(Walker, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      communication=communication,
                                      prefix=prefix)

        self.classification = Object.CLASSIFICATION_PEDESTRIAN
        self.classification_age = 0

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update walkers send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        self.publish_transform(self.get_ros_transform())
        self.publish_marker()
        super(Walker, self).update(frame, timestamp)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a walkers marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255
        color.g = 0
        color.b = 0
        return color

    def get_marker(self):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the actor

        :param use_parent_frame: per default (True) the header.frame_id
            is set to the frame of the actor's parent.
            If this is set to False, the actor's own frame is used as basis.
        :type use_parent_frame:  boolean
        :return:
        visualization_msgs.msg.Marker
        """
        marker = Marker(
            header=self.get_msg_header())
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

        marker.pose = transforms.carla_location_to_pose(
            self.carla_actor.bounding_box.location)
        marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
        marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
        marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
        self.publish_message('/carla/walker_marker', marker)

    def get_object_info(self):
        """
        Function to send object messages of this walker

        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'

        :return:
        """
        walker_object = Object(header=self.get_msg_header("map"))
        # ID
        walker_object.id = self.get_id()
        # Pose
        walker_object.pose = self.get_current_ros_pose()
        # Twist
        walker_object.twist = self.get_current_ros_twist()
        # Acceleration
        walker_object.accel = self.get_current_ros_accel()
        # Shape
        walker_object.shape.type = SolidPrimitive.BOX
        walker_object.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])

        # Classification if available in attributes
        if self.classification != Object.CLASSIFICATION_UNKNOWN:
            walker_object.object_classified = True
            walker_object.classification = self.classification
            walker_object.classification_certainty = 1.0
            self.classification_age += 1
            walker_object.classification_age = self.classification_age

        return walker_object
