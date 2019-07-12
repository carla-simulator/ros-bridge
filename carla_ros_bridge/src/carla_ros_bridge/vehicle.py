#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""

from std_msgs.msg import ColorRGBA
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive

from carla_ros_bridge.actor import Actor


class Vehicle(Actor):

    """
    Actor implementation details for vehicles
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
        if not prefix:
            prefix = "vehicle/{:03}".format(carla_actor.id)

        super(Vehicle, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      communication=communication,
                                      prefix=prefix)

        self.classification = Object.CLASSIFICATION_UNKNOWN
        if carla_actor.attributes.has_key('object_type'):
            if carla_actor.attributes['object_type'] == 'car':
                self.classification = Object.CLASSIFICATION_CAR
            elif carla_actor.attributes['object_type'] == 'bike':
                self.classification = Object.CLASSIFICATION_BIKE
            elif carla_actor.attributes['object_type'] == 'motorcycle':
                self.classification = Object.CLASSIFICATION_MOTORCYCLE
            elif carla_actor.attributes['object_type'] == 'truck':
                self.classification = Object.CLASSIFICATION_TRUCK
            elif carla_actor.attributes['object_type'] == 'other':
                self.classification = Object.CLASSIFICATION_OTHER_VEHICLE
        self.classification_age = 0

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
        self.publish_transform(self.get_ros_transform())
        self.publish_marker()
        super(Vehicle, self).update(frame, timestamp)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255
        color.g = 0
        color.b = 0
        return color

    def get_object_info(self):
        """
        Function to send object messages of this vehicle.

        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'

        :return:
        """
        vehicle_object = Object(header=self.get_msg_header("map"))
        # ID
        vehicle_object.id = self.get_id()
        # Pose
        vehicle_object.pose = self.get_current_ros_pose()
        # Twist
        vehicle_object.twist = self.get_current_ros_twist()
        # Acceleration
        vehicle_object.accel = self.get_current_ros_accel()
        # Shape
        vehicle_object.shape.type = SolidPrimitive.BOX
        vehicle_object.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])

        # Classification if available in attributes
        if self.classification != Object.CLASSIFICATION_UNKNOWN:
            vehicle_object.object_classified = True
            vehicle_object.classification = self.classification
            vehicle_object.classification_certainty = 1.0
            self.classification_age += 1
            vehicle_object.classification_age = self.classification_age

        return vehicle_object
