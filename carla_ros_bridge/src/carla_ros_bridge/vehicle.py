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

from carla_ros_bridge.traffic_participant import TrafficParticipant


class Vehicle(TrafficParticipant):

    """
    Actor implementation details for vehicles
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
        if not prefix:
            prefix = "vehicle/{:03}".format(carla_actor.id)

        self.classification = Object.CLASSIFICATION_CAR
        if 'object_type' in carla_actor.attributes:
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

        super(Vehicle, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      node=node,
                                      prefix=prefix)

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

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
