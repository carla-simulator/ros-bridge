#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic objects
"""

from carla_ros_bridge.actor import Actor


class Traffic(Actor):

    """
    Actor implementation details for traffic objects
    """

    def __init__(self, carla_actor, parent, communication):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        """
        super(Traffic, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      communication=communication,
                                      prefix='traffic')


class TrafficLight(Actor):

    """
    Traffic implementation details for traffic lights
    """

    def __init__(self, carla_actor, parent, communication):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.TrafficLight
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        """
        super(TrafficLight, self).__init__(carla_actor=carla_actor,
                                           parent=parent,
                                           communication=communication,
                                           prefix='traffic.traffic_light')
