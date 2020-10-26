#!/usr/bin/env python

#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle rss sensor
"""

from carla_ros_bridge.actor import Actor


class RssSensor(Actor):

    """
    Actor implementation details for a RSS sensor

    As the RSS sensor in CARLA requires additional
    utilization it's not handled as a sensor here.
    """

    def __init__(self, carla_actor, parent, node, _):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(RssSensor, self).__init__(carla_actor=carla_actor,
                                        parent=parent,
                                        node=node,
                                        prefix="rss")
