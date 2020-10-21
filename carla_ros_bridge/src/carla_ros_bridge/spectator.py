#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla spectator
"""

from carla_ros_bridge.actor import Actor


class Spectator(Actor):

    """
    Actor implementation details for spectators
    """

    def __init__(self, carla_actor, parent, node):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """
        super(Spectator, self).__init__(carla_actor=carla_actor,
                                        parent=parent,
                                        prefix='spectator',
                                        node=node)
