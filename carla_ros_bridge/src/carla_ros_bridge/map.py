#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle the carla map
"""

import carla


class Map(object):

    """
    Child implementation details for the map
    """

    def __init__(self, carla_world, topic, binding):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this child
        :type topic_prefix: string
        """

        self.carla_map = carla_world.get_map()
        self.topic = topic
        self.binding = binding
        self.map_published = False

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        self.binding.logdebug("Destroying Map()")
        self.carla_map = None

    def update(self):
        """
        Function (override) to update this object.

        On update map sends:
        - tf global frame

        :return:
        """
        if not self.map_published:
            self.binding.publish_map(self.carla_map)
        self.binding.publish_transform(1, carla.Transform())
