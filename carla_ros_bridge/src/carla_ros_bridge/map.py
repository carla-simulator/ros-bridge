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

from carla_ros_bridge.pseudo_actor import PseudoActor


class Map(PseudoActor):

    """
    Child implementation details for the map
    """

    def __init__(self, carla_world, topic_prefix, binding):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this child
        :type topic_prefix: string
        """

        super(Map, self).__init__(parent=None,
                                  binding=binding,
                                  topic_prefix=topic_prefix)
        self.carla_map = carla_world.get_map()
        self.map_published = False

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        self.carla_map = None
        super(Map, self).destroy()

    def update(self):
        """
        Function (override) to update this object.

        On update map sends:
        - tf global frame

        :return:
        """
        if not self.map_published:
            self.binding.publish_map(self.carla_map)
