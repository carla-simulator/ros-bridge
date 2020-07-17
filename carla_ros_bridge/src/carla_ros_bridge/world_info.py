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

from carla_msgs.msg import CarlaWorldInfo  # pylint: disable=import-error
from carla_ros_bridge.pseudo_actor import PseudoActor

from ros_compatibility import QoSProfile, latch_on


class WorldInfo(PseudoActor):
    """
    Publish the map
    """

    def __init__(self, carla_world, node):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param node: node-handle
        :type node: CompatibleNode
        """

        super(WorldInfo, self).__init__(parent=None, node=node,
                                        prefix="world_info")

        self.carla_map = carla_world.get_map()

        self.world_info_publisher = node.new_publisher(
            CarlaWorldInfo, self.get_topic_prefix(), qos_profile=QoSProfile(depth=10, durability=latch_on))

        self.map_published = False

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        self.logdebug("Destroying WorldInfo()")
        self.carla_map = None
        super(WorldInfo, self).destroy()

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        :return:
        """
        if not self.map_published:
            open_drive_msg = CarlaWorldInfo()
            open_drive_msg.map_name = self.carla_map.name
            open_drive_msg.opendrive = self.carla_map.to_opendrive()
            self.world_info_publisher.publish(open_drive_msg)
            self.map_published = True
