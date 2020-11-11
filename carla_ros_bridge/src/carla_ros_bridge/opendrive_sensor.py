#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a opendrive sensor
"""

import rospy
import numpy as np

from carla_ros_bridge.pseudo_actor import PseudoActor

from std_msgs.msg import String


class OpenDriveSensor(PseudoActor):

    """
    Pseudo opendrive sensor
    """

    def __init__(self, name, parent, node, carla_map):
        """
        Constructor
        :param name: name identiying the sensor
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_map: carla map object
        :type carla_map: carla.Map
        """
        super(OpenDriveSensor, self).__init__(parent=parent,
                                              node=node,
                                              prefix='opendrive/' + name)

        self.carla_map = carla_map
        self._map_published = False
        self.map_publisher = rospy.Publisher(self.get_topic_prefix() + "/map",
                                             String,
                                             queue_size=10,
                                             latch=True)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        if not self._map_published:
            self.map_publisher.publish(String(self.carla_map.to_opendrive()))
            self._map_published = True
