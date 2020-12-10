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

import rospy

from carla_msgs.msg import CarlaWorldInfo


class WorldInfo(object):

    """
    Publish the map
    """

    def __init__(self, carla_world):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        """

        self.carla_map = carla_world.get_map()

        self.map_published = False

        self.world_info_publisher = rospy.Publisher("/carla/world_info",
                                                    CarlaWorldInfo,
                                                    queue_size=10,
                                                    latch=True)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroying WorldInfo()")
        self.carla_map = None

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
