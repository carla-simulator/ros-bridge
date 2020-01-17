#!/usr/bin/env python

#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla Radar
"""
import math

from ainstein_radar_msgs.msg import RadarTarget, RadarTargetArray

from carla_ros_bridge.sensor import Sensor


class Radar(Sensor):

    """
    Actor implementation details of Carla RADAR
    """

    def __init__(self, carla_actor, parent, communication, synchronous_mode):
        """
        Constructor
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Radar, self).__init__(carla_actor=carla_actor,
                                    parent=parent,
                                    communication=communication,
                                    synchronous_mode=synchronous_mode,
                                    prefix="radar/" + carla_actor.attributes.get('role_name'))

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_radar_measurement):
        """
        Function to transform the a received Radar measurement into a ROS message

        :param carla_radar_measurement: carla Radar measurement object
        :type carla_radar_measurement: carla.RadarMeasurement
        """
        radar_target_array = RadarTargetArray()
        radar_target_array.header = self.get_msg_header(timestamp=carla_radar_measurement.timestamp)
        for detection in carla_radar_measurement:
            radar_target = RadarTarget()
            radar_target.elevation = math.degrees(detection.altitude)
            radar_target.speed = detection.velocity
            radar_target.azimuth = math.degrees(detection.azimuth)
            radar_target.range = detection.depth
            radar_target_array.targets.append(radar_target)
        self.publish_message(self.get_topic_prefix() + "/radar", radar_target_array)
