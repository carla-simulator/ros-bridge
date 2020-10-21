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

import rospy

from carla_msgs.msg import CarlaRadarMeasurement, CarlaRadarDetection

from carla_ros_bridge.sensor import Sensor


class Radar(Sensor):

    """
    Actor implementation details of Carla RADAR
    """

    def __init__(self, carla_actor, parent, node, synchronous_mode):
        """
        Constructor
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Radar, self).__init__(carla_actor=carla_actor,
                                    parent=parent,
                                    node=node,
                                    synchronous_mode=synchronous_mode,
                                    prefix="radar/" + carla_actor.attributes.get('role_name'))

        self.radar_publisher = rospy.Publisher(self.get_topic_prefix() +
                                               "/radar",
                                               CarlaRadarMeasurement,
                                               queue_size=10)
        self.listen()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_radar_measurement):
        """
        Function to transform the a received Radar measurement into a ROS message

        :param carla_radar_measurement: carla Radar measurement object
        :type carla_radar_measurement: carla.RadarMeasurement
        """
        radar_msg = CarlaRadarMeasurement()
        radar_msg.header = self.get_msg_header(timestamp=carla_radar_measurement.timestamp)
        for detection in carla_radar_measurement:
            radar_detection = CarlaRadarDetection()
            radar_detection.altitude = detection.altitude
            radar_detection.azimuth = detection.azimuth
            radar_detection.depth = detection.depth
            radar_detection.velocity = detection.velocity
            radar_msg.detections.append(radar_detection)
        self.radar_publisher.publish(radar_msg)
