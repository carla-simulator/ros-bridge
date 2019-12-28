#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla Radar
"""

from delphi_srr_msgs.msg import SrrTrack, SrrTrackArray

import math

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

    def sensor_data_updated(self, carla_radar_event):
        track_array = SrrTrackArray()
        track_array.header = self.get_msg_header(timestamp=carla_radar_event.timestamp)
        for detection in carla_radar_event:
            track = SrrTrack()
            track.header = track_array.header
            track.can_tx_detect_amplitude = 25.0
            track.can_tx_detect_range_rate = detection.velocity
            track.can_tx_detect_angle = math.degrees(detection.azimuth)
            track.can_tx_detect_range = detection.depth
            track_array.tracks.append(track)
        self.publish_message(self.get_topic_prefix() + "/radar", track_array)

