#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle lane invasion events
"""

from carla_ros_bridge.sensor import Sensor
from carla_msgs.msg import CarlaLaneInvasionEvent


class LaneInvasionSensor(Sensor):

    """
    Actor implementation details for a lane invasion sensor
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        super(LaneInvasionSensor, self).__init__(carla_actor=carla_actor,
                                                 parent=parent,
                                                 topic_prefix="lane_invasion",
                                                 append_role_name_topic_postfix=False)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, lane_invasion_event):
        """
        Function to wrap the lane invasion event into a ros messsage

        :param lane_invasion_event: carla lane invasion event object
        :type lane_invasion_event: carla.LaneInvasionEvent
        """
        lane_invasion_msg = CarlaLaneInvasionEvent()
        lane_invasion_msg.header = self.get_msg_header(use_parent_frame=False)
        for marking in lane_invasion_event.crossed_lane_markings:
            lane_invasion_msg.crossed_lane_markings.append(marking.type)
        self.publish_ros_message(
            self.topic_name(), lane_invasion_msg)
