#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
a sensor that reports the state of all traffic lights
"""

from carla_msgs.msg import CarlaTrafficLightStatusList
from carla_ros_bridge.traffic import TrafficLight
from carla_ros_bridge.pseudo_actor import PseudoActor


class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    def __init__(self, parent, communication, actor_list):
        """
        Constructor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(TrafficLightsSensor, self).__init__(parent=parent,
                                                  communication=communication,
                                                  prefix='traffic_lights')
        self.actor_list = actor_list

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.actor_list = None
        super(TrafficLightsSensor, self).destroy()

    def update(self, frame, timestamp):
        """
        Get the state of all known traffic lights
        """
        traffic_lights = CarlaTrafficLightStatusList()
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                traffic_lights.traffic_lights.append(actor.get_status())
        self.publish_message(self.get_topic_prefix(), traffic_lights)
