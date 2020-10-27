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

import rospy

from carla_msgs.msg import CarlaTrafficLightStatusList,\
    CarlaTrafficLightInfoList
from carla_ros_bridge.traffic import TrafficLight
from carla_ros_bridge.pseudo_actor import PseudoActor


class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    def __init__(self, parent, node, actor_list):
        """
        Constructor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(TrafficLightsSensor, self).__init__(parent=parent,
                                                  node=node,
                                                  prefix="")
        self.actor_list = actor_list
        self.traffic_light_status = CarlaTrafficLightStatusList()
        self.traffic_light_actors = []

        self.traffic_lights_info_publisher = rospy.Publisher(
            self.get_topic_prefix() + "traffic_lights_info",
            CarlaTrafficLightInfoList,
            queue_size=10,
            latch=True)
        self.traffic_lights_status_publisher = rospy.Publisher(
            self.get_topic_prefix() + "traffic_lights",
            CarlaTrafficLightStatusList,
            queue_size=10,
            latch=True)

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
        traffic_light_status = CarlaTrafficLightStatusList()
        traffic_light_actors = []
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                traffic_light_actors.append(actor)
                traffic_light_status.traffic_lights.append(actor.get_status())

        if traffic_light_actors != self.traffic_light_actors:
            self.traffic_light_actors = traffic_light_actors
            traffic_light_info_list = CarlaTrafficLightInfoList()
            for traffic_light in traffic_light_actors:
                traffic_light_info_list.traffic_lights.append(traffic_light.get_info())
            self.traffic_lights_info_publisher.publish(traffic_light_info_list)

        if traffic_light_status != self.traffic_light_status:
            self.traffic_light_status = traffic_light_status
            self.traffic_lights_status_publisher.publish(traffic_light_status)
