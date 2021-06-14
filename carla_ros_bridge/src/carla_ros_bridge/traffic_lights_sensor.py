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

from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic import TrafficLight

from carla_msgs.msg import (
    CarlaTrafficLightStatusList,
    CarlaTrafficLightInfoList
)


class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    def __init__(self, uid, name, parent, node, actor_list):
        """
        Constructor
        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(TrafficLightsSensor, self).__init__(uid=uid,
                                                  name=name,
                                                  parent=parent,
                                                  node=node)

        self.actor_list = actor_list
        self.traffic_light_status = CarlaTrafficLightStatusList()
        self.traffic_light_actors = []

        self.traffic_lights_info_publisher = node.new_publisher(
            CarlaTrafficLightInfoList,
            self.get_topic_prefix() + "/info",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.traffic_lights_status_publisher = node.new_publisher(
            CarlaTrafficLightStatusList,
            self.get_topic_prefix() + "/status",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(TrafficLightsSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.traffic_lights_info_publisher)
        self.node.destroy_publisher(self.traffic_lights_status_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.traffic_lights"

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