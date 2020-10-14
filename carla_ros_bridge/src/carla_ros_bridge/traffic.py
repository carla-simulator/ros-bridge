#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic objects
"""

from carla_ros_bridge.actor import Actor
import carla_common.transforms as trans
from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightInfo
from carla import TrafficLightState


class Traffic(Actor):

    """
    Actor implementation details for traffic objects
    """

    def __init__(self, carla_actor, parent, node):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """
        super(Traffic, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      node=node,
                                      prefix='traffic')


class TrafficLight(Actor):

    """
    Traffic implementation details for traffic lights
    """

    def __init__(self, carla_actor, parent, node):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.TrafficLight
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """
        super(TrafficLight, self).__init__(carla_actor=carla_actor,
                                           parent=parent,
                                           node=node,
                                           prefix='traffic.traffic_light')

    def get_status(self):
        """
        Get the current state of the traffic light
        """
        status = CarlaTrafficLightStatus()
        status.id = self.get_id()
        carla_state = self.carla_actor.get_state()
        if carla_state == TrafficLightState.Red:
            status.state = CarlaTrafficLightStatus.RED
        elif carla_state == TrafficLightState.Yellow:
            status.state = CarlaTrafficLightStatus.YELLOW
        elif carla_state == TrafficLightState.Green:
            status.state = CarlaTrafficLightStatus.GREEN
        elif carla_state == TrafficLightState.Off:
            status.state = CarlaTrafficLightStatus.OFF
        else:
            status.state = CarlaTrafficLightStatus.UNKNOWN
        return status

    def get_info(self):
        """
        Get the info of the traffic light
        """
        info = CarlaTrafficLightInfo()
        info.id = self.get_id()
        info.transform = self.get_current_ros_pose()
        info.trigger_volume.center = trans.carla_location_to_ros_point(
            self.carla_actor.trigger_volume.location)
        info.trigger_volume.size.x = self.carla_actor.trigger_volume.extent.x * 2.0
        info.trigger_volume.size.y = self.carla_actor.trigger_volume.extent.y * 2.0
        info.trigger_volume.size.z = self.carla_actor.trigger_volume.extent.z * 2.0
        return info
