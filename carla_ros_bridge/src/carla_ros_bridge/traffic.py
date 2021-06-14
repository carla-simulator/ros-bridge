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

from carla import TrafficLightState

import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightInfo


class Traffic(Actor):

    """
    Actor implementation details for traffic objects
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(Traffic, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=node,
                                      carla_actor=carla_actor)


class TrafficLight(Actor):

    """
    Traffic implementation details for traffic lights
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.TrafficLight
        """
        super(TrafficLight, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node,
                                           carla_actor=carla_actor)

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
        info.trigger_volume.center = trans.carla_location_to_ros_vector3(
            self.carla_actor.trigger_volume.location)
        info.trigger_volume.size.x = self.carla_actor.trigger_volume.extent.x * 2.0
        info.trigger_volume.size.y = self.carla_actor.trigger_volume.extent.y * 2.0
        info.trigger_volume.size.z = self.carla_actor.trigger_volume.extent.z * 2.0
        return info
