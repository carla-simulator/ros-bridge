#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a actor list sensor
"""

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.pseudo_actor import PseudoActor

from carla_msgs.msg import CarlaActorList, CarlaActorInfo


class ActorListSensor(PseudoActor):

    """
    Pseudo actor list sensor
    """

    def __init__(self, uid, name, parent, node, actor_list):
        """
        Constructor
        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(ActorListSensor, self).__init__(uid=uid,
                                              name=name,
                                              parent=parent,
                                              node=node)
        self.actor_list = actor_list
        self.actor_list_publisher = node.new_publisher(CarlaActorList, self.get_topic_prefix(), qos_profile=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(ActorListSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.actor_list_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.actor_list"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        ros_actor_list = CarlaActorList()

        for actor_id in self.actor_list.keys():
            if not isinstance(self.actor_list[actor_id], Actor):
                continue

            actor = self.actor_list[actor_id].carla_actor
            ros_actor = CarlaActorInfo()
            ros_actor.id = actor.id
            ros_actor.type = actor.type_id
            try:
                ros_actor.rolename = str(actor.attributes.get('role_name'))
            except ValueError:
                pass

            if actor.parent:
                ros_actor.parent_id = actor.parent.id
            else:
                ros_actor.parent_id = 0

            ros_actor_list.actors.append(ros_actor)

        self.actor_list_publisher.publish(ros_actor_list)
