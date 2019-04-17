#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Classes to handle Actor objects
"""

from carla_ros_bridge.actor_id_registry import ActorIdRegistry


class Actor(object):

    """
    Generic base class for all carla actors
    """

    global_id_registry = ActorIdRegistry()

    def __init__(self, carla_actor, parent, binding, topic_prefix=''):
        """
        Constructor

        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        self.parent = parent
        self.carla_actor = carla_actor
        self.binding = binding
#         self.frame_id = frame_id

        self.topic_prefix = ""
        if parent:
            self.topic_prefix = parent.get_topic_prefix()
        else:
            self.topic_prefix = "/carla"
        if len(topic_prefix):
            self.topic_prefix += "/" + topic_prefix

    def get_binding(self):
        """
        get the binding
        """
        return self.binding

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove the reference to the carla.Actor object.
        Finally forward call to super class.

        :return:
        """
        self.carla_actor = None
#

    def get_global_id(self):
        """
        Return a unique global id for the actor used for markers, object ids, etc.

        ros marker id should be int32, carla/unrealengine seems to use int64
        A lookup table is used to remap actor_id to small number between 0 and max_int32

        :return: mapped id of this actor (unique increasing counter value)
        :rtype: uint32
        """
        return Actor.global_id_registry.get_id(self.carla_actor.id)

    def get_id(self):
        """
        Getter for the carla_id of this.

        :return: unique carla_id of this parent object
        :rtype: int64
        """
        return self.carla_actor.id

    def get_parent_id(self):
        """
        Getter for the carla_id of the parent.

        :return: unique carla_id of the parent of this child
        :rtype: int64
        """
        if self.parent:
            return self.parent.get_id()
        else:
            return None

    def get_topic_prefix(self):
        """
        Function (override) to get the topic name of the current entity.

        Concatenate the child's onw topic prefix to the the parent topic name if not empty.

        :return: the final topic name of this
        :rtype: string
        """
        return self.topic_prefix

    def update(self):
        """
        Function to update this object. Derived classes can add code.
        """
        pass
