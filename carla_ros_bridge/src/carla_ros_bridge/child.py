#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Classes to handle child objects
"""

from abc import abstractmethod

from carla_ros_bridge.parent import Parent


class Child(Parent):

    """
    Generic base class for all child entities
    """

    def __init__(self, carla_id, carla_world, parent, topic_prefix=''):
        """
        Constructor

        :param carla_id: unique carla_id of this child object
            carla_id > 0: carla actor ids (see also carla.Actor)
            carla_id == 0: resevered for the (root) bridge object; not allowed in here
            carla_id == -1: used by the map object
        :type carla_id: int64
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this child
        :type topic_prefix: string
        """
        if carla_id == 0:
            raise ValueError("A child can never have an carla_id of zero. "
                             "Reserved for the parent root (the bridge object)")
        self.topic_prefix = topic_prefix.replace(".", "/").replace("-", "_")
        # each Child defines its own frame
        super(Child, self).__init__(
            carla_id=carla_id, carla_world=carla_world, frame_id=self.topic_prefix)
        self.parent = parent
        self.get_binding().logdebug("Created {}-Child(id={}, parent_id={}, topic_name={})".format(
            self.__class__.__name__, self.get_id(), self.get_parent_id(), self.topic_name()))

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove the reference to the carla_ros_bridge.Parent object.
        Finally forward call to super class.

        :return:
        """
        self.get_binding().logdebug(
            "Destroying {}-Child(id={})".format(self.__class__.__name__, self.get_id()))
        self.parent = None
        super(Child, self).destroy()

    def get_param(self, key, default=None):
        """
        Function (override) to query global parameters passed from the outside.

        Just forwards the request to the parent.

        :param key: the key of the parameter
        :type key: string
        :param default: the default value of the parameter to return if key is not found
        :type default: string
        :return: the parameter string
        :rtype: string
        """
        return self.parent.get_param(key, default)

    def topic_name(self):
        """
        Function (override) to get the topic name of the current entity.

        Concatenate the child's onw topic prefix to the the parent topic name if not empty.

        :return: the final topic name of this
        :rtype: string
        """
        if len(self.topic_prefix) > 0:
            return self.parent.topic_name() + "/" + self.topic_prefix
        else:
            return self.parent.topic_name()

    def get_parent_id(self):
        """
        Getter for the carla_id of the parent.

        :return: unique carla_id of the parent of this child
        :rtype: int64
        """
        return self.parent.get_id()

    def get_actor_list(self):
        """
        Get list of actors from parent

        :return: the list of actors
        :rtype: list
        """
        return self.parent.get_actor_list()

    def get_filtered_objectarray(self, filtered_id):
        """
        get objectarray of available actors, except the one with the filtered id

        :return: objectarray of actors
        :rtype: derived_object_msgs.ObjectArray
        """
        return self.parent.get_filtered_objectarray(filtered_id)

    def get_binding(self):
        return self.parent.get_binding()
