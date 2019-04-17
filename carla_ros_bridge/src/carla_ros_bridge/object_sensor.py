#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a object sensor
"""

import carla

from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.pseudo_actor import PseudoActor


class ObjectSensor(PseudoActor):

    """
    Child implementation details for an object sensor
    """

    def __init__(self, parent, topic_prefix, binding, actor_list, filtered_id):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this child
        :type topic_prefix: string
        """

        super(ObjectSensor, self).__init__(parent=parent,
                                           binding=binding,
                                           topic_prefix=topic_prefix)
        self.actor_list = actor_list
        self.filtered_id = filtered_id

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        self.get_objects_callback = None
        super(ObjectSensor, self).destroy()

    def update(self):
        """
        Function (override) to update this object.

        On update map sends:
        - tf global frame

        :return:
        """
        object_info_list = []
        for actor_id in self.actor_list.keys():
            # currently only Vehicles are added to the object array
            if self.filtered_id is not actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle):
                    print("{} vs {}: {}".format(self.filtered_id, actor_id, actor))
                    object_info_list.append(actor.get_object_info())
        self.binding.publish_objects(self.topic_prefix, object_info_list)
