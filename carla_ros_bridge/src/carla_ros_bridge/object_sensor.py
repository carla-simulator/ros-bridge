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

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

from derived_object_msgs.msg import ObjectArray


class ObjectSensor(PseudoActor):

    """
    Pseudo object sensor
    """

    def __init__(self, uid, name, parent, node, actor_list):
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
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(ObjectSensor, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)
        self.actor_list = actor_list
        self.object_publisher = node.new_publisher(ObjectArray,
                                                   self.get_topic_prefix(),
                                                   qos_profile=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(ObjectSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.objects"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update map sends:
        - tf global frame
        :return:
        """
        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="map", timestamp=timestamp)
        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle):
                    ros_objects.objects.append(actor.get_object_info())
                elif isinstance(actor, Walker):
                    ros_objects.objects.append(actor.get_object_info())
        self.object_publisher.publish(ros_objects)
