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

import rospy

from derived_object_msgs.msg import ObjectArray
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker
from carla_ros_bridge.pseudo_actor import PseudoActor


class ObjectSensor(PseudoActor):

    """
    Pseudo object sensor
    """

    def __init__(self, parent, node, actor_list, filtered_id):
        """
        Constructor
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        :param filtered_id: id to filter from actor_list
        :type filtered_id: int
        """

        super(ObjectSensor, self).__init__(parent=parent,
                                           node=node,
                                           prefix='objects')
        self.actor_list = actor_list
        self.filtered_id = filtered_id
        self.object_publisher = rospy.Publisher(self.get_topic_prefix(),
                                                ObjectArray,
                                                queue_size=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.actor_list = None
        super(ObjectSensor, self).destroy()

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update map sends:
        - tf global frame
        :return:
        """
        ros_objects = ObjectArray(header=self.get_msg_header("map"))
        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.filtered_id != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle):
                    ros_objects.objects.append(actor.get_object_info())
                elif isinstance(actor, Walker):
                    ros_objects.objects.append(actor.get_object_info())
        self.object_publisher.publish(ros_objects)
