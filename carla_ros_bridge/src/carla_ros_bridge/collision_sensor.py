#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle collision events
"""

from carla_ros_bridge.sensor import Sensor


class CollisionSensor(Sensor):

    """
    Actor implementation details for a collision sensor
    """

    def __init__(self, carla_actor, parent, binding):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        super(CollisionSensor, self).__init__(carla_actor=carla_actor,
                                              parent=parent,
                                              binding=binding,
                                              topic_prefix="collision")

    def sensor_data_updated(self, collision_event):
        """
        Function to wrap the collision event into a message

        :param collision_event: carla collision event object
        :type collision_event: carla.CollisionEvent
        """
        self.get_binding().publish_collision(self.get_topic_prefix(), collision_event)
