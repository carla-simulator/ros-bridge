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
from carla_msgs.msg import CarlaCollisionEvent


class CollisionSensor(Sensor):

    """
    Actor implementation details for a collision sensor
    """

    def __init__(self, carla_actor, parent, communication, synchronous_mode):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(CollisionSensor, self).__init__(carla_actor=carla_actor,
                                              parent=parent,
                                              communication=communication,
                                              synchronous_mode=synchronous_mode,
                                              is_event_sensor=True,
                                              prefix="collision")

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, collision_event):
        """
        Function to wrap the collision event into a ros messsage

        :param collision_event: carla collision event object
        :type collision_event: carla.CollisionEvent
        """
        collision_msg = CarlaCollisionEvent()
        collision_msg.header = self.get_msg_header()
        collision_msg.other_actor_id = collision_event.other_actor.id
        collision_msg.normal_impulse.x = collision_event.normal_impulse.x
        collision_msg.normal_impulse.y = collision_event.normal_impulse.y
        collision_msg.normal_impulse.z = collision_event.normal_impulse.z

        self.publish_message(
            self.get_topic_prefix(), collision_msg)
