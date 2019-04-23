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

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        super(CollisionSensor, self).__init__(carla_actor=carla_actor,
                                              parent=parent,
                                              topic_prefix="collision",
                                              append_role_name_topic_postfix=False)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, collision_event):
        """
        Function to wrap the collision event into a ros messsage

        :param collision_event: carla collision event object
        :type collision_event: carla.CollisionEvent
        """
        collision_msg = CarlaCollisionEvent()
        collision_msg.header = self.get_msg_header(use_parent_frame=False)
        collision_msg.other_actor_id = collision_event.other_actor.id
        collision_msg.normal_impulse.x = collision_event.normal_impulse.x
        collision_msg.normal_impulse.y = collision_event.normal_impulse.y
        collision_msg.normal_impulse.z = collision_event.normal_impulse.z

        self.publish_ros_message(
            self.topic_name(), collision_msg)
