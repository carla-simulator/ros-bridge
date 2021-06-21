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

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(CollisionSensor, self).__init__(uid=uid,
                                              name=name,
                                              parent=parent,
                                              relative_spawn_pose=relative_spawn_pose,
                                              node=node,
                                              carla_actor=carla_actor,
                                              synchronous_mode=synchronous_mode,
                                              is_event_sensor=True)

        self.collision_publisher = node.new_publisher(CarlaCollisionEvent,
                                                      self.get_topic_prefix(),
                                                      qos_profile=10)
        self.listen()

    def destroy(self):
        super(CollisionSensor, self).destroy()
        self.node.destroy_publisher(self.collision_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, collision_event):
        """
        Function to wrap the collision event into a ros messsage

        :param collision_event: carla collision event object
        :type collision_event: carla.CollisionEvent
        """
        collision_msg = CarlaCollisionEvent()
        collision_msg.header = self.get_msg_header(timestamp=collision_event.timestamp)
        collision_msg.other_actor_id = collision_event.other_actor.id
        collision_msg.normal_impulse.x = collision_event.normal_impulse.x
        collision_msg.normal_impulse.y = collision_event.normal_impulse.y
        collision_msg.normal_impulse.z = collision_event.normal_impulse.z

        self.collision_publisher.publish(collision_msg)
