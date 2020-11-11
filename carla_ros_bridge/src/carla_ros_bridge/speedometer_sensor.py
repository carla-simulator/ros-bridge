#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a speedometer sensor
"""

import rospy
import numpy as np

from carla_ros_bridge.pseudo_actor import PseudoActor

from std_msgs.msg import Float32


class SpeedometerSensor(PseudoActor):

    """
    Pseudo speedometer sensor
    """

    def __init__(self, name, parent, node):
        """
        Constructor
        :param name: name identiying the sensor
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(SpeedometerSensor, self).__init__(parent=parent,
                                                node=node,
                                                prefix='speedometer/' + name)

        self.speedometer_publisher = rospy.Publisher(self.get_topic_prefix() +
                                                     "/speedometer",
                                                     Float32,
                                                     queue_size=10)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        velocity = self.parent.carla_actor.get_velocity()
        transform = self.parent.carla_actor.get_transform()

        vel_np = np.array([velocity.x, velocity.y, velocity.z])
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)
        orientation = np.array([
            np.cos(pitch) * np.cos(yaw),
            np.cos(pitch) * np.sin(yaw),
            np.sin(pitch)
        ])
        speed = np.dot(vel_np, orientation)

        self.speedometer_publisher.publish(Float32(speed))
