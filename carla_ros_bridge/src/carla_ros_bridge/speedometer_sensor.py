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

import numpy as np

from carla_ros_bridge.pseudo_actor import PseudoActor

from std_msgs.msg import Float32


class SpeedometerSensor(PseudoActor):

    """
    Pseudo speedometer sensor
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(SpeedometerSensor, self).__init__(uid=uid,
                                                name=name,
                                                parent=parent,
                                                node=node)

        self.speedometer_publisher = node.new_publisher(Float32,
                                                        self.get_topic_prefix(),
                                                        qos_profile=10)

    def destroy(self):
        super(SpeedometerSensor, self).destroy()
        self.node.destroy_publisher(self.speedometer_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.speedometer"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        try:
            velocity = self.parent.carla_actor.get_velocity()
            transform = self.parent.carla_actor.get_transform()
        except AttributeError:
            # parent actor disappeared, do not send tf
            self.node.logwarn(
                "SpeedometerSensor could not publish. Parent actor {} not found".format(self.parent.uid))
            return

        vel_np = np.array([velocity.x, velocity.y, velocity.z])
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)
        orientation = np.array([
            np.cos(pitch) * np.cos(yaw),
            np.cos(pitch) * np.sin(yaw),
            np.sin(pitch)
        ])
        speed = np.dot(vel_np, orientation)

        self.speedometer_publisher.publish(Float32(data=speed))
