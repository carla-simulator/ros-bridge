#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a opendrive sensor
"""

from carla_ros_bridge.pseudo_actor import PseudoActor

from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String


class OpenDriveSensor(PseudoActor):

    """
    Pseudo opendrive sensor
    """

    def __init__(self, uid, name, parent, node, carla_map):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_map: carla map object
        :type carla_map: carla.Map
        """
        super(OpenDriveSensor, self).__init__(uid=uid,
                                              name=name,
                                              parent=parent,
                                              node=node)
        self.carla_map = carla_map
        self._map_published = False
        self.map_publisher = node.new_publisher(
            String,
            self.get_topic_prefix(),
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def destroy(self):
        super(OpenDriveSensor, self).destroy()
        self.node.destroy_publisher(self.map_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.opendrive_map"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        if not self._map_published:
            self.map_publisher.publish(String(data=self.carla_map.to_opendrive()))
            self._map_published = True
