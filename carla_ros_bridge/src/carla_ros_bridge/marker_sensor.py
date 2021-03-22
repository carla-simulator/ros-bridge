#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a marker sensor
"""

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic_participant import TrafficParticipant

from visualization_msgs.msg import MarkerArray


class MarkerSensor(PseudoActor):

    """
    Pseudo marker sensor
    """

    def __init__(self, uid, name, parent, node, actor_list):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(MarkerSensor, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)
        self.actor_list = actor_list

        self.marker_publisher = node.new_publisher(MarkerArray,
                                                   self.get_topic_prefix())

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.actor_list = None
        self.node.destroy_publisher(self.marker_publisher)
        super(MarkerSensor, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.markers"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update map sends:
        - tf global frame
        :return:
        """
        marker_array_msg = MarkerArray()
        for actor in self.actor_list.values():
            if isinstance(actor, TrafficParticipant):
                marker_array_msg.markers.append(actor.get_marker())
        self.marker_publisher.publish(marker_array_msg)
