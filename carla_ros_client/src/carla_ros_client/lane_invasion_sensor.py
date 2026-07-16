#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle lane invasion events
"""

from carla_ros_client.sensor import Sensor

from carla_msgs.msg import CarlaLaneInvasionEvent


class LaneInvasionSensor(Sensor):

    """
    Actor implementation details for a lane invasion sensor
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_client.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(LaneInvasionSensor, self).__init__(uid=uid,
                                                 name=name,
                                                 parent=parent,
                                                 node=node,
                                                 carla_actor=carla_actor)

        self.lane_invasion_publisher = node.new_publisher(CarlaLaneInvasionEvent,
                                                          self.get_topic_prefix(),
                                                          qos_profile=10)
        self.listen()

    def destroy(self):
        super(LaneInvasionSensor, self).destroy()
        self.node.destroy_publisher(self.lane_invasion_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, lane_invasion_event):
        """
        Function to wrap the lane invasion event into a ros messsage

        :param lane_invasion_event: carla lane invasion event object
        :type lane_invasion_event: carla.LaneInvasionEvent
        """
        lane_invasion_msg = CarlaLaneInvasionEvent()
        lane_invasion_msg.header = self.get_msg_header(timestamp=lane_invasion_event.timestamp)
        for marking in lane_invasion_event.crossed_lane_markings:
            lane_invasion_msg.crossed_lane_markings.append(marking.type)
        self.lane_invasion_publisher.publish(lane_invasion_msg)
