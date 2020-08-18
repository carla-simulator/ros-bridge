#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla pedestrians
"""

from derived_object_msgs.msg import Object  # pylint: disable=import-error
from carla import WalkerControl
from carla_msgs.msg import CarlaWalkerControl  # pylint: disable=import-error
from carla_ros_bridge.traffic_participant import TrafficParticipant

from ros_compatibility import destroy_subscription


class Walker(TrafficParticipant):
    """
    Actor implementation details for pedestrians
    """

    def __init__(self, carla_actor, parent, node):
        """
        Constructor

        :param carla_actor: carla walker actor object
        :type carla_actor: carla.Walker
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        if carla_actor.attributes.get('role_name'):
            prefix = carla_actor.attributes.get('role_name')
        else:
            prefix = "walker/w{:03}".format(carla_actor.id)

        super(Walker, self).__init__(carla_actor=carla_actor, parent=parent,
                                     node=node, prefix=prefix)

        self.control_subscriber = self.node.create_subscriber(
            CarlaWalkerControl,
            self.get_topic_prefix() + "/walker_control_cmd", self.control_command_updated)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        self.node.logdebug("Destroy Walker(id={})".format(self.get_id()))
        destroy_subscription(self.control_subscriber)
        self.control_subscriber = None
        super(Walker, self).destroy()

    def control_command_updated(self, ros_walker_control):
        """
        Receive a CarlaWalkerControl msg and send to CARLA
        This function gets called whenever a ROS message is received via
        '/carla/<role name>/walker_control_cmd' topic.
        The received ROS message is converted into carla.WalkerControl command and
        sent to CARLA.
        :param ros_walker_control: current walker control input received via ROS
        :type self.info.output: carla_ros_bridge.msg.CarlaWalkerControl
        :return:
        """
        walker_control = WalkerControl()
        walker_control.direction.x = ros_walker_control.direction.x
        walker_control.direction.y = -ros_walker_control.direction.y
        walker_control.direction.z = ros_walker_control.direction.z
        walker_control.speed = ros_walker_control.speed
        walker_control.jump = ros_walker_control.jump
        self.carla_actor.apply_control(walker_control)

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return Object.CLASSIFICATION_PEDESTRIAN
