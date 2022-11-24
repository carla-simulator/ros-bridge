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

import carla_common.transforms as trans
from carla import WalkerControl

from carla_ros_bridge.traffic_participant import TrafficParticipant

from carla_msgs.msg import CarlaWalkerControl
from derived_object_msgs.msg import Object


class Walker(TrafficParticipant):

    """
    Actor implementation details for pedestrians
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla walker actor object
        :type carla_actor: carla.Walker
        """
        super(Walker, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     node=node,
                                     carla_actor=carla_actor)

        self.control_subscriber = self.node.new_subscription(
            CarlaWalkerControl,
            self.get_topic_prefix() + "/walker_control_cmd",
            self.control_command_updated,
            qos_profile=10)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        super(Walker, self).destroy()
        self.node.destroy_subscription(self.control_subscriber)

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

    def get_current_ros_pose(self):
        """
        Function to return the pose for walkers.

        :return: the pose of the walker
        :rtype: geometry_msgs.msg.Pose
        """
        # Moving position of walkers from the pivot point to the bottom of the bounding box.
        extent = self.carla_actor.bounding_box.extent
        pose_transform = self.carla_actor.get_transform()
        pose_transform.location -= pose_transform.get_up_vector() * extent.z
        return trans.carla_transform_to_ros_pose(pose_transform)

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return Object.CLASSIFICATION_PEDESTRIAN
