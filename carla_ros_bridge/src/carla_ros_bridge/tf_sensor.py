#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a tf sensor
"""

import os
from carla_ros_bridge.pseudo_actor import PseudoActor
from ros_compatibility import ros_timestamp
from geometry_msgs.msg import TransformStamped, Transform

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    import tf
elif ROS_VERSION == 2:
    import tf2_ros



class TFSensor(PseudoActor):

    """
    Pseudo tf sensor
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

        super(TFSensor, self).__init__(uid=uid,
                                       name=name,
                                       parent=parent,
                                       node=node)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.tf"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        self.parent.get_prefix()
        self.tf_broadcaster.sendTransform(TransformStamped(
            header=self.get_msg_header("map"),
            child_frame_id=self.parent.get_prefix(),
            transform=self.parent.get_current_ros_transform()))
