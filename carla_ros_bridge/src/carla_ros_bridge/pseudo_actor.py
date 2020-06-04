#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Class to handle Pseudo Actors (that are not existing in Carla world)
"""

from std_msgs.msg import Header
import os
ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))
if ROS_VERSION == 1:
    import rospy
    from ros_compatibility import *
elif ROS_VERSION == 2:
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy
    from rclpy.qos import QoSProfile
    import sys
    print(os.getcwd())
    # TODO: fix setup.py to easily import CompatibleNode (as in ROS1)
    sys.path.append(os.getcwd() +
                    '/install/ros_compatibility/lib/python3.6/site-packages/src/ros_compatibility')
    from rclpy.node import Node
    from rclpy import executors
    from rclpy.time import Time
    from ament_index_python.packages import get_package_share_directory
    from ros_compatible_node import CompatibleNode
else:
    raise NotImplementedError("Make sure you have a valid ROS_VERSION env variable set.")


class PseudoActor(CompatibleNode):
    """
    Generic base class for Pseudo actors (that are not existing in Carla world)
    """

    def __init__(self, parent, communication, prefix=None):
        """
        Constructor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.PseudoActor
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        """

        # super(PseudoActor, self).__init__("pseudo_actor_node")

        self.parent = parent
        if self.parent:
            self.parent_id = parent.get_id()
        else:
            self.parent_id = None

        self.communication = communication

        # Concatenate the onw prefix to the the parent topic name if not empty.
        self.prefix = ""
        if parent:
            self.prefix = parent.get_prefix()
        if prefix:
            if self.prefix:
                self.prefix += "/"
            self.prefix += prefix

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.parent = None
        self.communication = None

    def publish_message(self, topic, msg, is_latched=False):
        """
        hand message over to communication layer
        """
        return self.communication.publish_message(topic, msg, is_latched)

    def get_msg_header(self, frame_id=None, timestamp=None):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        if frame_id:
            header.frame_id = frame_id
        else:
            header.frame_id = self.get_prefix()
        if timestamp:
            if ROS_VERSION == 1:
                header.stamp = rospy.Time.from_sec(timestamp)
            elif ROS_VERSION == 2:

                def ros_timestamp(sec=0):
                    from builtin_interfaces.msg import Time
                    time = Time()
                    time.sec = int(sec)
                    time.nanosec = int((sec - int(sec)) * 1000000000)
                    return time

                header.stamp = ros_timestamp(timestamp)
            else:
                raise NotImplementedError(
                    "Make sure you have a valid ROS_VERSION env variable set.")
        else:
            header.stamp = self.communication.get_current_ros_time()
        return header

    def get_parent_id(self):
        """
        Getter for the carla_id of the parent.
        :return: unique carla_id of the parent of this child
        :rtype: int64
        """
        return self.parent_id

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/carla/" + self.prefix

    def get_prefix(self):
        """
        get the fully qualified prefix of object
        :return: prefix
        :rtype: string
        """
        return self.prefix

    def update(self, frame, timestamp):
        """
        Function to update this object. Derived classes can add code.
        """
        pass
