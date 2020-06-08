#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Handle communication of ROS topics
"""
import os
ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    from ros_compatibility import *
    latch = True
elif ROS_VERSION == 2:
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from rclpy.callback_groups import ReentrantCallbackGroup
    import sys
    print(os.getcwd())
    # TODO: fix setup.py to easily import CompatibleNode (as in ROS1)
    sys.path.append(os.getcwd() +
                    '/install/ros_compatibility/lib/python3.6/site-packages/src/ros_compatibility')
    from ament_index_python.packages import get_package_share_directory
    from ros_compatible_node import CompatibleNode, ros_timestamp
    latch = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
else:
    raise NotImplementedError("Make sure you have a valid ROS_VERSION env variable set.")

from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


class Communication(CompatibleNode):
    """
    Handle communication of ROS topics
    """

    def __init__(self):
        """
        Constructor
        """
        super(Communication, self).__init__("communication", rospy_init=False)
        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.pub = {}
        self.subscribers = {}
        self.ros_timestamp = ros_timestamp()

        if ROS_VERSION == 1:
            self.callback_group = None
        elif ROS_VERSION == 2:
            self.callback_group = ReentrantCallbackGroup()

        # needed?
        self.pub['clock'] = self.new_publisher(Clock, 'clock')
        self.pub['tf'] = self.new_publisher(TFMessage, 'tf', qos_profile=QoSProfile(depth=100))

    def send_msgs(self):
        """
        Function to actually send the collected ROS messages out via the publisher

        :return:
        """

        # prepare tf message
        tf_msg = None

        if ROS_VERSION == 1:
            tf_msg = TFMessage(self.tf_to_publish)
        elif ROS_VERSION == 2:
            tf_msg = TFMessage()
            tf_msg.transforms = self.tf_to_publish
        try:
            self.pub['tf'].publish(tf_msg)
        except Exception as error:  # pylint: disable=broad-except
            self.logwarn("Failed to publish message: {}".format(error))

        for publisher, msg in self.msgs_to_publish:
            try:
                publisher.publish(msg)
            except Exception as error:  # pylint: disable=broad-except
                self.logwarn("Failed to publish message: {}".format(error))
        self.msgs_to_publish = []
        self.tf_to_publish = []

    def publish_message(self, topic, msg, is_latched=False):
        """
        Function to publish ROS messages.

        Store the message in a list (waiting for their publication)
        with their associated publisher.
        If required corresponding publishers are created automatically.

        Messages for /tf topics are handle differently
        in order to publish all transforms, objects in the same message

        :param topic: ROS topic to publish the message on
        :type topic: string
        :param msg: the ROS message to be published
        :type msg: a valid ROS message type
        :return:
        """
        if topic == 'tf':
            # transform are merged in same message
            self.tf_to_publish.append(msg)
        else:
            if topic not in self.pub:
                if is_latched:
                    qos_profile = QoSProfile(depth=10, durability=latch)
                    self.pub[topic] = self.new_publisher(type(msg), topic, qos_profile=qos_profile, callback_group=self.callback_group)
                else:
                    # Use default QoS profile.
                    self.pub[topic] = self.new_publisher(type(msg), topic, callback_group=self.callback_group)
            self.msgs_to_publish.append((self.pub[topic], msg))

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if ROS_VERSION == 1:
            self.ros_timestamp = ros_timestamp(carla_timestamp.elapsed_seconds, from_sec=True)
            self.publish_message('clock', Clock(self.ros_timestamp))
        elif ROS_VERSION == 2:
            self.publish_message('clock', Clock())  # removed argument in ros 2 (?)

    def get_current_ros_time(self):
        """
        get the current ros time

        :return: the current ros time
        :rtype ros.Time
        """
        return self.ros_timestamp
