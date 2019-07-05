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
import rospy

from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


class Communication(object):

    """
    Handle communication of ROS topics
    """

    def __init__(self):
        """
        Constructor
        """
        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.publishers = {}
        self.subscribers = {}
        self.ros_timestamp = rospy.Time()

        # needed?
        self.publishers['clock'] = rospy.Publisher(
            'clock', Clock, queue_size=10)
        self.publishers['tf'] = rospy.Publisher(
            'tf', TFMessage, queue_size=100)

    def send_msgs(self):
        """
        Function to actually send the collected ROS messages out via the publisher

        :return:
        """
        # prepare tf message
        tf_msg = TFMessage(self.tf_to_publish)
        try:
            self.publishers['tf'].publish(tf_msg)
        except rospy.ROSSerializationException as error:
            rospy.logwarn("Failed to serialize message on publishing: {}".format(error))
        except Exception as error:  # pylint: disable=broad-except
            rospy.logwarn("Failed to publish message: {}".format(error))

        for publisher, msg in self.msgs_to_publish:
            try:
                publisher.publish(msg)
            except rospy.ROSSerializationException as error:  # pylint: disable=broad-except
                rospy.logwarn("Failed to serialize message on publishing: {}".format(error))
            except Exception as error:  # pylint: disable=broad-except
                rospy.logwarn("Failed to publish message: {}".format(error))
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
            if topic not in self.publishers:
                self.publishers[topic] = rospy.Publisher(
                    topic, type(msg), queue_size=10, latch=is_latched)
            self.msgs_to_publish.append((self.publishers[topic], msg))

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.ros_timestamp = rospy.Time.from_sec(
            carla_timestamp.elapsed_seconds)
        self.publish_message('clock', Clock(self.ros_timestamp))

    def get_current_ros_time(self):
        """
        get the current ros time

        :return: the current ros time
        :rtype ros.Time
        """
        return self.ros_timestamp
