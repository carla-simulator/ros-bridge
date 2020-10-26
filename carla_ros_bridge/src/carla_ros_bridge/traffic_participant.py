#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic participants
"""

import rospy

from derived_object_msgs.msg import Object
from nav_msgs.msg import Odometry
from shape_msgs.msg import SolidPrimitive

from carla_ros_bridge.actor import Actor


class TrafficParticipant(Actor):

    """
    actor implementation details for traffic participant
    """

    def __init__(self, carla_actor, parent, node, prefix):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        self.classification_age = 0
        super(TrafficParticipant, self).__init__(carla_actor=carla_actor,
                                                 parent=parent,
                                                 node=node,
                                                 prefix=prefix)

        self.odometry_publisher = rospy.Publisher(self.get_topic_prefix() +
                                                  "/odometry",
                                                  Odometry,
                                                  queue_size=10)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update vehicles send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        self.publish_transform(self.get_ros_transform(None, None, str(self.get_id())))
        self.publish_marker()
        self.send_odometry()

        super(TrafficParticipant, self).update(frame, timestamp)

    def send_odometry(self):
        """
        Sends odometry
        :return:
        """
        odometry = Odometry(header=self.get_msg_header("map"))
        odometry.child_frame_id = self.get_prefix()
        odometry.pose.pose = self.get_current_ros_pose()
        odometry.twist.twist = self.get_current_ros_twist_rotated()
        self.odometry_publisher.publish(odometry)

    def get_object_info(self):
        """
        Function to send object messages of this traffic participant.

        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'

        :return:
        """
        obj = Object(header=self.get_msg_header("map"))
        # ID
        obj.id = self.get_id()
        # Pose
        obj.pose = self.get_current_ros_pose()
        # Twist
        obj.twist = self.get_current_ros_twist()
        # Acceleration
        obj.accel = self.get_current_ros_accel()
        # Shape
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])

        # Classification if available in attributes
        if self.get_classification() != Object.CLASSIFICATION_UNKNOWN:
            obj.object_classified = True
            obj.classification = self.get_classification()
            obj.classification_certainty = 255
            obj.classification_age = self.classification_age

        return obj

    def get_classification(self):  # pylint: disable=no-self-use
        """
        Function to get object classification (overridden in subclasses)
        """
        return Object.CLASSIFICATION_UNKNOWN
