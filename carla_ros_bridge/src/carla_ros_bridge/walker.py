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
import rospy
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive

from carla_ros_bridge.actor import Actor
from carla_msgs.msg import CarlaWalkerControl
from carla import WalkerControl


class Walker(Actor):

    """
    Actor implementation details for pedestrians
    """

    def __init__(self, carla_actor, parent, communication, prefix=None):
        """
        Constructor

        :param carla_actor: carla walker actor object
        :type carla_actor: carla.Walker
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        if not prefix:
            prefix = "walker/{:03}".format(carla_actor.id)

        super(Walker, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     communication=communication,
                                     prefix=prefix)

        self.classification = Object.CLASSIFICATION_PEDESTRIAN
        self.classification_age = 0

        self.control_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/walker_control_cmd",
            CarlaWalkerControl, self.control_command_updated)

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

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update walkers send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        self.publish_transform(self.get_ros_transform())
        self.publish_marker()
        super(Walker, self).update(frame, timestamp)

    def get_object_info(self):
        """
        Function to send object messages of this walker

        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'

        :return:
        """
        walker_object = Object(header=self.get_msg_header("map"))
        # ID
        walker_object.id = self.get_id()
        # Pose
        walker_object.pose = self.get_current_ros_pose()
        # Twist
        walker_object.twist = self.get_current_ros_twist()
        # Acceleration
        walker_object.accel = self.get_current_ros_accel()
        # Shape
        walker_object.shape.type = SolidPrimitive.BOX
        walker_object.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])

        # Classification if available in attributes
        if self.classification != Object.CLASSIFICATION_UNKNOWN:
            walker_object.object_classified = True
            walker_object.classification = self.classification
            walker_object.classification_certainty = 1.0
            self.classification_age += 1
            walker_object.classification_age = self.classification_age

        return walker_object
