#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
provide functions to control actors 
"""

import rospy
import numpy
import carla_common.transforms as trans
from carla_ros_bridge.pseudo_actor import PseudoActor
from geometry_msgs.msg import Pose, Twist


class ActorControl(PseudoActor):

    """
    provide functions to control actors 
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identifying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(ActorControl, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)

        self.set_location_subscriber = rospy.Subscriber(self.get_topic_prefix() +
                                                        "/set_transform",
                                                        Pose,
                                                        self.on_pose)

        self.twist_control_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/set_target_velocity",
            Twist, self.on_twist)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        self.set_location_subscriber.unregister()
        self.set_location_subscriber = None
        self.twist_control_subscriber.unregister()
        self.twist_control_subscriber = None
        super(ActorControl, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo actor
        :return: name
        """
        return "actor.pseudo.control"

    def on_pose(self, pose):
        if self.parent and self.parent.carla_actor.is_alive:
            self.parent.carla_actor.set_transform(trans.ros_pose_to_carla_transform(pose))

    def on_twist(self, twist):
        """
        Set angular/linear velocity (this does not respect vehicle dynamics)
        """
        if not self.vehicle_control_override:
            angular_velocity = Vector3D()
            angular_velocity.z = math.degrees(twist.angular.z)

            rotation_matrix = transforms.carla_rotation_to_numpy_rotation_matrix(
                self.carla_actor.get_transform().rotation)
            linear_vector = numpy.array([twist.linear.x, twist.linear.y, twist.linear.z])
            rotated_linear_vector = rotation_matrix.dot(linear_vector)
            linear_velocity = Vector3D()
            linear_velocity.x = rotated_linear_vector[0]
            linear_velocity.y = -rotated_linear_vector[1]
            linear_velocity.z = rotated_linear_vector[2]

            rospy.logdebug("Set velocity linear: {}, angular: {}".format(
                linear_velocity, angular_velocity))
            self.carla_actor.set_target_velocity(linear_velocity)
            self.carla_actor.set_target_angular_velocity(angular_velocity)
