#!/usr/bin/env python
#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Node to re-spawn vehicle in the ros-bridge

Subscribes to ROS topic /initialpose and publishes the pose on /carla/<role_name>/set_transform

Uses ROS parameter: role_name

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class SetInitialPose(object):

    def __init__(self):

        rospy.init_node('set_initial_pose', anonymous=True)

        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        # control_id should correspond to the id of the actor.pseudo.control
        # actor that is set in the config file used to spawn it
        self.control_id = rospy.get_param('~control_id', 'control')
        self.transform_publisher = rospy.Publisher(
            "/carla/{}/{}/set_transform".format(self.role_name, self.control_id), Pose, queue_size=10)

        self.initial_pose_subscriber = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.intial_pose_callback)

    def intial_pose_callback(self, initial_pose):
        pose_to_publish = initial_pose.pose.pose
        pose_to_publish.position.z += 2.0
        self.transform_publisher.publish(pose_to_publish)


def main():
    """
    main function
    """
    set_initial_pose_node = SetInitialPose()
    rospy.spin()


if __name__ == '__main__':
    main()
