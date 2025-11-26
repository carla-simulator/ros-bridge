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

import json
import os

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

class SetInitialPose(CompatibleNode):

    def __init__(self):
        super(SetInitialPose, self).__init__("set_initial_pose")

        self.vehicle_role_names = []

        # Read vehicle role_name from file if provided
        self.objects_definition_file = self.get_param('objects_definition_file', '')
        if self.objects_definition_file:
            if not os.path.exists(self.objects_definition_file):
                raise RuntimeError(
                    "Could not read object definitions from {}".format(self.objects_definition_file))
            with open(self.objects_definition_file) as handle:
                json_actors = json.loads(handle.read())

        for actor in json_actors["objects"]:
            actor_type_split = actor["type"].split('.')
            actor_type = actor_type_split[0]
            if actor_type == "vehicle":
                self.vehicle_role_names.append(actor["id"])

        if len(self.vehicle_role_names) == 0:
            self.vehicle_role_names = self.get_param("role_name", "ego_vehicle")

        if len(self.vehicle_role_names) > 1:
            self.logwarn("Set initial pose only supports one single vehicle. Using {} from [{}]".format(self.vehicle_role_names[0], self.vehicle_role_names))

        self.transform_publisher = self.new_publisher(
            Pose,
            "/carla/vehicles/{}/control/set_transform".format(self.vehicle_role_names[0]),
            qos_profile=10)

        self.initial_pose_subscriber = self.new_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.intial_pose_callback,
            qos_profile=10)

    def intial_pose_callback(self, initial_pose):
        pose_to_publish = initial_pose.pose.pose
        pose_to_publish.position.z += 2.0
        self.transform_publisher.publish(pose_to_publish)


def main():
    """
    main function
    """
    roscomp.init("set_initial_pose")

    try:
        set_initial_pose_node = SetInitialPose()
        set_initial_pose_node.spin()
    except KeyboardInterrupt:
        roscomp.loginfo("Cancelled by user.")
    finally:
        if roscomp.ok():
            roscomp.shutdown()

if __name__ == '__main__':
    main()
