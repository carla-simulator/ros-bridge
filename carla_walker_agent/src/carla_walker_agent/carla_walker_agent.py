#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Agent for Walker
"""
import math
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Vector3
from carla_msgs.msg import CarlaWalkerControl
from ros_compatibility import (
    CompatibleNode,
    QoSProfile,
    ros_ok,
    ROSInterruptException,
    ros_init)

import os
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    import rospy
elif ROS_VERSION == 2:
    import time
    import threading


class CarlaWalkerAgent(CompatibleNode):
    """
    walker agent
    """
    # minimum distance to target waypoint before switching to next
    MIN_DISTANCE = 0.5

    def __init__(self):
        """
        Constructor
        """
        super(CarlaWalkerAgent, self).__init__('carla_walker_agent')

        role_name = self.get_param("role_name", "ego_vehicle")
        self._target_speed = self.get_param("target_speed", 2.0)

        self._route_assigned = False
        self._waypoints = []
        self._current_pose = Pose()
        self.on_shutdown(self._on_shutdown)

        # wait for ros bridge to create relevant topics
        try:
            self.wait_for_one_message(
                "/carla/{}/odometry".format(role_name), Odometry)
        except ROSInterruptException as e:
            if not ros_ok:
                raise e

        self._odometry_subscriber = self.create_subscriber(
            Odometry, "/carla/{}/odometry".format(role_name), self.odometry_updated)

        self.control_publisher = self.new_publisher(
            CarlaWalkerControl, "/carla/{}/walker_control_cmd".format(role_name),
            QoSProfile(depth=1, durability=False))

        self._route_subscriber = self.create_subscriber(
            Path, "/carla/{}/waypoints".format(role_name), self.path_updated)

        self._target_speed_subscriber = self.create_subscriber(
            Float64,  "/carla/{}/target_speed".format(role_name), self.target_speed_updated)

    def _on_shutdown(self):
        """
        callback on shutdown
        """
        self.loginfo("Shutting down, stopping walker...")
        self.control_publisher.publish(CarlaWalkerControl())  # stop

    def target_speed_updated(self, target_speed):
        """
        callback on new target speed
        """
        self.loginfo("New target speed received: {}".format(target_speed.data))
        self._target_speed = target_speed.data

    def path_updated(self, path):
        """
        callback on new route
        """
        self.loginfo("New plan with {} waypoints received. Assigning plan...".format(
            len(path.poses)))
        self.control_publisher.publish(CarlaWalkerControl())  # stop
        self._waypoints = []
        for elem in path.poses:
            self._waypoints.append(elem.pose)

    def odometry_updated(self, odo):
        """
        callback on new odometry
        """
        self._current_pose = odo.pose.pose

    def run(self):
        """

        Control loop

        :return:
        """
        loop_frequency = 20
        if ROS_VERSION == 1:
            r = rospy.Rate(loop_frequency)
        self.loginfo("Starting run loop")
        while ros_ok():
            if self._waypoints:
                control = CarlaWalkerControl()
                direction = Vector3()
                direction.x = self._waypoints[0].position.x - self._current_pose.position.x
                direction.y = self._waypoints[0].position.y - self._current_pose.position.y
                direction_norm = math.sqrt(direction.x**2 + direction.y**2)
                if direction_norm > CarlaWalkerAgent.MIN_DISTANCE:
                    control.speed = self._target_speed
                    control.direction.x = direction.x / direction_norm
                    control.direction.y = direction.y / direction_norm
                else:
                    self._waypoints = self._waypoints[1:]
                    if self._waypoints:
                        self.loginfo("next waypoint: {} {}".format(
                            self._waypoints[0].position.x, self._waypoints[0].position.y))
                    else:
                        self.loginfo("Route finished.")
                self.control_publisher.publish(control)
            try:
                if ROS_VERSION == 1:
                    r.sleep()
                elif ROS_VERSION == 2:
                    # TODO: use rclpy.Rate, not working yet
                    time.sleep(1 / loop_frequency)
            except ROSInterruptException:
                pass


def main(args=None):
    """

    main function

    :return:
    """
    ros_init(args)

    controller = CarlaWalkerAgent()

    if ROS_VERSION == 2:
        spin_thread = threading.Thread(target=controller.spin, daemon=True)
        spin_thread.start()

    try:
        controller.run()
    finally:
        del controller
        print("Done")


if __name__ == "__main__":
    main()
