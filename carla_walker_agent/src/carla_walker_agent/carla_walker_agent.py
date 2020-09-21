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
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Vector3
from carla_msgs.msg import CarlaWalkerControl


class CarlaWalkerAgent(object):
    """
    walker agent
    """
    # minimum distance to target waypoint before switching to next
    MIN_DISTANCE = 0.5

    def __init__(self, role_name, target_speed):
        """
        Constructor
        """
        self._route_assigned = False
        self._target_speed = target_speed
        self._waypoints = []
        self._current_pose = Pose()
        rospy.on_shutdown(self.on_shutdown)

        # wait for ros bridge to create relevant topics
        try:
            rospy.wait_for_message(
                "/carla/{}/odometry".format(role_name), Odometry)
        except rospy.ROSInterruptException as e:
            if not rospy.is_shutdown():
                raise e

        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)

        self.control_publisher = rospy.Publisher(
            "/carla/{}/walker_control_cmd".format(role_name), CarlaWalkerControl, queue_size=1)

        self._route_subscriber = rospy.Subscriber(
            "/carla/{}/waypoints".format(role_name), Path, self.path_updated)

        self._target_speed_subscriber = rospy.Subscriber(
            "/carla/{}/target_speed".format(role_name), Float64, self.target_speed_updated)

    def on_shutdown(self):
        """
        callback on shutdown
        """
        rospy.loginfo("Shutting down, stopping walker...")
        self.control_publisher.publish(CarlaWalkerControl())  # stop

    def target_speed_updated(self, target_speed):
        """
        callback on new target speed
        """
        rospy.loginfo("New target speed received: {}".format(target_speed.data))
        self._target_speed = target_speed.data

    def path_updated(self, path):
        """
        callback on new route
        """
        rospy.loginfo("New plan with {} waypoints received. Assigning plan...".format(
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
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
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
                        rospy.loginfo("next waypoint: {} {}".format(
                            self._waypoints[0].position.x, self._waypoints[0].position.y))
                    else:
                        rospy.loginfo("Route finished.")
                self.control_publisher.publish(control)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    """

    main function

    :return:
    """
    rospy.init_node('carla_walker_agent', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    target_speed = rospy.get_param("~target_speed", 20)
    controller = CarlaWalkerAgent(role_name, target_speed)
    try:
        controller.run()
    finally:
        del controller
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
