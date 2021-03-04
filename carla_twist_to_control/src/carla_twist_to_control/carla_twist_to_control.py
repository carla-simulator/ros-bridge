#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive geometry_nav_msgs::Twist and publish carla_msgs::CarlaEgoVehicleControl

use max wheel steer angle
"""
from ros_compatibility import (
    CompatibleNode,
    QoSProfile,
    latch_on,
    ros_ok,
    ROSException,
    ROSInterruptException,
    ros_init)
import sys
from geometry_msgs.msg import Twist  # pylint: disable=import-error
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo  # pylint: disable=import-error

import os
ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    import rospy  # pylint: disable=import-error
elif ROS_VERSION == 2:
    import rclpy  # pylint: disable=import-error


class TwistToVehicleControl(CompatibleNode):  # pylint: disable=too-few-public-methods
    """
    receive geometry_nav_msgs::Twist and publish carla_msgs::CarlaEgoVehicleControl

    use max wheel steer angle
    """

    MAX_LON_ACCELERATION = 10

    def __init__(self, rospy_init=True):
        """
        Constructor
        """
        super(TwistToVehicleControl, self).__init__("twist_to_control", rospy_init=rospy_init)
        self.max_steering_angle = None

    def initialize_twist_to_control(self, role_name):
        self.create_subscriber(CarlaEgoVehicleInfo,
                               "/carla/{}/vehicle_info".format(role_name), self.update_vehicle_info,
                               qos_profile=QoSProfile(depth=1, durability=latch_on))

        self.create_subscriber(Twist, "/carla/{}/twist".format(role_name), self.twist_received)

        self.pub = self.new_publisher(CarlaEgoVehicleControl,
                                      "/carla/{}/vehicle_control_cmd".format(role_name))

    def update_vehicle_info(self, vehicle_info):
        """
        callback to receive ego-vehicle info
        """
        if not vehicle_info.wheels:  # pylint: disable=no-member
            self.logerr("Cannot determine max steering angle: Vehicle has no wheels.")
            sys.exit(1)

        self.max_steering_angle = vehicle_info.wheels[0].max_steer_angle  # pylint: disable=no-member
        if not self.max_steering_angle:
            self.logerr("Cannot determine max steering angle: Value is %s",
                        self.max_steering_angle)
            sys.exit(1)
        self.loginfo("Vehicle info received. Max steering angle={}".format(self.max_steering_angle))

    def twist_received(self, twist):
        """
        receive twist and convert to carla vehicle control
        """
        if self.max_steering_angle is None:
            self.logwarn("Did not yet receive vehicle info.")
            return

        control = CarlaEgoVehicleControl()
        if twist == Twist():
            # stop
            control.throttle = 0.
            control.brake = 1.
            control.steer = 0.
        else:
            if twist.linear.x > 0:
                control.throttle = min(TwistToVehicleControl.MAX_LON_ACCELERATION,
                                       twist.linear.x) / TwistToVehicleControl.MAX_LON_ACCELERATION
            else:
                control.reverse = True
                control.throttle = max(-TwistToVehicleControl.MAX_LON_ACCELERATION,
                                       twist.linear.x) / -TwistToVehicleControl.MAX_LON_ACCELERATION

            if twist.angular.z > 0:
                control.steer = -min(self.max_steering_angle, twist.angular.z) / \
                    self.max_steering_angle
            else:
                control.steer = -max(-self.max_steering_angle, twist.angular.z) / \
                    self.max_steering_angle
        try:
            self.pub.publish(control)
        except ROSException as e:
            if ros_ok():
                self.logwarn("Error while publishing control: {}".format(e))


def main(args=None):
    """
    main function

    :return:
    """
    ros_init(args)

    role_name = None
    twist_to_vehicle_control = None

    if ROS_VERSION == 1:
        twist_to_vehicle_control = TwistToVehicleControl()
        role_name = rospy.get_param("~role_name", "ego_vehicle")
    elif ROS_VERSION == 2:
        twist_to_vehicle_control = TwistToVehicleControl()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(twist_to_vehicle_control)
        role_name = twist_to_vehicle_control.get_param("role_name", "ego_vehicle")

    twist_to_vehicle_control.initialize_twist_to_control(role_name)

    try:
        twist_to_vehicle_control.spin()
    finally:
        twist_to_vehicle_control.loginfo("Done, deleting twist to control")
        del twist_to_vehicle_control


if __name__ == "__main__":
    main()
