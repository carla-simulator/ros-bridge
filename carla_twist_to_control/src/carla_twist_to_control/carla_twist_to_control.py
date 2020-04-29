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
import sys
import rospy
from geometry_msgs.msg import Twist
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo


class TwistToVehicleControl(object):  # pylint: disable=too-few-public-methods
    """
    receive geometry_nav_msgs::Twist and publish carla_msgs::CarlaEgoVehicleControl

    use max wheel steer angle
    """

    MAX_LON_ACCELERATION = 10

    def __init__(self, role_name):
        """
        Constructor
        """
        rospy.loginfo("Wait for vehicle info...")
        try:
            vehicle_info = rospy.wait_for_message("/carla/{}/vehicle_info".format(role_name),
                                                  CarlaEgoVehicleInfo)
        except rospy.ROSInterruptException as e:
            if not rospy.is_shutdown():
                raise e
            else:
                sys.exit(0)
        if not vehicle_info.wheels:  # pylint: disable=no-member
            rospy.logerr("Cannot determine max steering angle: Vehicle has no wheels.")
            sys.exit(1)

        self.max_steering_angle = vehicle_info.wheels[0].max_steer_angle  # pylint: disable=no-member
        if not self.max_steering_angle:
            rospy.logerr("Cannot determine max steering angle: Value is %s",
                         self.max_steering_angle)
            sys.exit(1)
        rospy.loginfo("Vehicle info received. Max steering angle=%s",
                      self.max_steering_angle)

        rospy.Subscriber("/carla/{}/twist".format(role_name), Twist, self.twist_received)

        self.pub = rospy.Publisher("/carla/{}/vehicle_control_cmd".format(role_name),
                                   CarlaEgoVehicleControl, queue_size=1)

    def twist_received(self, twist):
        """
        receive twist and convert to carla vehicle control
        """
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
        except rospy.ROSException as e:
            if not rospy.is_shutdown():
                rospy.logwarn("Error while publishing control: {}".format(e))


def main():
    """
    main function

    :return:
    """
    rospy.init_node('convert_twist_to_vehicle_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    twist_to_vehicle_control = TwistToVehicleControl(role_name)
    try:
        rospy.spin()
    finally:
        del twist_to_vehicle_control
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
