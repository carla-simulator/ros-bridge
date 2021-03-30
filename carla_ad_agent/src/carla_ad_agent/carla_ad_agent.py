#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
A basic AD agent using CARLA waypoints
"""
import sys
import time
from std_msgs.msg import Float64  # pylint: disable=import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=import-error
from ros_compatibility import (
    CompatibleNode,
    ROSInterruptException,
    ros_ok,
    QoSProfile,
    ROSException,
    latch_on,
    ros_init,
    ROS_VERSION,
    logwarn,
    loginfo,
    ros_shutdown)

if ROS_VERSION == 1:
    # TODO: different ways to import the carla_ad_agent submodules (e.g. carla_ad_agent.basic_agent) between ros1 and ros2 shouldn't be necessary
    from basic_agent import BasicAgent
elif ROS_VERSION == 2:
    import rclpy
    from carla_ad_agent.basic_agent import BasicAgent


class CarlaAdAgent(CompatibleNode):
    """
    A basic AD agent using CARLA waypoints
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaAdAgent, self).__init__('carla_ad_agent')

        role_name = self.get_param("role_name", "ego_vehicle")
        avoid_risk = self.get_param("avoid_risk", True)

        self.target_speed = self.get_param("target_speed", 30.0)
        self.agent = None

        # wait for ego vehicle
        vehicle_info = None
        self.loginfo("Wait for vehicle info ...")
        vehicle_info = self.wait_for_one_message("/carla/{}/vehicle_info".format(
            role_name), CarlaEgoVehicleInfo, qos_profile=QoSProfile(depth=1, durability=latch_on))
        self.loginfo("Vehicle info received.")

        self.agent = BasicAgent(role_name, vehicle_info.id, self, avoid_risk)

        self.target_speed_subscriber = self.create_subscriber(
            Float64, "/carla/{}/target_speed".format(role_name), self.target_speed_updated, QoSProfile(depth=1, durability=True))

        self.speed_command_publisher = self.new_publisher(
            Float64, "/carla/{}/speed_command".format(role_name), QoSProfile(depth=1, durability=True))

    def target_speed_updated(self, target_speed):
        """
        callback on new target speed
        """
        self.loginfo("New target speed received: {}".format(target_speed.data))
        self.target_speed = target_speed.data

    def run_step(self):
        """
        Execute one step of navigation.
        """
        if not self.agent:
            self.loginfo("Waiting for ego vehicle...")
        else:
            hazard_detected = self.agent.run_step()
            speed_command = Float64()
            if hazard_detected:
                # stop, publish speed 0.0km/h
                speed_command.data = 0.0
                self.speed_command_publisher.publish(speed_command)
            else:
                speed_command.data = self.target_speed
                self.speed_command_publisher.publish(speed_command)
        return


def main(args=None):
    """

    main function

    :return:
    """
    ros_init(args)
    controller = None
    try:
        controller = CarlaAdAgent()
        while ros_ok():
            time.sleep(0.01)
            if ROS_VERSION == 2:
                rclpy.spin_once(controller)
            controller.run_step()
    except (ROSInterruptException, ROSException) as e:
        if ros_ok():
            logwarn("ROS Error during exection: {}".format(e))
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        if controller is not None:
            stopping_speed = Float64()
            stopping_speed.data = 0.0
            controller.speed_command_publisher.publish(stopping_speed)
        ros_shutdown()


if __name__ == "__main__":
    main()
