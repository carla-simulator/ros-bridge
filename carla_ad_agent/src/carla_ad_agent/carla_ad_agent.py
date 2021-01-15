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
from nav_msgs.msg import Path  # pylint: disable=import-error
from std_msgs.msg import Float64  # pylint: disable=import-error
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleControl  # pylint: disable=import-error
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
    import rospy
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

        self._target_speed = self.get_param("target_speed", 20)
        self._route_assigned = False
        self._global_plan = None
        self._agent = None

        # wait for ego vehicle
        vehicle_info = None
        self.loginfo("Wait for vehicle info ...")
        vehicle_info = self.wait_for_one_message(
            "/carla/{}/vehicle_info".format(role_name), CarlaEgoVehicleInfo,
            qos_profile=QoSProfile(depth=1, durability=latch_on))
        self.loginfo("Vehicle info received.")

        self._agent = BasicAgent(role_name, vehicle_info.id,  # pylint: disable=no-member
                                 self, avoid_risk)

        self._route_subscriber = self.create_subscriber(
            Path, "/carla/{}/waypoints".format(role_name), self.path_updated,
            QoSProfile(depth=1, durability=True))

        self._target_speed_subscriber = self.create_subscriber(
            Float64, "/carla/{}/target_speed".format(role_name), self.target_speed_updated,
            QoSProfile(depth=1, durability=True))

        self.vehicle_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl, "/carla/{}/vehicle_control_cmd".format(role_name),
            QoSProfile(depth=1, durability=False))

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
        self.loginfo("New plan with {} waypoints received.".format(len(path.poses)))
        if self._agent:
            self.vehicle_control_publisher.publish(self._agent.emergency_stop())
        self._global_plan = path
        self._route_assigned = False

    def run_step(self):
        """
        Execute one step of navigation.
        """
        control = CarlaEgoVehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False

        if not self._agent:
            self.loginfo("Waiting for ego vehicle...")
            return control

        if not self._route_assigned and self._global_plan:
            self.loginfo("Assigning plan...")
            self._agent._local_planner.set_global_plan(  # pylint: disable=protected-access
                self._global_plan.poses)
            self._route_assigned = True
        else:
            control, finished = self._agent.run_step(self._target_speed)
            if finished:
                self._global_plan = None
                self._route_assigned = False

        return control

    def run(self):
        """

        Control loop

        :return:
        """
        loop_frequency = 10

        if ROS_VERSION == 1:
            r = rospy.Rate(loop_frequency)

        while ros_ok():
            if ROS_VERSION == 2:
                rclpy.spin_once(self)
            if self._global_plan:
                control = self.run_step()
                if control:
                    control.steer = -control.steer
                    self.vehicle_control_publisher.publish(control)
            else:
                if ROS_VERSION == 1:
                    r.sleep()
                elif ROS_VERSION == 2:
                    # TODO: use rclpy.Rate, not working yet
                    time.sleep(1 / loop_frequency)


def main(args=None):
    """

    main function

    :return:
    """
    ros_init(args)
    controller = None
    try:
        controller = CarlaAdAgent()
        controller.run()
    except (ROSInterruptException, ROSException) as e:
        if ros_ok():
            logwarn("ROS Error during exection: {}".format(e))
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        if controller is not None and controller._agent:
            controller.vehicle_control_publisher.publish(controller._agent.emergency_stop())
        ros_shutdown()


if __name__ == "__main__":
    main()
