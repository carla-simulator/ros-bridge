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
from carla_ad_agent.basic_agent import BasicAgent
from ros_compatibility import CompatibleNode, ROSInterruptException, ros_ok, QoSProfile, ROSException
import threading

import os
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    import rospy
elif ROS_VERSION == 2:
    import rclpy

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
        self.on_shutdown(self._on_shutdown)


        # wait for ego vehicle
        vehicle_info = None
        try:
            self.loginfo("Wait for vehicle info ...")
            vehicle_info = self.wait_for_one_message(
                "/carla/{}/vehicle_info".format(role_name), CarlaEgoVehicleInfo)
            self.loginfo("Vehicle info recieved.")
        except ROSException: 
            self.logerr("Timeout while waiting for vehicle info!")
            sys.exit(1)

        self._route_subscriber =self.create_subscriber(
            Path, "/carla/{}/waypoints".format(role_name), self.path_updated)

        self._target_speed_subscriber = self.create_subscriber(
            Float64, "/carla/{}/target_speed".format(role_name), self.target_speed_updated)

        self.vehicle_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl, "/carla/{}/vehicle_control_cmd".format(role_name), 
                QoSProfile(depth=1, durability=False))
        self._agent = BasicAgent(role_name, vehicle_info.id,  # pylint: disable=no-member
                                 self, avoid_risk)

    def _on_shutdown(self):
        """
        callback on shutdown
        """
        self.loginfo("Shutting down, stopping ego vehicle...")
        if self._agent:
            self.vehicle_control_publisher.publish(self._agent.emergency_stop())

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
    
    def spin_loop(self):
        while ros_ok:
            self.spin_once()

    def run(self):
        """

        Control loop

        :return:
        """
        loop_frequency = 10

        if ROS_VERSION == 1:
            r = rospy.Rate(loop_frequency)

        while ros_ok():
            self.spin_once()
            if self._global_plan:
                control = self.run_step()
                if control:
                    control.steer = -control.steer
                    self.vehicle_control_publisher.publish(control)
            else:
                try:
                    if ROS_VERSION == 1:
                        r.sleep()
                    elif ROS_VERSION == 2:
                        #TODO: use rclpy.Rate, not working yet
                        time.sleep(1 / loop_frequency)
                except ROSInterruptException:
                    pass
            


def main(args=None):
    """

    main function

    :return:
    """
    if ROS_VERSION == 2:
        rclpy.init()

    controller = CarlaAdAgent()
    try:
        controller.run()

    finally:
        del controller
        print("Done")


if __name__ == "__main__":
    main()
