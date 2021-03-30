#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

from collections import deque
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=import-error
from ros_compatibility import QoSProfile, CompatibleNode, loginfo, ros_init, ROS_VERSION
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time
import math
from visualization_msgs.msg import Marker

import os
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from vehicle_pid_controller import VehiclePIDController
    from misc import distance_vehicle
elif ROS_VERSION == 2:
    from carla_ad_agent.vehicle_pid_controller import VehiclePIDController
    from carla_ad_agent.misc import distance_vehicle
    import rclpy


class LocalPlanner(CompatibleNode):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self):
        super(LocalPlanner, self).__init__('local_planner')

        # ros parameters
        role_name = self.get_param("role_name", "ego_vehicle")
        self.control_time_step = self.get_param("control_time_step", 0.05)
        args_lateral_dict = {}
        args_lateral_dict['K_P'] = self.get_param("Kp_lateral", 0.9)
        args_lateral_dict['K_I'] = self.get_param("Ki_lateral", 0.0)
        args_lateral_dict['K_D'] = self.get_param("Kd_lateral", 0.0)
        args_longitudinal_dict = {}
        args_longitudinal_dict['K_P'] = self.get_param("Kp_longitudinal", 0.206)
        args_longitudinal_dict['K_I'] = self.get_param("Ki_longitudinal", 0.0206)
        args_longitudinal_dict['K_D'] = self.get_param("Kd_longitudinal", 0.515)

        self.target_route_point = None
        self._vehicle_controller = None
        self._waypoints_queue = deque(maxlen=20000)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self.path_received = False
        self._current_speed = None
        self._current_pose = None
        self._target_speed = 0.0

        # subscribers
        self._odometry_subscriber = self.create_subscriber(
            Odometry, "/carla/{}/odometry".format(role_name), self.odometry_updated)
        self._path_subscriber = self.create_subscriber(
            Path, "/carla/{}/waypoints".format(role_name), self.path_updated, QoSProfile(depth=1, durability=True))
        self._target_speed_subscriber = self.create_subscriber(Float64, "/carla/{}/speed_command".format(
            role_name), self.target_speed_updated, QoSProfile(depth=1, durability=True))

        # publishers
        self._target_point_publisher = self.new_publisher(
            Marker, "/carla/{}/next_target".format(role_name), QoSProfile(depth=10, durability=False))
        self._control_cmd_publisher = self.new_publisher(
            CarlaEgoVehicleControl, "/carla/{}/vehicle_control_cmd".format(role_name), QoSProfile(depth=1, durability=False))

        # initializing controller
        self._vehicle_controller = VehiclePIDController(
            self, args_lateral=args_lateral_dict, args_longitudinal=args_longitudinal_dict)

        # wait for required messages
        self.loginfo('Local planner waiting for a path and target speed...')
        while self._current_pose is None or self._target_speed is None or self.path_received is False:
            time.sleep(0.05)
            if ROS_VERSION == 2:
                rclpy.spin_once(self, timeout_sec=0)

    def odometry_updated(self, new_pose):
        """
        callback on new odometry
        """
        self._current_speed = math.sqrt(new_pose.twist.twist.linear.x ** 2 +
                                        new_pose.twist.twist.linear.y ** 2 +
                                        new_pose.twist.twist.linear.z ** 2) * 3.6
        self._current_pose = new_pose.pose.pose

    def target_speed_updated(self, new_target_speed):
        self._target_speed = new_target_speed.data

    def path_updated(self, new_path):
        """
        set a global plan to follow
        """
        self.loginfo('New path to follow received.')
        self.path_received = True
        self.target_route_point = None
        self._waypoint_buffer.clear()
        self._waypoints_queue.clear()
        for elem in new_path.poses:
            self._waypoints_queue.append(elem.pose)

    def run_step(self):
        """
        Execute one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """
        if self.path_received is False:
            return

        if not self._waypoint_buffer and not self._waypoints_queue:
            self.emergency_stop()
            self.loginfo("Route finished. Waiting for a new one.")
            self.path_received = False
            return

        # When target speed is 0, brake
        if self._target_speed == 0.0:
            self.emergency_stop()
            return

        #   Buffering the waypoints
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(
                        self._waypoints_queue.popleft())
                else:
                    break

        # target waypoint
        self.target_route_point = self._waypoint_buffer[0]
        target_point = Marker()
        target_point.type = 0
        target_point.header.frame_id = "map"
        target_point.pose = self.target_route_point
        target_point.scale.x = 1.0
        target_point.scale.y = 0.2
        target_point.scale.z = 0.2
        target_point.color.r = 255.0
        target_point.color.a = 1.0
        self._target_point_publisher.publish(target_point)

        # move using PID controllers
        control = self._vehicle_controller.run_step(
            self._target_speed, self._current_speed, self._current_pose, self.target_route_point)

        # purge the queue of obsolete waypoints
        max_index = -1

        sampling_radius = self._target_speed * 1 / 3.6  # search radius for next waypoints in seconds
        min_distance = sampling_radius * self.MIN_DISTANCE_PERCENTAGE

        for i, route_point in enumerate(self._waypoint_buffer):
            if distance_vehicle(route_point, self._current_pose.position) < min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        self._control_cmd_publisher.publish(control)
        return

    def emergency_stop(self):
        control = CarlaEgoVehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False
        control.manual_gear_shift = False
        self._control_cmd_publisher.publish(control)


def main(args=None):
    """

    main function

    :return:
    """
    ros_init(args)
    local_planner = None
    update_timer = None
    try:
        local_planner = LocalPlanner()
        if ROS_VERSION == 1:
            local_planner.on_shutdown(local_planner.emergency_stop)
        local_planner.loginfo('Local planner is starting.')
        update_timer = local_planner.new_timer(
            local_planner.control_time_step, lambda timer_event=None: local_planner.run_step())
        local_planner.spin()
    except KeyboardInterrupt:
        pass
    finally:
        loginfo('Local planner shutting down.')
        if update_timer:
            if ROS_VERSION == 1:
                update_timer.shutdown()
            else:
                update_timer.destroy()
        if ROS_VERSION == 2:
            local_planner.emergency_stop()
        local_planner.shutdown()


if __name__ == "__main__":
    main()
