#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math
import numpy as np
from transforms3d.euler import quat2euler
from geometry_msgs.msg import Point  # pylint: disable=import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=import-error


class VehiclePIDController(object):  # pylint: disable=too-few-public-methods
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal)
    to perform the low level control a vehicle from client side
    """

    def __init__(self, node, args_lateral=None, args_longitudinal=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller using
                             the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID
                                  controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        if not args_lateral:
            args_lateral = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        if not args_longitudinal:
            args_longitudinal = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}

        self.node = node
        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = PIDLateralController(**args_lateral)

    def run_step(self, target_speed, current_speed, current_pose, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: control signal (throttle and steering)
        """
        control = CarlaEgoVehicleControl()
        throttle = self._lon_controller.run_step(target_speed, current_speed)
        steering = self._lat_controller.run_step(current_pose, waypoint)
        control.steer = -steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

    def run_step(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        previous_error = self.error
        self.error = target_speed - current_speed
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -40.0, 40.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        return np.clip(output, 0.0, 1.0)


class PIDLateralController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.0):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._e_buffer = deque(maxlen=10)
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0

    def run_step(self, current_pose, waypoint):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param current_pose: current pose of the vehicle
        :return: steering control in the range [-1, 1]
        """
        v_begin = current_pose.position
        quaternion = (
            current_pose.orientation.w,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z
        )
        _, _, yaw = quat2euler(quaternion)
        v_end = Point()
        v_end.x = v_begin.x + math.cos(yaw)
        v_end.y = v_begin.y + math.sin(yaw)

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.position.x -
                          v_begin.x, waypoint.position.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        previous_error = self.error
        self.error = _dot
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -400.0, 400.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        return np.clip(output, -1.0, 1.0)
