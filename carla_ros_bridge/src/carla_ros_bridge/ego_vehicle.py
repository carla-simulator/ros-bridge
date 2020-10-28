#!/usr/bin/env python

#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import math
import numpy

import rospy

from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Transform

from carla import VehicleControl
from carla import Vector3D

from carla_ros_bridge.vehicle import Vehicle
import carla_common.transforms as transforms

from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel,\
    CarlaEgoVehicleControl, CarlaEgoVehicleStatus


class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    def __init__(self, carla_actor, parent, node, vehicle_control_applied_callback):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """
        super(EgoVehicle, self).__init__(carla_actor=carla_actor,
                                         parent=parent,
                                         node=node,
                                         prefix=carla_actor.attributes.get('role_name'))

        self.vehicle_info_published = False
        self.vehicle_control_override = False
        self._vehicle_control_applied_callback = vehicle_control_applied_callback

        self.vehicle_status_publisher = rospy.Publisher(
            self.get_topic_prefix() + "/vehicle_status",
            CarlaEgoVehicleStatus,
            queue_size=10)
        self.vehicle_info_publisher = rospy.Publisher(self.get_topic_prefix() +
                                                      "/vehicle_info",
                                                      CarlaEgoVehicleInfo,
                                                      queue_size=10,
                                                      latch=True)

        self.control_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/vehicle_control_cmd",
            CarlaEgoVehicleControl,
            lambda data: self.control_command_updated(data, manual_override=False))

        self.manual_control_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/vehicle_control_cmd_manual",
            CarlaEgoVehicleControl,
            lambda data: self.control_command_updated(data, manual_override=True))

        self.control_override_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/vehicle_control_manual_override",
            Bool, self.control_command_override)

        self.enable_autopilot_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/enable_autopilot",
            Bool, self.enable_autopilot_updated)

        self.twist_control_subscriber = rospy.Subscriber(
            self.get_topic_prefix() + "/twist_cmd",
            Twist, self.twist_command_updated)

    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 255
        color.b = 0
        return color

    def send_vehicle_msgs(self):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_status = CarlaEgoVehicleStatus(
            header=self.get_msg_header("map"))
        vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_status.acceleration.linear = self.get_current_ros_accel().linear
        vehicle_status.orientation = self.get_current_ros_pose().orientation
        vehicle_status.control.throttle = self.carla_actor.get_control().throttle
        vehicle_status.control.steer = self.carla_actor.get_control().steer
        vehicle_status.control.brake = self.carla_actor.get_control().brake
        vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
        vehicle_status.control.reverse = self.carla_actor.get_control().reverse
        vehicle_status.control.gear = self.carla_actor.get_control().gear
        vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
        self.vehicle_status_publisher.publish(vehicle_status)

        # only send vehicle once (in latched-mode)
        if not self.vehicle_info_published:
            self.vehicle_info_published = True
            vehicle_info = CarlaEgoVehicleInfo()
            vehicle_info.id = self.carla_actor.id
            vehicle_info.type = self.carla_actor.type_id
            vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
            vehicle_physics = self.carla_actor.get_physics_control()

            for wheel in vehicle_physics.wheels:
                wheel_info = CarlaEgoVehicleInfoWheel()
                wheel_info.tire_friction = wheel.tire_friction
                wheel_info.damping_rate = wheel.damping_rate
                wheel_info.max_steer_angle = math.radians(wheel.max_steer_angle)
                wheel_info.radius = wheel.radius
                wheel_info.max_brake_torque = wheel.max_brake_torque
                wheel_info.max_handbrake_torque = wheel.max_handbrake_torque
                wheel_info.position.x = (wheel.position.x/100.0) - \
                    self.carla_actor.get_transform().location.x
                wheel_info.position.y = -((wheel.position.y/100.0) -
                                          self.carla_actor.get_transform().location.y)
                wheel_info.position.z = (wheel.position.z/100.0) - \
                    self.carla_actor.get_transform().location.z
                vehicle_info.wheels.append(wheel_info)

            vehicle_info.max_rpm = vehicle_physics.max_rpm
            vehicle_info.max_rpm = vehicle_physics.max_rpm
            vehicle_info.moi = vehicle_physics.moi
            vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
            vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
                vehicle_physics.damping_rate_zero_throttle_clutch_engaged
            vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
                vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
            vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
            vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
            vehicle_info.clutch_strength = vehicle_physics.clutch_strength
            vehicle_info.mass = vehicle_physics.mass
            vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
            vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
            vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
            vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

            self.vehicle_info_publisher.publish(vehicle_info)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        self.send_vehicle_msgs()
        super(EgoVehicle, self).update(frame, timestamp)
        no_rotation = Transform()
        no_rotation.rotation.w = 1.0
        self.publish_transform(self.get_ros_transform(
            no_rotation, frame_id=str(self.get_id()), child_frame_id=self.get_prefix()))

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroy Vehicle(id={})".format(self.get_id()))
        self.control_subscriber.unregister()
        self.control_subscriber = None
        self.enable_autopilot_subscriber.unregister()
        self.enable_autopilot_subscriber = None
        self.twist_control_subscriber.unregister()
        self.twist_control_subscriber = None
        self.control_override_subscriber.unregister()
        self.control_override_subscriber = None
        self.manual_control_subscriber.unregister()
        self.manual_control_subscriber = None
        super(EgoVehicle, self).destroy()

    def twist_command_updated(self, twist):
        """
        Set angular/linear velocity (this does not respect vehicle dynamics)
        """
        if not self.vehicle_control_override:
            angular_velocity = Vector3D()
            angular_velocity.z = math.degrees(twist.angular.z)

            rotation_matrix = transforms.carla_rotation_to_numpy_rotation_matrix(
                self.carla_actor.get_transform().rotation)
            linear_vector = numpy.array([twist.linear.x, twist.linear.y, twist.linear.z])
            rotated_linear_vector = rotation_matrix.dot(linear_vector)
            linear_velocity = Vector3D()
            linear_velocity.x = rotated_linear_vector[0]
            linear_velocity.y = -rotated_linear_vector[1]
            linear_velocity.z = rotated_linear_vector[2]

            rospy.logdebug("Set velocity linear: {}, angular: {}".format(
                linear_velocity, angular_velocity))
            self.carla_actor.set_target_velocity(linear_velocity)
            self.carla_actor.set_target_angular_velocity(angular_velocity)

    def control_command_override(self, enable):
        """
        Set the vehicle control mode according to ros topic
        """
        self.vehicle_control_override = enable.data

    def control_command_updated(self, ros_vehicle_control, manual_override):
        """
        Receive a CarlaEgoVehicleControl msg and send to CARLA

        This function gets called whenever a ROS CarlaEgoVehicleControl is received.
        If the mode is valid (either normal or manual), the received ROS message is
        converted into carla.VehicleControl command and sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param manual_override: manually override the vehicle control command
        :param ros_vehicle_control: current vehicle control input received via ROS
        :type ros_vehicle_control: carla_msgs.msg.CarlaEgoVehicleControl
        :return:
        """
        if manual_override == self.vehicle_control_override:
            vehicle_control = VehicleControl()
            vehicle_control.hand_brake = ros_vehicle_control.hand_brake
            vehicle_control.brake = ros_vehicle_control.brake
            vehicle_control.steer = ros_vehicle_control.steer
            vehicle_control.throttle = ros_vehicle_control.throttle
            vehicle_control.reverse = ros_vehicle_control.reverse
            self.carla_actor.apply_control(vehicle_control)
            self._vehicle_control_applied_callback(self.get_id())

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        rospy.logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.data))
        self.carla_actor.set_autopilot(enable_auto_pilot.data)

    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
            carla_vector.y * carla_vector.y + \
            carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
        return speed
