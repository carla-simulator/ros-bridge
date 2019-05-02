#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import math

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool

from carla import VehicleControl

from carla_ros_bridge.vehicle import Vehicle
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleInfoWheel
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus


class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static factory method to create ego vehicle actors

        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of the new traffic actor
        :type parent: carla_ros_bridge.Parent
        :return: the created vehicle actor
        :rtype: carla_ros_bridge.Vehicle or derived type
        """
        return EgoVehicle(carla_actor=carla_actor, parent=parent)

    def __init__(self, carla_actor, parent):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        """
        super(EgoVehicle, self).__init__(carla_actor=carla_actor,
                                         parent=parent,
                                         topic_prefix=carla_actor.attributes.get('role_name'),
                                         append_role_name_topic_postfix=False)

        self.vehicle_info_published = False

        self.control_subscriber = rospy.Subscriber(
            self.topic_name() + "/vehicle_control_cmd",
            CarlaEgoVehicleControl, self.control_command_updated)

        self.enable_autopilot_subscriber = rospy.Subscriber(
            self.topic_name() + "/enable_autopilot",
            Bool, self.enable_autopilot_updated)

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
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published

        :return:
        """
        vehicle_status = CarlaEgoVehicleStatus()
        vehicle_status.header.stamp = self.get_current_ros_time()
        vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_status.acceleration = self.get_vehicle_acceleration_abs(self.carla_actor)
        vehicle_status.orientation = self.get_current_ros_pose().orientation
        vehicle_status.control.throttle = self.carla_actor.get_control().throttle
        vehicle_status.control.steer = self.carla_actor.get_control().steer
        vehicle_status.control.brake = self.carla_actor.get_control().brake
        vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
        vehicle_status.control.reverse = self.carla_actor.get_control().reverse
        vehicle_status.control.gear = self.carla_actor.get_control().gear
        vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
        self.publish_ros_message(self.topic_name() + "/vehicle_status", vehicle_status)

        if not self.vehicle_info_published:
            self.vehicle_info_published = True
            vehicle_info = CarlaEgoVehicleInfo()
            vehicle_info.type = self.carla_actor.type_id
            vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
            vehicle_physics = self.carla_actor.get_physics_control()

            for wheel in vehicle_physics.wheels:
                wheel_info = CarlaEgoVehicleInfoWheel()
                wheel_info.tire_friction = wheel.tire_friction
                wheel_info.damping_rate = wheel.damping_rate
                wheel_info.steer_angle = math.radians(wheel.steer_angle)
                wheel_info.disable_steering = wheel.disable_steering
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

            self.publish_ros_message(self.topic_name() + "/vehicle_info", vehicle_info, True)

        # @todo: do we still need this?
        odometry = Odometry(header=self.get_msg_header())
        odometry.child_frame_id = self.get_frame_id()
        odometry.pose.pose = self.get_current_ros_pose()
        odometry.twist.twist = self.get_current_ros_twist()

        self.publish_ros_message(self.topic_name() + "/odometry", odometry)

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        objects = super(EgoVehicle, self).get_filtered_objectarray(self.carla_actor.id)
        self.publish_ros_message(self.topic_name() + '/objects', objects)
        self.send_vehicle_msgs()
        super(EgoVehicle, self).update()

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on CarlaEgoVehicleControl commands.
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroy Vehicle(id={})".format(self.get_id()))
        self.control_subscriber.unregister()
        self.control_subscriber = None
        self.enable_autopilot_subscriber.unregister()
        self.enable_autopilot_subscriber = None
        super(EgoVehicle, self).destroy()

    def control_command_updated(self, ros_vehicle_control):
        """
        Receive a CarlaEgoVehicleControl msg and send to CARLA

        This function gets called whenever a ROS message is received via
        '/carla/ego_vehicle/vehicle_control_cmd' topic.
        The received ROS message is converted into carla.VehicleControl command and
        sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param ros_vehicle_control: current vehicle control input received via ROS
        :type ros_vehicle_control: carla_msgs.msg.CarlaEgoVehicleControl
        :return:
        """
        vehicle_control = VehicleControl()
        vehicle_control.hand_brake = ros_vehicle_control.hand_brake
        vehicle_control.brake = ros_vehicle_control.brake
        vehicle_control.steer = ros_vehicle_control.steer
        vehicle_control.throttle = ros_vehicle_control.throttle
        vehicle_control.reverse = ros_vehicle_control.reverse
        self.carla_actor.apply_control(vehicle_control)

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

    @staticmethod
    def get_vehicle_acceleration_abs(carla_vehicle):
        """
        Get the absolute acceleration of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: vehicle acceleration value [m/s^2 >=0]
        :rtype: float64
        """
        return math.sqrt(EgoVehicle.get_vector_length_squared(carla_vehicle.get_acceleration()))
