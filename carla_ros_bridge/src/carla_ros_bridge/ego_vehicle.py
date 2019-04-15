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

from carla import VehicleControl

from carla_ros_bridge.vehicle import Vehicle


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

        self.get_binding().register_vehicle_control_subscriber(
            self.topic_name(), self.control_command_updated)
        self.get_binding().register_vehicle_autopilot_subscriber(
            self.topic_name(), self.enable_autopilot_updated)

    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        """
        return (0, 255, 0)

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        objects = super(EgoVehicle, self).get_filtered_objectarray(self.carla_actor.id)
        self.get_binding().publish_objects(self.topic_name() + '/objects', objects)

        if not self.vehicle_info_published:
            self.vehicle_info_published = True
            self.get_binding().publish_ego_vehicle_info(
                self.topic_name(),
                self.carla_actor.type_id,
                self.carla_actor.attributes.get('role_name'),
                self.carla_actor.get_physics_control())
        self.get_binding().publish_ego_vehicle_status(self.topic_name(),
                                                      self.get_frame_id(),
                                                      self.carla_actor.get_velocity(),
                                                      self.carla_actor.get_transform(),
                                                      self.carla_actor.get_control(),
                                                      self.carla_actor.get_acceleration())
        super(EgoVehicle, self).update()

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on CarlaEgoVehicleControl commands.
        Finally forward call to super class.

        :return:
        """
        self.get_binding().logdebug("Destroy Vehicle(id={})".format(self.get_id()))
        super(EgoVehicle, self).destroy()

    def control_command_updated(self, vehicle_control):
        """
        Receive a CarlaEgoVehicleControl msg and send to CARLA

        :param vehicle_control: current vehicle control input received via binding
        :type vehicle_control: carla_msgs.msg.CarlaEgoVehicleControl
        :return:
        """
        self.carla_actor.apply_control(vehicle_control)

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        self.get_binding().logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot))
        self.carla_actor.set_autopilot(enable_auto_pilot)
