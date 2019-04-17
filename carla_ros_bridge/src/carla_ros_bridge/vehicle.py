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

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.binding.binding import VehicleClass


class Vehicle(Actor):

    """
    Actor implementation details for vehicles
    """

    def __init__(self, carla_actor, parent, binding, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        if topic_prefix is None:
            topic_prefix = "vehicle/{:03}".format(
                Actor.global_id_registry.get_id(carla_actor.id))

        super(Vehicle, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      binding=binding,
                                      topic_prefix=topic_prefix)

        self.classification = VehicleClass.UNKNOWN
        if carla_actor.attributes.has_key('object_type'):
            if carla_actor.attributes['object_type'] == 'car':
                self.classification = VehicleClass.CAR
            elif carla_actor.attributes['object_type'] == 'bike':
                self.classification = VehicleClass.BIKE
            elif carla_actor.attributes['object_type'] == 'motorcycle':
                self.classification = VehicleClass.MOTORCYCLE
            elif carla_actor.attributes['object_type'] == 'truck':
                self.classification = VehicleClass.TRUCK
            elif carla_actor.attributes['object_type'] == 'other':
                self.classification = VehicleClass.VEHICLE
        self.classification_age = 0

    def destroy(self):
        """
        Function (override) to destroy this object.

        Finally forward call to super class.

        :return:
        """
        self.get_binding().logdebug("Destroy Vehicle(id={})".format(self.get_id()))
        super(Vehicle, self).destroy()

    def update(self):
        """
        Function (override) to update this object.

        On update vehicles send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        self.get_binding().publish_transform(self.get_topic_prefix(),
                                             self.carla_actor.get_transform())
        self.get_binding().publish_marker(self.get_topic_prefix(),
                                          self.carla_actor.bounding_box,
                                          self.get_marker_color(),
                                          self.get_global_id())
        super(Vehicle, self).update()

    @classmethod
    def get_marker_color(cls):
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a vehicle marker
        """
        return (255, 0, 0)

    def get_object_info(self):
        """
        get info about the object, used for simulated object-sensors
        """
        vehicle_object = {}
        vehicle_object['id'] = self.get_global_id()
        vehicle_object['transform'] = self.carla_actor.get_transform()
        vehicle_object['accel'] = self.carla_actor.get_acceleration()
        vehicle_object['velocity'] = self.carla_actor.get_velocity()
        vehicle_object['bounding_box'] = self.carla_actor.bounding_box
        vehicle_object['classification'] = self.classification
        vehicle_object['classification_age'] = self.classification_age
        return vehicle_object
