#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a object sensor
"""

import carla
import carla_common.transforms as trans
import ctypes
import math

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from transforms3d.euler import euler2quat

class ObjectSensor(PseudoActor):

    """
    Pseudo object sensor
    """

    OBJECT_LABELS = {
        carla.CityObjectLabel.Car: Object.CLASSIFICATION_CAR,
        carla.CityObjectLabel.Truck: Object.CLASSIFICATION_TRUCK,
        carla.CityObjectLabel.Bus: Object.CLASSIFICATION_OTHER_VEHICLE,
        carla.CityObjectLabel.Motorcycle:Object.CLASSIFICATION_MOTORCYCLE,
        carla.CityObjectLabel.Bicycle: Object.CLASSIFICATION_BIKE,
        carla.CityObjectLabel.Pedestrians: Object.CLASSIFICATION_PEDESTRIAN
    }

    def __init__(self, uid, name, parent, node, actor_list, world, attributes=()):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        :param attributes: per-sensor spawn attributes (diagnostic_msgs/KeyValue[])
        """

        super(ObjectSensor, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)
        self.actor_list = actor_list
        self.world = world
        self.node = node
        self._publish_period = float(next(
            (attribute.value for attribute in attributes if attribute.key == 'sensor_tick'), 0.0))
        if not math.isfinite(self._publish_period) or self._publish_period < 0.0:
            raise ValueError("sensor_tick must be finite and non-negative")
        self._next_publish_timestamp = None
        self._last_update_timestamp = None
        self._static_vehicles = None
        self.object_publisher = node.new_publisher(ObjectArray,
                                                   self.get_topic_prefix(),
                                                   qos_profile=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(ObjectSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.objects"
    
    def _get_vehicle_from_environment_objects(self, environment_object, object_classification):
        obj = Object(header=self.get_msg_header("carla_map"))
        obj.id = ctypes.c_uint32(environment_object.id).value
        obj.pose = trans.carla_transform_to_ros_pose(
            self._get_environment_object_transform(environment_object))

        if self.node.parameters['ignore_altitude']:
            obj.pose.position.z = 0.0

        if self.node.parameters['ignore_tilt']:
            obj.pose.orientation = trans.reset_tilt_angles(
                obj.pose.orientation)

        # only static obj
        obj.twist = trans.carla_velocity_to_ros_twist(carla.Vector3D(0.0, 0.0, 0.0), carla.Vector3D(0.0, 0.0, 0.0))
        obj.accel = trans.carla_acceleration_to_ros_accel(carla.Vector3D(0.0, 0.0, 0.0))
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            environment_object.bounding_box.extent.x * 2.0,
            environment_object.bounding_box.extent.y * 2.0,
            environment_object.bounding_box.extent.z * 2.0])
        obj.classification = object_classification
        obj.classification_certainty = 1
        obj.object_classified = True

        return obj

    def _get_environment_object_transform(self, environment_object):
        box = environment_object.bounding_box
        return carla.Transform(box.location, box.rotation)

    def _get_environment_object_world_vertices(self, environment_object):
        return environment_object.bounding_box.get_world_vertices(carla.Transform())

    def _get_static_vehicles(self):
        """
        Get the static vehicles of the map as pairs of the CARLA environment object and its object
        message. The header of the object message is set when it is published.

        Static vehicles do not move, so they are queried and converted only once: each query is a
        round trip that waits while the server computes a frame.
        """
        if self._static_vehicles is None:
            self._static_vehicles = []
            # iterate over all possible static vehicles
            for object_key, object_value in self.OBJECT_LABELS.items():
                static_vehicles = self.world.get_environment_objects(object_key)
                for vehicle in static_vehicles:
                    # take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):
                        vehicle_obj = self._get_vehicle_from_environment_objects(
                            vehicle, object_value)
                        self._static_vehicles.append((vehicle, vehicle_obj))
        return self._static_vehicles

    def _should_publish(self, timestamp):
        """Check and advance this sensor's simulation-time publication deadline."""
        # determine whether to publish this tick or skip, if publication rate is set
        if self._publish_period > 0.0:
            if self._last_update_timestamp is None or timestamp < self._last_update_timestamp:
                self._next_publish_timestamp = timestamp
            self._last_update_timestamp = timestamp
            # allow for rounding in CARLA's simulation timestamps
            if timestamp + 1e-6 < self._next_publish_timestamp:
                return False
            # retain phase for non-integer tick ratios and skip missed deadlines without bursts
            periods = max(1, math.floor(
                (timestamp + 1e-6 - self._next_publish_timestamp) / self._publish_period) + 1)
            self._next_publish_timestamp += periods * self._publish_period

        return True

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update carla_map sends:
        - tf global frame
        :return:
        """

        if not self._should_publish(timestamp):
            return

        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="carla_map", timestamp=timestamp)
        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle):
                    ros_objects.objects.append(actor.get_object_info(frame))
                elif isinstance(actor, Walker):
                    ros_objects.objects.append(actor.get_object_info(frame))

        if(self.node.parameters['publish_static_vehicles']):
            # add also static vehicles to ros_objects.object array
            header = self.get_msg_header("carla_map")
            for _, vehicle_obj in self._get_static_vehicles():
                vehicle_obj.header = header
                ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)
