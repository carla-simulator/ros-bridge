#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a marker sensor
"""

import itertools

import carla

from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic_participant import TrafficParticipant
from carla_common.transforms import carla_location_to_ros_point, carla_rotation_to_ros_quaternion

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

# Using colors from CityScapesPalette specified here:
# https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
OBJECT_LABELS = {
    #carla.CityObjectLabel.None: ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.8),
    carla.CityObjectLabel.Buildings: ColorRGBA(r=70.0/255.0, g=70.0/255.0, b=70.0/255.0, a=0.8),
    carla.CityObjectLabel.Fences: ColorRGBA(r=100.0/255.0, g=40.0/255.0, b=40.0/255.0, a=0.8),
    #carla.CityObjectLabel.Other: ColorRGBA(r=55.0/255.0, g=90.0/255.0, b=80.0/255.0, a=0.8),
    #carla.CityObjectLabel.Pedestrians: ColorRGBA(r=220.0/255.0, g=20.0/255.0, b=60.0/255.0, a=0.8),
    carla.CityObjectLabel.Poles: ColorRGBA(r=153.0/255.0, g=153.0/255.0, b=153.0/255.0, a=0.8),
    carla.CityObjectLabel.RoadLines: ColorRGBA(r=157.0/255.0, g=234.0/255.0, b=50.0/255.0, a=0.8),
    carla.CityObjectLabel.Roads: ColorRGBA(r=128.0/255.0, g=64.0/255.0, b=128.0/255.0, a=0.8),
    carla.CityObjectLabel.Sidewalks: ColorRGBA(r=244.0/255.0, g=35.0/255.0, b=232.0/255.0, a=0.8),
    carla.CityObjectLabel.Vegetation: ColorRGBA(r=107.0/255.0, g=142.0/255.0, b=35.0/255.0, a=0.8),
    #carla.CityObjectLabel.Vehicles: ColorRGBA(r=0.0/255.0, g=0.0/255.0, b=142.0/255.0, a=0.8),
    carla.CityObjectLabel.Walls: ColorRGBA(r=102.0/255.0, g=102.0/255.0, b=156.0/255.0, a=0.8),
    carla.CityObjectLabel.TrafficSigns: ColorRGBA(r=220.0/255.0, g=220.0/255.0, b=0.0/255.0, a=0.8),
    #carla.CityObjectLabel.Sky: ColorRGBA(r=70.0/255.0, g=130.0/255.0, b=180.0/255.0, a=0.8),
    #carla.CityObjectLabel.Ground: ColorRGBA(r=81.0/255.0, g=0.0/255.0, b=81.0/255.0, a=0.8),
    carla.CityObjectLabel.Bridge: ColorRGBA(r=150.0/255.0, g=100.0/255.0, b=100.0/255.0, a=0.8),
    carla.CityObjectLabel.RailTrack: ColorRGBA(r=230.0/255.0, g=150.0/255.0, b=140.0/255.0, a=0.8),
    carla.CityObjectLabel.GuardRail: ColorRGBA(r=180.0/255.0, g=165.0/255.0, b=180.0/255.0, a=0.8),
    carla.CityObjectLabel.TrafficLight: ColorRGBA(r=250.0/255.0, g=170.0/255.0, b=30.0/255.0, a=0.8),
    #carla.CityObjectLabel.Static: ColorRGBA(r=110.0/255.0, g=190.0/255.0, b=160.0/255.0, a=0.8),
    #carla.CityObjectLabel.Dynamic: ColorRGBA(r=170.0/255.0, g=120.0/255.0, b=50.0/255.0, a=0.8),
    #carla.CityObjectLabel.Water: ColorRGBA(r=45.0/255.0, g=60.0/255.0, b=150.0/255.0, a=0.8),
    #carla.CityObjectLabel.Terrain: ColorRGBA(r=145.0/255.0, g=170.0/255.0, b=100.0/255.0, a=0.8),
}


class MarkerSensor(PseudoActor):

    """
    Pseudo marker sensor
    """

    def __init__(self, uid, name, parent, node, actor_list, world):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(MarkerSensor, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)
        self.actor_list = actor_list
        self.world = world
        self.node = node

        self.marker_publisher = node.new_publisher(MarkerArray,
                                                   self.get_topic_prefix(),
                                                   qos_profile=10)
        self.static_marker_publisher = node.new_publisher(MarkerArray,
                                                   self.get_topic_prefix() + "/static",
                                                   qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # id generator for static objects.
        self.static_id_gen = itertools.count(1)

        static_markers = self._get_static_markers(OBJECT_LABELS.keys())
        if static_markers:
            self.static_marker_publisher.publish(static_markers)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.actor_list = None
        self.node.destroy_publisher(self.marker_publisher)
        self.node.destroy_publisher(self.static_marker_publisher)
        super(MarkerSensor, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.markers"

    def _get_marker_from_environment_object(self, environment_object):
        marker = Marker(header=self.get_msg_header(frame_id="map"))
        marker.ns = str(environment_object.type)
        marker.id = next(self.static_id_gen)

        box = environment_object.bounding_box
        marker.pose.position = carla_location_to_ros_point(box.location)
        marker.pose.orientation = carla_rotation_to_ros_quaternion(box.rotation)
        marker.scale.x = max(0.1, 2*box.extent.x)
        marker.scale.y = max(0.1, 2*box.extent.y)
        marker.scale.z = max(0.1, 2*box.extent.z)
        marker.type = Marker.CUBE

        marker.color = OBJECT_LABELS.get(environment_object.type, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8))
        return marker

    def _get_static_markers(self, object_types):
        static_markers = MarkerArray()
        for object_type in object_types:
            objects = self.world.get_environment_objects(object_type)
            for obj in objects:
                marker = self._get_marker_from_environment_object(obj)
                static_markers.markers.append(marker)

        return static_markers

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update map sends:
        - tf global frame
        :return:
        """
        marker_array_msg = MarkerArray()
        for actor in self.actor_list.values():
            if isinstance(actor, TrafficParticipant):
                marker_array_msg.markers.append(actor.get_marker(timestamp))
        self.marker_publisher.publish(marker_array_msg)
