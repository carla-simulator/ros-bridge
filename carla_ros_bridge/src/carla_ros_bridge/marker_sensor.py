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

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic_participant import TrafficParticipant

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from carla import CityObjectLabel
from carla_common.transforms import carla_location_to_ros_point, carla_rotation_to_ros_quaternion
from ros_compatibility import QoSProfile, latch_on


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
                                                   self.get_topic_prefix())
        self.static_marker_publisher = node.new_publisher(MarkerArray,
                                                   'carla/static_markers',
                                                   qos_profile=QoSProfile(depth=1, durability=latch_on))
        static_markers = self.get_map_markers([
            CityObjectLabel.Fences,
            CityObjectLabel.Buildings,
            CityObjectLabel.TrafficSigns,
            CityObjectLabel.Vegetation,
            CityObjectLabel.Walls,
            CityObjectLabel.GuardRail,
            CityObjectLabel.TrafficLight])
        if static_markers:
            self.static_marker_publisher.publish(static_markers)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        self.actor_list = None
        self.node.destroy_publisher(self.marker_publisher)
        super(MarkerSensor, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.markers"

    def marker_from_carla_object(self, carla_object, object_type=None):
        marker = Marker(header=self.get_msg_header(frame_id="map"))
        box = carla_object.bounding_box
        marker.pose.position = carla_location_to_ros_point(box.location)
        marker.pose.orientation = carla_rotation_to_ros_quaternion(box.rotation)
        marker.scale.x = 2*box.extent.x
        marker.scale.y = 2*box.extent.y
        marker.scale.z = 2*box.extent.z
        marker.type = Marker.CUBE

        color = ColorRGBA()
        color.a = 0.8
        if object_type is CityObjectLabel.TrafficSigns:
            color.r = 255.0
            color.g = 0.0
            color.b = 0.0
        elif object_type is CityObjectLabel.TrafficLight:
            color.r = 255.0
            color.g = 128.0
            color.b = 0.0
        elif object_type is CityObjectLabel.Buildings:
            color.r = 0.0
            color.g = 0.0
            color.b = 255.0
        elif object_type is CityObjectLabel.Fences:
            color.r = 125.0
            color.g = 0.0
            color.b = 255.0
        elif object_type is CityObjectLabel.Walls:
            color.r = 0.0
            color.g = 150.0
            color.b = 150.0
        elif object_type is CityObjectLabel.Vegetation:
            if marker.scale.x * marker.scale.y * marker.scale.z > 350:
                return None
            color.r = 0.0
            color.g = 255.0
            color.b = 0.0
        else:
            color.r = 255.0
            color.g = 255.0
            color.b = 0.0
        
        marker.color = color
        return marker


    def get_map_markers(self, object_types):
        static_objects = {}
        for object_type in object_types:
            objects = self.world.get_environment_objects(object_type)
            for carla_object in objects:
                marker = self.marker_from_carla_object(carla_object, object_type)
                if marker is None:
                    continue
                static_objects[carla_object.id] = marker
        object_markers = MarkerArray()
        count = 0
        sorted_ids = sorted(static_objects)
        marker_ids = []
        for key in sorted_ids:
            if key < 2147483647:
                marker = static_objects[key]
                marker.id = key
                object_markers.markers.append(marker)
                marker_ids.append(key)
            elif (2147483647-count) not in marker_ids:
                marker = static_objects[key]
                marker.id = 2147483647-count
                object_markers.markers.append(marker)
                marker_ids.append(2147483647-count)
                count+=1
            else:
                self.node.logwarn("Could not create a Marker of object with id {},".format(key)
                    + "id is too high to use directly (>2147483647) and could not find a replacement")
        return object_markers

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
                marker_array_msg.markers.append(actor.get_marker())
        self.marker_publisher.publish(marker_array_msg)
