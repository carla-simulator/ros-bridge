#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a bounding box sensor
"""

import carla
import rospy
import string

import carla_common.transforms as trans

from visualization_msgs.msg import Marker, MarkerArray

from carla_ros_bridge.pseudo_actor import PseudoActor

LABELS = [
    carla.CityObjectLabel.Buildings,
    carla.CityObjectLabel.Fences,
    carla.CityObjectLabel.Other,
    carla.CityObjectLabel.Pedestrians,
    carla.CityObjectLabel.Poles,
    carla.CityObjectLabel.RoadLines,
    carla.CityObjectLabel.Roads,
    carla.CityObjectLabel.Sidewalks,
    carla.CityObjectLabel.Vegetation,
    carla.CityObjectLabel.Vehicles,
    carla.CityObjectLabel.Walls,
    carla.CityObjectLabel.TrafficSigns,
    carla.CityObjectLabel.Sky,
    carla.CityObjectLabel.Ground,
    carla.CityObjectLabel.Bridge,
    carla.CityObjectLabel.RailTrack,
    carla.CityObjectLabel.GuardRail,
    carla.CityObjectLabel.TrafficLight,
    carla.CityObjectLabel.Static,
    carla.CityObjectLabel.Dynamic,
    carla.CityObjectLabel.Water,
    carla.CityObjectLabel.Terrain
]

COLORS = [
    (70, 70, 70),    # Buildings
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrians
    (153, 153, 153), # Poles
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Roads
    (244, 35, 232),  # Sidewalks
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicles
    (102, 102, 156), # Walls
    (220, 220, 0),   # TrafficSigns
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]


class BoundingBoxSensor(PseudoActor):

    """
    Pseudo bounding box sensor
    """

    def __init__(self, parent, node):
        """
        Constructor
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(BoundingBoxSensor, self).__init__(parent=parent,
                                           node=node,
                                           prefix='bounding_boxes')

        self._published = False
        self._publishers = {}

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(BoundingBoxSensor, self).destroy()

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update map sends:
        - tf global frame
        :return:
        """
        if not self._published:
            header = self.get_msg_header("map")

            for i, label in enumerate(LABELS):
                if str(label) not in self._publishers:
                    self._publishers[str(label)] = rospy.Publisher(
                        self.get_topic_prefix() + "/" + string.lower(str(label)),
                        MarkerArray,
                        queue_size=10,
                        latch=True)

                bbs = self.node.carla_world.get_level_bbs(label)
                color = COLORS[i]

                marker_array_msg = MarkerArray()
                for j, bbox in enumerate(bbs):
                    marker_msg = Marker(header=header)
                    marker_msg.ns = str(label)
                    marker_msg.id = j
                    marker_msg.type = Marker.CUBE
                    marker_msg.pose = trans.carla_location_to_pose(bbox.location)
                    marker_msg.pose.orientation.w = 1.0
                    marker_msg.scale.x = max(0.1, bbox.extent.x * 2.0)
                    marker_msg.scale.y = max(0.1, bbox.extent.y * 2.0)
                    marker_msg.scale.z = max(0.1, bbox.extent.z * 2.0)

                    marker_msg.color.r = color[1] / 255.0
                    marker_msg.color.g = color[0] / 255.0
                    marker_msg.color.b = color[2] / 255.0
                    marker_msg.color.a = 0.5

                    marker_array_msg.markers.append(marker_msg)

                self._publishers[str(label)].publish(marker_array_msg)

            self._published = True
