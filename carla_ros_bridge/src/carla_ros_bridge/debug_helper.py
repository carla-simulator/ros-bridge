#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to draw marker
"""
import math
import os

import carla
from transforms3d.euler import quat2euler

from visualization_msgs.msg import Marker, MarkerArray  # pylint: disable=import-error


class DebugHelper(object):
    """
    Helper to draw markers in CARLA
    """

    def __init__(self, carla_debug_helper, node):
        """
        Constructor

        :param carla_debug_helper: carla debug helper
        :type carla_debug_helper: carla.DebugHelper
        """
        self.debug = carla_debug_helper
        self.node = node
        self.marker_subscriber = self.node.new_subscription(MarkerArray, "/carla/debug_marker",
                                                            self.on_marker, qos_profile=10)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions

        :return:
        """
        self.node.logdebug("Destroy DebugHelper")
        self.debug = None
        self.node.destroy_subscription(self.marker_subscriber)

    def on_marker(self, marker_array):
        """
        Receive markers from ROS and apply in CARLA
        """
        for marker in marker_array.markers:
            if marker.header.frame_id != "map":
                self.node.logwarn("Could not draw marker in frame '{}'. Only 'map' supported.".format(
                    marker.header.frame_id))
                continue
            lifetime = -1.
            if marker.lifetime:
                lifetime = marker.lifetime.to_sec()
            color = carla.Color(int(marker.color.r * 255), int(marker.color.g * 255),
                                int(marker.color.b * 255), int(marker.color.a * 255))

            if marker.type == Marker.POINTS:
                self.draw_points(marker, lifetime, color)
            elif marker.type == Marker.LINE_STRIP:
                self.draw_line_strips(marker, lifetime, color)
            elif marker.type == Marker.ARROW:
                self.draw_arrow(marker, lifetime, color)
            elif marker.type == Marker.CUBE:
                self.draw_box(marker, lifetime, color)
            else:
                self.node.logwarn(
                    "Marker type '{}' not supported.".format(marker.type))

    def draw_arrow(self, marker, lifetime, color):
        """
        draw arrow from ros marker
        """
        if marker.points:
            if not len(marker.points) == 2:
                self.node.logwarn("Drawing arrow from points requires two points. Received {}".format(
                    len(marker.points)))
                return
            thickness = marker.scale.x
            arrow_size = marker.scale.y
            start = carla.Location(x=marker.points[0].x, y=-marker.points[0].y,
                                   z=marker.points[0].z)
            end = carla.Location(x=marker.points[1].x, y=-marker.points[1].y, z=marker.points[1].z)
            self.node.loginfo("Draw Arrow from {} to {} (color: {}, lifetime: {}, "
                              "thickness: {}, arrow_size: {})".format(start, end, color, lifetime,
                                                                      thickness, arrow_size))
            self.debug.draw_arrow(start, end, thickness=thickness, arrow_size=arrow_size,
                                  color=color, life_time=lifetime)

        else:
            self.node.logwarn("Drawing arrow from Position/Orientation not yet supported. "
                              "Please use points.")

    def draw_points(self, marker, lifetime, color):
        """
        draw points from ros marker
        """
        for point in marker.points:
            location = carla.Location(x=point.x, y=-point.y, z=point.z)
            size = marker.scale.x
            self.node.loginfo("Draw Point {} (color: {}, lifetime: {}, size: {})".format(
                location, color, lifetime, size))
            self.debug.draw_point(location, size=size, color=color, life_time=lifetime)

    def draw_line_strips(self, marker, lifetime, color):
        """
        draw lines from ros marker
        """
        if len(marker.points) < 2:
            self.node.logwarn("Drawing line-strip requires at least two points. Received {}".format(
                len(marker.points)))
            return
        last_point = None
        thickness = marker.scale.x
        for point in marker.points:
            if last_point:
                start = carla.Location(x=last_point.x, y=-last_point.y, z=last_point.z)
                end = carla.Location(x=point.x, y=-point.y, z=point.z)
                self.node.loginfo("Draw Line from {} to {} (color: {}, lifetime: {}, "
                                  "thickness: {})".format(start, end, color, lifetime, thickness))
                self.debug.draw_line(start, end, thickness=thickness, color=color,
                                     life_time=lifetime)
            last_point = point

    def draw_box(self, marker, lifetime, color):
        """
        draw box from ros marker
        """
        box = carla.BoundingBox()
        box.location.x = marker.pose.position.x
        box.location.y = -marker.pose.position.y
        box.location.z = marker.pose.position.z
        box.extent.x = marker.scale.x / 2
        box.extent.y = marker.scale.y / 2
        box.extent.z = marker.scale.z / 2

        roll, pitch, yaw = quat2euler([
            marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z

        ])
        rotation = carla.Rotation()
        rotation.roll = math.degrees(roll)
        rotation.pitch = math.degrees(pitch)
        rotation.yaw = -math.degrees(yaw)
        self.node.loginfo("Draw Box {} (rotation: {}, color: {}, lifetime: {})".format(
            box, rotation, color, lifetime))
        self.debug.draw_box(box, rotation, thickness=0.1, color=color, life_time=lifetime)
