#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Spawns a camera, attached to an ego vehicle.

The pose of the camera can be changed by publishing
to /carla/<ROLENAME>/spectator_position.
"""
# pylint: disable=import-error
import sys
import math
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaWorldInfo

from ros_compatibility import (
    CompatibleNode,
    euler_from_quaternion,
    QoSProfile,
    ROSException,
    ROSInterruptException
)
import os
ROS_VERSION = int(os.environ['ROS_VERSION'])
if ROS_VERSION == 2:
    import rclpy

import carla

# ==============================================================================
# -- CarlaSpectatorCamera ------------------------------------------------------------
# ==============================================================================


class CarlaSpectatorCamera(object):
    """
    Spawns a camera, attached to an ego vehicle.

    The pose of the camera can be changed by publishing
    to /carla/<ROLENAME>/spectator_position.
    """

    def __init__(self):
        """
        Constructor
        """
        self._node = CompatibleNode('spectator_camera')
        self.role_name = self._node.get_param("role_name", "ego_vehicle")
        self.camera_resolution_x = self._node.get_param("resolution_x", 800)
        self.camera_resolution_y = self._node.get_param("resolution_y", 600)
        self.camera_fov = self._node.get_param("fov", 50)
        self.host = self._node.get_param('carla/host', '127.0.0.1')
        self.port = self._node.get_param('carla/port', 2000)
        self.timeout = self._node.get_param("carla/timeout", 10)
        self.world = None
        self.pose = None
        self.camera_actor = None
        self.ego_vehicle = None
        self._node.create_subscriber(PoseStamped,
                                     "/carla/{}/spectator_pose".format(self.role_name), self.pose_received)

    def pose_received(self, pose):
        """
        Move the camera
        """
        if self.pose != pose:
            self._node.logdebug("Camera pose changed. Updating carla camera")
            self.pose = pose
            transform = self.get_camera_transform()
            if transform and self.camera_actor:
                self.camera_actor.set_transform(transform)

    def get_camera_transform(self):
        """
        Calculate the CARLA camera transform
        """
        if not self.pose:
            self._node.loginfo("no pose!")
            return None
        if self.pose.header.frame_id != self.role_name:
            self._node.logwarn("Unsupported frame received. Supported {}, received {}".format(
                self.role_name, self.pose.header.frame_id))
            return None
        sensor_location = carla.Location(x=self.pose.pose.position.x,
                                         y=-self.pose.pose.position.y,
                                         z=self.pose.pose.position.z)
        quaternion = (
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # rotate to CARLA
        sensor_rotation = carla.Rotation(pitch=math.degrees(roll)-90,
                                         roll=math.degrees(pitch),
                                         yaw=-math.degrees(yaw)-90)
        return carla.Transform(sensor_location, sensor_rotation)

    def create_camera(self, ego_actor):
        """
        Attach the camera to the ego vehicle
        """
        bp_library = self.world.get_blueprint_library()
        try:
            bp = bp_library.find("sensor.camera.rgb")
            bp.set_attribute('role_name', "spectator_view")
            self._node.loginfo("Creating camera with resolution {}x{}, fov {}".format(
                self.camera_resolution_x,
                self.camera_resolution_y,
                self.camera_fov))
            bp.set_attribute('image_size_x', str(self.camera_resolution_x))
            bp.set_attribute('image_size_y', str(self.camera_resolution_y))
            bp.set_attribute('fov', str(self.camera_fov))
        except KeyError as e:
            self._node.logfatal("Camera could not be spawned: '{}'".format(e))
        transform = self.get_camera_transform()
        if not transform:
            transform = carla.Transform()
        self.camera_actor = self.world.spawn_actor(bp, transform,
                                                   attach_to=ego_actor)

    def find_ego_vehicle_actor(self):
        """
        Look for an carla actor with role name
        """
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break
        ego_vehicle_changed = False
        if hero is None and self.ego_vehicle is not None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and \
                self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
            ego_vehicle_changed = True

        if ego_vehicle_changed:
            self.destroy()
            self._node.loginfo("Ego vehicle changed.")
            self.ego_vehicle = hero
            if self.ego_vehicle:
                self.create_camera(self.ego_vehicle)

    def destroy(self):
        """
        destroy the camera
        """
        if self.camera_actor:
            self.camera_actor.destroy()
            self.camera_actor = None

    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        self._node.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self._node.wait_for_one_message("/carla/world_info", CarlaWorldInfo, timeout=10.0,
                                            qos_profile=QoSProfile(depth=10, durability=True))
        except ROSException:
            self._node.logerr("Error while waiting for world info!")
            sys.exit(1)
        self._node.loginfo("CARLA world available. Waiting for ego vehicle...")

        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()

        period = 0.1
        try:
            self._node.new_timer(period, lambda timer_event=None: self.find_ego_vehicle_actor())
            self._node.spin()
        except ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    if ROS_VERSION == 2:
        rclpy.init()

    carla_spectator_camera = CarlaSpectatorCamera()
    try:
        carla_spectator_camera.run()
    finally:
        if carla_spectator_camera is not None:
            carla_spectator_camera.destroy()


if __name__ == '__main__':
    main()
