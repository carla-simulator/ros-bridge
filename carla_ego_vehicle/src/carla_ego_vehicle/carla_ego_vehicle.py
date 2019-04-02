#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Carla Client to spawn the ego vehicle

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""
from __future__ import print_function

import sys
import glob
import os
import random
import math
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  # pylint: disable=wrong-import-position

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    """
    GNSS Sensor
    """

    def __init__(self, parent_actor):
        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(
            carla.Location(x=1.0, z=2.8)), attach_to=parent_actor)

    def __del__(self):
        if self.sensor.is_alive:
            self.sensor.destroy()
        self.sensor = None

# ==============================================================================
# -- LidarSensor ---------------------------------------------------------------
# ==============================================================================


class LidarSensor(object):
    """
    Lidar Sensor
    """

    def __init__(self, parent_actor):
        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('role_name', 'sensor')
        bp.set_attribute('range', '5000')
        bp.set_attribute('channels', '32')
        bp.set_attribute('points_per_second', '320000')
        bp.set_attribute('upper_fov', '2.0')
        bp.set_attribute('lower_fov', '-26.8')
        bp.set_attribute('rotation_frequency', '20')
        self.sensor = world.spawn_actor(bp, carla.Transform(
            carla.Location(x=0.0, z=2.4)), attach_to=parent_actor)

    def __del__(self):
        if self.sensor.is_alive:
            self.sensor.destroy()
        self.sensor = None

# ==============================================================================
# -- FrontCameraSensor ---------------------------------------------------------
# ==============================================================================


class FrontCamera(object):
    """
    Front camera for detections
    """

    def __init__(self, parent_actor):
        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('role_name', 'front')
        self.sensor = world.spawn_actor(bp, carla.Transform(
            carla.Location(x=2.0, z=2.0)), attach_to=parent_actor)

    def __del__(self):
        if self.sensor.is_alive:
            self.sensor.destroy()
        self.sensor = None

# ==============================================================================
# -- CameraSensor for Visualization --------------------------------------------
# ==============================================================================


class ViewCamera(object):
    """
    View camera for visualization purposes
    """

    def __init__(self, parent_actor):
        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('role_name', 'view')
        bp.set_attribute('image_size_x', '800')
        bp.set_attribute('image_size_y', '600')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(
            x=-5.5, z=2.8), carla.Rotation(pitch=-15)), attach_to=parent_actor)

    def __del__(self):
        if self.sensor.is_alive:
            self.sensor.destroy()
        self.sensor = None

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    """
    Collision sensor
    """

    def __init__(self, parent_actor):
        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent_actor)

    def __del__(self):
        if self.sensor.is_alive:
            self.sensor.destroy()
        self.sensor = None

# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    """
    Lane invasion sensor
    """

    def __init__(self, parent_actor):
        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent_actor)

    def __del__(self):
        if self.sensor.is_alive:
            self.sensor.destroy()
        self.sensor = None

# ==============================================================================
# -- EgoVehicle ---------------------------------------------------------------------
# ==============================================================================


class EgoVehicle(object):
    """
    Handles the spawning of the ego vehicle and its sensors
    """

    def __init__(self, carla_world, actor_filter):
        self.world = carla_world
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.lidar_sensor = None
        self.front_camera = None
        self.view_camera = None
        self.actor_filter = actor_filter
        self.actor_spawnpoint = None
        self.initialpose_subscriber = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.on_initialpose)
        self.restart()

    def on_initialpose(self, initial_pose):
        """
        Callback for /initialpose

        Receiving an initial pose (e.g. from RVIZ '2D Pose estimate') triggers a respawn.

        :return:
        """
        self.actor_spawnpoint = initial_pose.pose.pose
        self.restart()

    def restart(self):
        """
        (Re)spawns the vehicle

        Either at a given actor_spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Get vehicle blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self.actor_filter))
        blueprint.set_attribute('role_name', 'ego_vehicle')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the vehicle.
        if self.actor_spawnpoint:
            spawn_point = carla.Transform()
            spawn_point.location.x = self.actor_spawnpoint.position.x
            spawn_point.location.y = -self.actor_spawnpoint.position.y
            spawn_point.location.z = self.actor_spawnpoint.position.z + 2  # spawn 2m above ground
            quaternion = (
                self.actor_spawnpoint.orientation.x,
                self.actor_spawnpoint.orientation.y,
                self.actor_spawnpoint.orientation.z,
                self.actor_spawnpoint.orientation.w
            )
            _, _, yaw = euler_from_quaternion(quaternion)
            spawn_point.rotation.yaw = -math.degrees(yaw)
            rospy.loginfo("Spawn at x={} y={} z={} yaw={}".format(spawn_point.location.x,
                                                                  spawn_point.location.y,
                                                                  spawn_point.location.z,
                                                                  spawn_point.rotation.yaw))
            if self.player is not None:
                self.destroy()
            while self.player is None:
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        else:
            if self.player is not None:
                spawn_point = self.player.get_transform()
                spawn_point.location.z += 2.0
                spawn_point.rotation.roll = 0.0
                spawn_point.rotation.pitch = 0.0
                self.destroy()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            while self.player is None:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player)
        self.gnss_sensor = GnssSensor(self.player)
        self.lidar_sensor = LidarSensor(self.player)
        self.front_camera = FrontCamera(self.player)
        self.view_camera = ViewCamera(self.player)

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        del self.collision_sensor
        del self.lane_invasion_sensor
        del self.gnss_sensor
        del self.lidar_sensor
        del self.front_camera
        del self.view_camera
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.lidar_sensor = None
        self.front_camera = None
        self.view_camera = None
        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node('carla_client')

    host = rospy.get_param('/carla/host', '127.0.0.1')
    port = rospy.get_param('/carla/port', '2000')
    vehicle_filter = rospy.get_param('/carla/client/vehicle_filter', 'vehicle.*')

    rospy.loginfo('listening to server %s:%s', host, port)
    rospy.loginfo('using vehicle filter: %s', vehicle_filter)

    ego_vehicle = None
    try:
        client = carla.Client(host, port)
        client.set_timeout(2.0)

        ego_vehicle = EgoVehicle(client.get_world(), vehicle_filter)

        rospy.spin()

    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
