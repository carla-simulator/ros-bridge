#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning a Ego Vehicle in ROS

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""
from __future__ import print_function

from abc import abstractmethod

import os
import random
import math
import json
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

import carla

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class CarlaEgoVehicle(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        rospy.init_node('ego_vehicle', anonymous=True)
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.sensor_definition_file = rospy.get_param('~sensor_definition_file')
        self.world = None
        self.player = None
        self.sensor_actors = []
        self.actor_filter = rospy.get_param('~vehicle_filter', 'vehicle.*')
        self.actor_spawnpoint = None
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        # check argument and set spawn_point
        spawn_point_param = rospy.get_param('~spawn_point')
        if spawn_point_param:
            print("Using ros parameter for spawnpoint: {}".format(spawn_point_param))
            spawn_point = spawn_point_param.split(',')
            if len(spawn_point) != 6:
                raise ValueError("Invalid spawnpoint '{}'".format(spawn_point_param))
            pose = Pose()
            pose.position.x = float(spawn_point[0])
            pose.position.y = -float(spawn_point[1])
            pose.position.z = float(spawn_point[2])
            quat = quaternion_from_euler(
                math.radians(float(spawn_point[3])),
                math.radians(float(spawn_point[4])),
                math.radians(float(spawn_point[5])))
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.actor_spawnpoint = pose

        self.initialpose_subscriber = rospy.Subscriber(
            "/carla/{}/initialpose".format(self.role_name),
            PoseWithCovarianceStamped,
            self.on_initialpose)
        rospy.loginfo('listening to server %s:%s', self.host, self.port)
        rospy.loginfo('using vehicle filter: %s', self.actor_filter)

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
        blueprint.set_attribute('role_name', "{}".format(self.role_name))
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
            rospy.loginfo("Spawn {} at x={} y={} z={} yaw={}".format(self.role_name,
                                                                     spawn_point.location.x,
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

        # Read sensors from file
        if not os.path.exists(self.sensor_definition_file):
            raise RuntimeError(
                "Could not read sensor-definition from {}".format(self.sensor_definition_file))
        json_sensors = None
        with open(self.sensor_definition_file) as handle:
            json_sensors = json.loads(handle.read())

        # Set up the sensors
        self.sensor_actors = self.setup_sensors(json_sensors["sensors"])

    def setup_sensors(self, sensors):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :return:
        """
        actors = []
        bp_library = self.world.get_blueprint_library()
        for sensor_spec in sensors:
            try:
                bp = bp_library.find(str(sensor_spec['type']))
                bp.set_attribute('role_name', str(sensor_spec['id']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    try:
                        bp.set_attribute('sensor_tick', str(sensor_spec['sensor_tick']))
                    except KeyError:
                        pass
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    bp.set_attribute('range', str(sensor_spec['range']))
                    bp.set_attribute('rotation_frequency', str(sensor_spec['rotation_frequency']))
                    bp.set_attribute('channels', str(sensor_spec['channels']))
                    bp.set_attribute('upper_fov', str(sensor_spec['upper_fov']))
                    bp.set_attribute('lower_fov', str(sensor_spec['lower_fov']))
                    bp.set_attribute('points_per_second', str(sensor_spec['points_per_second']))
                    try:
                        bp.set_attribute('sensor_tick', str(sensor_spec['sensor_tick']))
                    except KeyError:
                        pass
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, because sensor spec is invalid: '{}'".format(e))
                continue

            # create sensor
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = self.world.spawn_actor(bp, sensor_transform,
                                            attach_to=self.player)
            actors.append(sensor)
        return actors

    @abstractmethod
    def sensors(self):
        """
        return a list of sensors attached
        """
        return []

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        for i, _ in enumerate(self.sensor_actors):
            if self.sensor_actors[i] is not None:
                self.sensor_actors[i].destroy()
                self.sensor_actors[i] = None
        self.sensor_actors = []

        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None

    def run(self):
        """
        main loop
        """
        client = carla.Client(self.host, self.port)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.restart()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    ego_vehicle = CarlaEgoVehicle()
    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    main()
