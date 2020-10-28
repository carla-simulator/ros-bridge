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
import json
import math
import os
import random
import traceback
from abc import abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose  # pylint: disable=import-error
from carla_msgs.msg import CarlaWorldInfo  # pylint: disable=import-error
from ros_compatibility import (
    CompatibleNode,
    destroy_subscription,
    euler_from_quaternion,
    quaternion_from_euler,
    QoSProfile
)

import carla

ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 2:
    import rclpy  # pylint: disable=import-error
    from ament_index_python.packages import get_package_share_directory  # pylint: disable=import-error


secure_random = random.SystemRandom()

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class CarlaEgoVehicle(CompatibleNode):
    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        super(CarlaEgoVehicle, self).__init__('ego_vehicle_node')
        self.world = None
        self.player = None
        self.player_created = False
        self.sensor_actors = []
        self.actor_spawnpoint = None
        self.timeout = self.get_param('carla/timeout', 2)
        self.host = self.get_param('carla/host', 'localhost')
        self.port = self.get_param('carla/port', 2000)
        self.actor_filter = self.get_param('vehicle_filter', 'vehicle.*')
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        # check argument and set spawn_point
        spawn_point_param = self.get_param('spawn_point')

        if ROS_VERSION == 1:
            self.sensor_definition_file = self.get_param('sensor_definition_file', 'sensors.json')
        elif ROS_VERSION == 2:
            sensor_path = get_package_share_directory('carla_ego_vehicle') + '/config/sensors.json'
            self.sensor_definition_file = self.get_param('sensor_definition_file', sensor_path)
        if spawn_point_param and self.spawn_ego_vehicle():
            self.loginfo("Using ros parameter for spawnpoint: {}".format(spawn_point_param))
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

        self.initialpose_subscriber = self.create_subscriber(PoseWithCovarianceStamped,
                                                             "/carla/{}/initialpose".format(
                                                                 self.role_name),
                                                             self.on_initialpose)

        self.loginfo("listening to server {}:{}".format(self.host, self.port))
        self.loginfo("using vehicle filter: {}".format(self.actor_filter))

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        self.startup_subscription = self.create_subscriber(
            CarlaWorldInfo,
            '/carla/world_info',
            self.run,
            QoSProfile(depth=1, durability=False)
        )

    # pylint: disable=inconsistent-return-statements
    def spawn_ego_vehicle(self):
        """
        Helper method for condition-checking in self.restart().
        """
        if ROS_VERSION == 1:
            return self.get_param('~spawn_ego_vehicle')
        elif ROS_VERSION == 2:
            return self.get_parameter('spawn_ego_vehicle')

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
        blueprint = secure_random.choice(
            self.world.get_blueprint_library().filter(self.actor_filter))
        blueprint.set_attribute('role_name', "{}".format(self.role_name))
        if blueprint.has_attribute('color'):
            color = secure_random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the vehicle.
        if not self.spawn_ego_vehicle():
            actors = self.world.get_actors().filter(self.actor_filter)
            for actor in actors:
                if actor.attributes['role_name'] == self.role_name:
                    self.player = actor
                    break
        else:
            if self.actor_spawnpoint:
                spawn_point = carla.Transform()
                spawn_point.location.x = self.actor_spawnpoint.position.x
                spawn_point.location.y = -self.actor_spawnpoint.position.y
                spawn_point.location.z = self.actor_spawnpoint.position.z + \
                    2  # spawn 2m above ground
                quaternion = [
                    self.actor_spawnpoint.orientation.x,
                    self.actor_spawnpoint.orientation.y,
                    self.actor_spawnpoint.orientation.z,
                    self.actor_spawnpoint.orientation.w
                ]
                _, _, yaw = euler_from_quaternion(quaternion)
                spawn_point.rotation.yaw = -math.degrees(yaw)
                self.loginfo("Spawn {} at x={} y={} z={} yaw={}".format(self.role_name,
                                                                        spawn_point.location.x,
                                                                        spawn_point.location.y,
                                                                        spawn_point.location.z,
                                                                        spawn_point.rotation.yaw))
                if self.player is not None:
                    self.player.set_transform(spawn_point)
                while self.player is None:
                    self.player = self.world.try_spawn_actor(blueprint, spawn_point)
                    self.player_created = True

            else:
                if self.player is not None:
                    spawn_point = self.player.get_transform()
                    spawn_point.location.z += 2.0
                    spawn_point.rotation.roll = 0.0
                    spawn_point.rotation.pitch = 0.0
                    self.player.set_transform(spawn_point)
                while self.player is None:
                    spawn_points = self.world.get_map().get_spawn_points()
                    spawn_point = secure_random.choice(
                        spawn_points) if spawn_points else carla.Transform()
                    self.player = self.world.try_spawn_actor(blueprint, spawn_point)
                    self.player_created = True

        if self.player_created:
            # Read sensors from file
            try:
                with open(self.sensor_definition_file) as f:
                    json_sensors = json.load(f)["sensors"]
            except (OSError, json.JSONDecodeError, KeyError) as e:
                raise RuntimeError(
                    "Could not read sensor-definition from '{}' error is: {}".format(self.sensor_definition_file, e))

            # Set up the sensors
            self.sensor_actors = self.setup_sensors(json_sensors)

            self.player_created = False

    def setup_sensors(self, sensors):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :return:
        """
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_name = str(sensor_spec['type']) + "/" + str(sensor_spec['id'])
                if sensor_name in sensor_names:
                    self.logfatal(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                    raise NameError(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                sensor_names.append(sensor_name)

                sensor_location = carla.Location(x=sensor_spec.pop("x"),
                                                 y=sensor_spec.pop("y"),
                                                 z=sensor_spec.pop("z"))
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec.pop('pitch', 0.0),
                    roll=sensor_spec.pop('roll', 0.0),
                    yaw=sensor_spec.pop('yaw', 0.0))
                sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                bp = bp_library.find(sensor_type)
                bp.set_attribute('role_name', sensor_id)
                for attribute, value in sensor_spec.items():
                    bp.set_attribute(str(attribute), str(value))

            except KeyError as e:
                self.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except IndexError as e:
                self.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

            # create sensor
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
        super(CarlaEgoVehicle, self).destroy()

    def run(self, _msg):
        """
        Called after the Carla world was loaded; spawns the ego vehicle.
        """
        self.loginfo("CARLA world available. Spawn ego vehicle...")
        # The subscription should only call this function once
        destroy_subscription(self.startup_subscription)

        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(self.timeout)
            self.world = client.get_world()
            self.restart()
        except Exception:  # pylint: disable=broad-except
            self.logerr("Could not run node: {}".format(traceback.format_exc()))
            self.shutdown()
            return

        self.loginfo("Ego spawned.")


def main():
    """
    main function
    """
    executor = None
    if ROS_VERSION == 2:
        rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor()

    ego_vehicle = CarlaEgoVehicle()
    ego_vehicle.spin(executor)


if __name__ == '__main__':
    main()
