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

from abc import abstractmethod

import os
import sys
import random
import math
import json
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from carla_msgs.msg import CarlaWorldInfo

import carla

import carla_common.transforms as trans

from carla_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

secure_random = random.SystemRandom()

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class CarlaSpawnActors(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        rospy.init_node('spawn_actors_node', anonymous=True)
        self.host = rospy.get_param('carla/host', '127.0.0.1')
        self.port = rospy.get_param('carla/port', 2000)
        self.timeout = rospy.get_param('carla/timeout', 10)
        self.actors_definition_file = rospy.get_param('~actors_definition_file')
        self.world = None
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')

        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []
        self.ego_player = None

        self.initial_pose_ego_subscriber = rospy.Subscriber(
            "/carla/{}/initialpose".format(self.role_name),
            PoseWithCovarianceStamped,
            self.on_initialpose)
        rospy.loginfo('listening to server %s:%s', self.host, self.port)
        
        rospy.wait_for_service('/carla/spawn_object')
        self.spawn_object_service = rospy.ServiceProxy("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = rospy.ServiceProxy("/carla/destroy_object", DestroyObject)

    def on_initialpose(self, initial_pose):
        """
        Callback for /initialpose

        Receiving an initial pose (e.g. from RVIZ '2D Pose estimate') triggers a respawn of ego vehicle.

        :return:
        """
        if self.ego_player is None:
            rospy.loginfo("Cannot respawn ego ego vehicle, not created yet")
            return

        new_initial_ego_pose = initial_pose.pose.pose

        # Compute new spawn point
        spawn_point = carla.Transform()
        spawn_point.location.x = new_initial_ego_pose.x
        spawn_point.location.y = -new_initial_ego_pose.y
        spawn_point.location.z = new_initial_ego_pose.z + \
            2  # spawn 2m above ground
        quaternion = (
            new_initial_ego_pose.x,
            new_initial_ego_pose.y,
            new_initial_ego_pose.z,
            new_initial_ego_pose.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        spawn_point.rotation.yaw = -math.degrees(yaw)
        rospy.loginfo("Respawning ego vehicle at: x={} y={} z={} yaw={}".format(
                                                                spawn_point.location.x,
                                                                spawn_point.location.y,
                                                                spawn_point.location.z,
                                                                spawn_point.rotation.yaw))

        self.ego_player.set_transform(spawn_point)
        

    def spawn_actors(self):
        """
        Spawns the actors

        Either at a given actor_spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Read sensors from file
        if not os.path.exists(self.actors_definition_file):
            raise RuntimeError(
                "Could not read sensor-definition from {}".format(self.actors_definition_file))
        json_sensors = None
        with open(self.actors_definition_file) as handle:
            json_actors = json.loads(handle.read())

        global_sensors = []
        vehicles = []
        for actor in json_actors["actors"]:
            actor_type = actor["type"].split('.')[0]
            if actor_type == "sensor":
                global_sensors.append(actor)
            elif actor_type == "vehicle":
                vehicles.append(actor)
            else :
                rospy.logerr("Actor with type {} is not a vehicle nor a sensor".format(actor["type"]))
        
        try:
            self.global_sensors = self.setup_sensors(global_sensors)
        except Exception as e:
            rospy.logerr("Setting up gloabl sensors failed with error: {}".format(e))
        
        try:
            self.players = self.setup_vehicles(vehicles)
        except Exception as e:
            rospy.logerr("Setting up vehicles failed with error: {}".format(e))
        
    def setup_vehicles(self, vehicles):
        players = []
        for vehicle in vehicles:
            # Get vehicle blueprint.
            blueprint = secure_random.choice(
                self.world.get_blueprint_library().filter(vehicle["type"]))
            blueprint.set_attribute('role_name', "{}".format(vehicle["id"]))
            if blueprint.has_attribute('color'):
                color = secure_random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)

            player = None
            spawn_point = None

            if "spawn_point" in vehicle:
                spawn_point = carla.Transform()
                spawn_point.location.x = vehicle["spawn_point"]["x"]
                spawn_point.location.y = -vehicle["spawn_point"]["y"]
                spawn_point.location.z = vehicle["spawn_point"]["z"] + \
                    2  # spawn 2m above ground
                spawn_point.rotation.yaw = -vehicle["spawn_point"]["yaw"]
                rospy.loginfo("Spawn {} at x={} y={} z={} yaw={}".format(vehicle["id"],
                                                                        spawn_point.location.x,
                                                                        spawn_point.location.y,
                                                                        spawn_point.location.z,
                                                                        spawn_point.rotation.yaw))
            else:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = secure_random.choice(spawn_points) if spawn_points else carla.Transform()


            spawn_object_request = SpawnObjectRequest()
            spawn_object_request.type = blueprint.id
            spawn_object_request.id = vehicle["id"]
            spawn_object_request.attach_to = 0

            while player is None:
                spawn_object_request.transform = trans.carla_transform_to_ros_pose(spawn_point)
                response = self.spawn_object_service(spawn_object_request)
                if response.id != -1:
                    player = self.world.get_actor(response.id)
                    players.append(player)

                    if vehicle["id"] == self.role_name:
                        self.ego_player = player

                    # Set up the sensors
                    self.vehicles_sensors.append(self.setup_sensors(vehicle["sensors"], player.id))

        return players

    def setup_sensors(self, sensors, attached_vehicle_id=0):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :return:
        """
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                sensor_id = str(sensor_spec.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    rospy.logfatal(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                    raise NameError(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                sensor_names.append(sensor_name)

                sensor_location = carla.Location(x=sensor_spec.pop("x", 0.0),
                                                 y=sensor_spec.pop("y", 0.0),
                                                 z=sensor_spec.pop("z", 0.0))
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec.pop('pitch', 0.0),
                    roll=sensor_spec.pop('roll', 0.0),
                    yaw=sensor_spec.pop('yaw', 0.0))
                sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                spawn_object_request = SpawnObjectRequest()
                spawn_object_request.type = sensor_type
                spawn_object_request.id = sensor_id
                spawn_object_request.attach_to = attached_vehicle_id
                spawn_object_request.transform = trans.carla_transform_to_ros_pose(
                    sensor_transform)
                for attribute, value in sensor_spec.items():
                    spawn_object_request.attributes.append(
                        KeyValue(str(attribute), str(value)))

                response = self.spawn_object_service(spawn_object_request)
                if response.id == -1:
                    raise Exception(response.error_string)

            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except Exception as e:
                rospy.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

            actors.append(response.id)
        return actors

    @abstractmethod
    def sensors(self):
        """
        return a list of sensors attached
        """
        return []

    def destroy(self):
        """
        destroy all the players and sensors
        """
        # destroy global sensors
        for actor_id in self.global_sensors:
            destroy_object_request = DestroyObjectRequest(actor_id)
            try:
                response = self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once(str(e))
        self.global_sensors = []

        # destroy vehicles sensors
        for vehicle_sensors_id in self.vehicles_sensors:
            for actor_id in vehicle_sensors_id:
                destroy_object_request = DestroyObjectRequest(actor_id)
                try:
                    response = self.destroy_object_service(destroy_object_request)
                except rospy.ServiceException as e:
                    rospy.logwarn_once(str(e))
        self.vehicles_sensors = []

        # destroy player
        for player in self.players:
            if player and player.is_alive:
                destroy_object_request = DestroyObjectRequest(player.id)
                try:
                    self.destroy_object_service(destroy_object_request)
                except rospy.ServiceException as e:
                    rospy.logwarn_once(str(e))
        self.players = []


    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
            sys.exit(1)

        rospy.loginfo("CARLA world available. Spawn ego vehicle...")
        rospy.on_shutdown(self.destroy)
        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()
        self.spawn_actors()
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
    spawn_actors_node = CarlaSpawnActors()
    try:
        spawn_actors_node.run()
    finally:
        if spawn_actors_node is not None:
            spawn_actors_node.destroy()


if __name__ == '__main__':
    main()
