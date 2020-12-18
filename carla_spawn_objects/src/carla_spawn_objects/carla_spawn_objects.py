#!/usr/bin/env python
#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning objects (carla actors and pseudo_actors) in ROS

Gets config file from ros parameter ~objects_definition_file and spawns corresponding objects
through ROS service /carla/spawn_object.

Looks for an initial spawn point first in the launchfile, then in the config file, and 
finally ask for a random one to the spawn service.

"""

import os
import math
import json

from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
from carla_msgs.msg import CarlaActorList


from carla_msgs.srv import SpawnObject, DestroyObject

from ros_compatibility import (
    CompatibleNode,
    ROSInterruptException,
    ServiceException,
    quaternion_from_euler,
    ros_init
)

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    from carla_msgs.srv import SpawnObjectRequest, DestroyObjectRequest

# ==============================================================================
# -- CarlaSpawnObjects ------------------------------------------------------------
# ==============================================================================


class CarlaSpawnObjects(CompatibleNode):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        super(CarlaSpawnObjects, self).__init__('carla_spawn_objects')
        self.objects_definition_file = self.get_param('objects_definition_file')
        self.spawn_sensors_only = self.get_param('spawn_sensors_only', None)

        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []

        self.spawn_object_service = self.create_service_client("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = self.create_service_client(
            "/carla/destroy_object", DestroyObject)

    def spawn_objects(self):
        """
        Spawns the objects

        Either at a given spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Read sensors from file
        if not self.objects_definition_file or not os.path.exists(self.objects_definition_file):
            raise RuntimeError(
                "Could not read object definitions from {}".format(self.objects_definition_file))
        with open(self.objects_definition_file) as handle:
            json_actors = json.loads(handle.read())

        global_sensors = []
        vehicles = []
        found_sensor_actor_list = False

        for actor in json_actors["objects"]:
            actor_type = actor["type"].split('.')[0]
            if actor["type"] == "sensor.pseudo.actor_list" and self.spawn_sensors_only:
                global_sensors.append(actor)
                found_sensor_actor_list = True
            elif actor_type == "sensor":
                global_sensors.append(actor)
            elif actor_type == "vehicle" or actor_type == "walker":
                vehicles.append(actor)
            else:
                self.logwarn(
                    "Object with type {} is not a vehicle, a walker or a sensor, ignoring".format(actor["type"]))
        if self.spawn_sensors_only is True and found_sensor_actor_list is False:
            raise Exception("Parameter 'spawn_sensors_only' enabled, " +
                            "but 'sensor.pseudo.actor_list' is not instantiated, add it to your config file.")

        try:
            self.global_sensors = self.setup_sensors(global_sensors)
        except Exception as e:
            raise Exception("Setting up global sensors failed: {}".format(e))

        if self.spawn_sensors_only is True:
            # get vehicle id from topic /carla/actor_list for all vehicles listed in config file
            actor_info_list = self.wait_for_one_message("/carla/actor_list", CarlaActorList)
            for vehicle in vehicles:
                for actor_info in actor_info_list.actors:
                    if actor_info.type == vehicle["type"] and actor_info.rolename == vehicle["id"]:
                        vehicle["carla_id"] = actor_info.id

        try:
            self.players = self.setup_vehicles(vehicles)
        except Exception as e:
            raise Exception("Setting up vehicles failed: {}".format(e))

    def setup_vehicles(self, vehicles):
        players = []
        for vehicle in vehicles:
            if self.spawn_sensors_only is True:
                # spawn sensors of already spawned vehicles
                try:
                    carla_id = vehicle["carla_id"]
                except KeyError as e:
                    self.logerr(
                        "Could not spawn sensors of vehicle {}, its carla ID is not known.".format(vehicle["id"]))
                    break
                # spawn the vehicle's sensors
                try:
                    self.vehicles_sensors.append(self.setup_sensors(vehicle["sensors"], carla_id))
                except Exception as e:
                    raise Exception(
                        "Setting up sensors of already spawned vehicle {} failed with error: {}".format(vehicle["id"], e))
            else:
                if ROS_VERSION == 1:
                    spawn_object_request = SpawnObjectRequest()
                elif ROS_VERSION == 2:
                    spawn_object_request = SpawnObject.Request()
                spawn_object_request.type = vehicle["type"]
                spawn_object_request.id = vehicle["id"]
                spawn_object_request.attach_to = 0
                spawn_object_request.random_pose = False

                spawn_point = None

                # check if there's a spawn_point corresponding to this vehicle
                spawn_point_param = self.get_param("~spawn_point_" + vehicle["id"], None)
                spawn_param_used = False
                if (spawn_point_param is not None):
                    # try to use spawn_point from parameters
                    try:
                        spawn_point = self.check_spawn_point_param(spawn_point_param)
                        self.loginfo("Spawn point from ros parameters")
                        spawn_param_used = True
                    except Exception as e:
                        self.logerr("{}: Could not use spawn point from parameters, ".format(vehicle["id"]) +
                                    "the spawn point from config file will be used. Error is: {}".format(e))

                if "spawn_point" in vehicle and spawn_param_used is False:
                    # get spawn point from config file
                    try:
                        spawn_point = self.create_spawn_point(
                            vehicle["spawn_point"]["x"],
                            vehicle["spawn_point"]["y"],
                            vehicle["spawn_point"]["z"],
                            vehicle["spawn_point"]["roll"],
                            vehicle["spawn_point"]["pitch"],
                            vehicle["spawn_point"]["yaw"]
                        )
                        self.loginfo("Spawn point from configuration file")
                    except KeyError as e:
                        self.logerr("{}: Could not use the spawn point from config file, ".format(vehicle["id"]) +
                                    "the mandatory attribute {} is missing, a random spawn point will be used".format(e))

                if spawn_point is None:
                    # pose not specified, ask for a random one in the service call
                    self.loginfo("Spawn point selected at random")
                    spawn_point = Pose()  # empty pose
                    spawn_object_request.random_pose = True

                player_spawned = False
                while not player_spawned:
                    spawn_object_request.transform = spawn_point

                    response = self.call_service(self.spawn_object_service, spawn_object_request)
                    if response.id != -1:
                        player_spawned = True
                        players.append(response.id)
                        # Set up the sensors
                        try:
                            self.vehicles_sensors.append(
                                self.setup_sensors(vehicle["sensors"], response.id))
                        except KeyError:
                            self.logwarn(
                                "Vehicle {} have no 'sensors' field in his config file, none will be spawned")

        return players

    def setup_sensors(self, sensors, attached_vehicle_id=0):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_id: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        actors = []
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                sensor_id = str(sensor_spec.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    raise NameError(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                sensor_names.append(sensor_name)

                if attached_vehicle_id == 0 and "pseudo" not in sensor_type:
                    spawn_point = sensor_spec.pop("spawn_point")
                    sensor_transform = self.create_spawn_point(
                        spawn_point.pop("x"),
                        spawn_point.pop("y"),
                        spawn_point.pop("z"),
                        spawn_point.pop("roll", 0.0),
                        spawn_point.pop("pitch", 0.0),
                        spawn_point.pop("yaw", 0.0))
                else:
                    # if sensor attached to a vehicle, or is a 'pseudo_actor', allow default pose
                    spawn_point = sensor_spec.pop("spawn_point", 0)
                    if spawn_point == 0:
                        sensor_transform = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    else:
                        sensor_transform = self.create_spawn_point(
                            spawn_point.pop("x", 0.0),
                            spawn_point.pop("y", 0.0),
                            spawn_point.pop("z", 0.0),
                            spawn_point.pop("roll", 0.0),
                            spawn_point.pop("pitch", 0.0),
                            spawn_point.pop("yaw", 0.0))

                if ROS_VERSION == 1:
                    spawn_object_request = SpawnObjectRequest()
                elif ROS_VERSION == 2:
                    spawn_object_request = SpawnObject.Request()
                spawn_object_request.type = sensor_type
                spawn_object_request.id = sensor_id
                spawn_object_request.attach_to = attached_vehicle_id
                spawn_object_request.transform = sensor_transform
                spawn_object_request.random_pose = False  # never set a random pose for a sensor

                for attribute, value in sensor_spec.items():
                    spawn_object_request.attributes.append(
                        KeyValue(key=str(attribute), value=str(value)))

                response = self.call_service(self.spawn_object_service, spawn_object_request)
                if response.id == -1:
                    raise Exception(response.error_string)

            except KeyError as e:
                self.logerr(
                    "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))
                continue

            except Exception as e:
                self.logerr(
                    "Sensor {} will not be spawned: {}".format(sensor_name, e))
                continue

            actors.append(response.id)
        return actors

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        spawn_point = Pose()
        spawn_point.position.x = x
        spawn_point.position.y = y
        spawn_point.position.z = z
        quat = quaternion_from_euler(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw))

        spawn_point.orientation.x = quat[0]
        spawn_point.orientation.y = quat[1]
        spawn_point.orientation.z = quat[2]
        spawn_point.orientation.w = quat[3]
        return spawn_point

    def check_spawn_point_param(self, spawn_point_parameter):
        components = spawn_point_parameter.split(',')
        if len(components) != 6:
            raise ValueError("Invalid spawnpoint '{}'".format(spawn_point_parameter))
        spawn_point = self.create_spawn_point(
            float(components[0]),
            float(components[1]),
            float(components[2]),
            float(components[3]),
            float(components[4]),
            float(components[5])
        )
        return spawn_point

    def destroy(self):
        """
        destroy all the players and sensors
        """
        self.loginfo("Shutting down...")
        # destroy global sensors
        for actor_id in self.global_sensors:
            if ROS_VERSION == 1:
                destroy_object_request = DestroyObjectRequest(actor_id)
            elif ROS_VERSION == 2:
                destroy_object_request = DestroyObject.Request(actor_id)
            try:
                self.call_service(self.destroy_object_service, destroy_object_request)
            except ServiceException as e:
                self.logwarn_once(str(e))
        self.global_sensors = []

        # destroy vehicles sensors
        for vehicle_sensors_id in self.vehicles_sensors:
            for actor_id in vehicle_sensors_id:
                if ROS_VERSION == 1:
                    destroy_object_request = DestroyObjectRequest(actor_id)
                elif ROS_VERSION == 2:
                    destroy_object_request = DestroyObject.Request(actor_id)
                try:
                    self.call_service(self.destroy_object_service, destroy_object_request)
                except ServiceException as e:
                    self.logwarn_once(str(e))
        self.vehicles_sensors = []

        # destroy player
        for player_id in self.players:
            if ROS_VERSION == 1:
                destroy_object_request = DestroyObjectRequest(player_id)
            elif ROS_VERSION == 2:
                destroy_object_request = DestroyObject.Request(player_id)
            try:
                self.call_service(self.destroy_object_service, destroy_object_request)
            except ServiceException as e:
                self.logwarn_once(str(e))
        self.players = []

    def run(self):
        """
        main loop
        """
        self.on_shutdown(self.destroy)
        self.spawn_objects()
        try:
            self.spin()
        except ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main(args=None):
    """
    main function
    """
    ros_init(args)
    spawn_objects_node = None
    try:
        spawn_objects_node = CarlaSpawnObjects()
        spawn_objects_node.run()
    except Exception as e:
        print(
            "Exception caught: {}".format(e))
    finally:
        if spawn_objects_node is not None:
            spawn_objects_node.destroy()


if __name__ == '__main__':
    main()
