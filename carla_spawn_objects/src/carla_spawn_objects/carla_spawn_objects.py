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

from abc import abstractmethod

import os
import sys
import math
import json
import rospy
from tf.transformations import quaternion_from_euler
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
from carla_msgs.msg import CarlaWorldInfo, CarlaActorList


from carla_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

# ==============================================================================
# -- CarlaSpawnObjects ------------------------------------------------------------
# ==============================================================================


class CarlaSpawnObjects(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        rospy.init_node('spawn_objects_node', anonymous=True)
        self.objects_definition_file = rospy.get_param('~objects_definition_file')
        self.spawn_sensors_only = rospy.get_param('~spawn_sensors_only', None)

        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []

        rospy.wait_for_service('/carla/spawn_object')
        rospy.wait_for_service('/carla/destroy_object')

        self.spawn_object_service = rospy.ServiceProxy("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = rospy.ServiceProxy("/carla/destroy_object", DestroyObject)

    def spawn_objects(self):
        """
        Spawns the objects

        Either at a given spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Read sensors from file
        if not os.path.exists(self.objects_definition_file):
            raise RuntimeError(
                "Could not read sensor-definition from {}".format(self.objects_definition_file))

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
                rospy.logwarn(
                    "Object with type {} is not a vehicle, a walker or a sensor, ignoring".format(actor["type"]))
        if self.spawn_sensors_only is True and found_sensor_actor_list is False:
            raise RuntimeError("Parameter 'spawn_sensors_only' enabled, " +
                               "but 'sensor.pseudo.actor_list' is not instantiated, add it to your config file.")

        try:
            self.setup_sensors(global_sensors)
        except RuntimeError as e:
            raise RuntimeError("Setting up global sensors failed: {}".format(e))

        if self.spawn_sensors_only is True:
            # get vehicle id from topic /carla/actor_list for all vehicles listed in config file
            actor_info_list = rospy.wait_for_message("/carla/actor_list", CarlaActorList)
            for vehicle in vehicles:
                for actor_info in actor_info_list.actors:
                    if actor_info.type == vehicle["type"] and actor_info.rolename == vehicle["id"]:
                        vehicle["carla_id"] = actor_info.id

        try:
            self.setup_vehicles(vehicles)
        except RuntimeError as e:
            raise RuntimeError("Setting up vehicles failed: {}".format(e))

    def setup_vehicles(self, vehicles):
        for vehicle in vehicles:
            if self.spawn_sensors_only is True:
                # spawn sensors of already spawned vehicles
                try:
                    carla_id = vehicle["carla_id"]
                except KeyError as e:
                    rospy.logerr(
                        "Could not spawn sensors of vehicle {}, its carla ID is not known.".format(vehicle["id"]))
                    break
                # spawn the vehicle's sensors
                self.setup_sensors(vehicle["sensors"], carla_id)
            else:
                spawn_object_request = SpawnObjectRequest()
                spawn_object_request.type = vehicle["type"]
                spawn_object_request.id = vehicle["id"]
                spawn_object_request.attach_to = 0
                spawn_object_request.random_pose = False

                spawn_point = None

                # check if there's a spawn_point corresponding to this vehicle
                spawn_point_param = rospy.get_param("~spawn_point_" + vehicle["id"], None)
                spawn_param_used = False
                if (spawn_point_param is not None):
                    # try to use spawn_point from parameters
                    spawn_point = self.check_spawn_point_param(spawn_point_param)
                    if spawn_point is None:
                        rospy.logwarn("{}: Could not use spawn point from parameters, ".format(vehicle["id"]) +
                                      "the spawn point from config file will be used.")
                    else:
                        rospy.loginfo("Spawn point from ros parameters")
                        spawn_param_used = True

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
                        rospy.loginfo("Spawn point from configuration file")
                    except KeyError as e:
                        rospy.logerr("{}: Could not use the spawn point from config file, ".format(vehicle["id"]) +
                                     "the mandatory attribute {} is missing, a random spawn point will be used".format(e))

                if spawn_point is None:
                    # pose not specified, ask for a random one in the service call
                    rospy.loginfo("Spawn point selected at random")
                    spawn_point = Pose()  # empty pose
                    spawn_object_request.random_pose = True

                player_spawned = False
                while not player_spawned:
                    spawn_object_request.transform = spawn_point
                    response = self.spawn_object_service(spawn_object_request)
                    if response.id != -1:
                        player_spawned = True
                        self.players.append(response.id)
                        # Set up the sensors
                        try:
                            self.setup_sensors(vehicle["sensors"], response.id)
                        except KeyError:
                            rospy.logwarn(
                                "Vehicle {} have no 'sensors' field in his config file, none will be spawned")

    def setup_sensors(self, sensors, attached_vehicle_id=None):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_id: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_type = str(sensor_spec.pop("type"))
                sensor_id = str(sensor_spec.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    raise NameError
                sensor_names.append(sensor_name)

                if attached_vehicle_id is None and "pseudo" not in sensor_type:
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

                spawn_object_request = SpawnObjectRequest()
                spawn_object_request.type = sensor_type
                spawn_object_request.id = sensor_id
                spawn_object_request.attach_to = attached_vehicle_id if attached_vehicle_id is not None else 0
                spawn_object_request.transform = sensor_transform
                spawn_object_request.random_pose = False  # never set a random pose for a sensor

                attached_objects = []
                for attribute, value in sensor_spec.items():
                    if attribute == "attached_objects":
                        for attached_object in sensor_spec["attached_objects"]:
                            attached_objects.append(attached_object)
                        continue
                    spawn_object_request.attributes.append(
                        KeyValue(str(attribute), str(value)))

                response = self.spawn_object_service(spawn_object_request)
                if response.id == -1:
                    raise RuntimeError(response.error_string)

                if attached_objects:
                    # spawn the attached objects
                    self.setup_sensors(attached_objects, response.id)

                if attached_vehicle_id is None:
                    self.global_sensors.append(response.id)
                else:
                    self.vehicles_sensors.append(response.id)

            except KeyError as e:
                rospy.logerr(
                    "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))
                continue

            except RuntimeError as e:
                rospy.logerr(
                    "Sensor {} will not be spawned: {}".format(sensor_name, e))
                continue

            except NameError:
                rospy.logerr("Sensor rolename '{}' is only allowed to be used once.".format(
                    sensor_spec['id']))
                continue

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
            rospy.logwarn("Invalid spawnpoint '{}'".format(spawn_point_parameter))
            return None
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
        # destroy vehicles sensors
        for actor_id in self.vehicles_sensors:
            destroy_object_request = DestroyObjectRequest(actor_id)
            try:
                response = self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once(str(e))
        self.vehicles_sensors = []

        # destroy global sensors
        for actor_id in self.global_sensors:
            destroy_object_request = DestroyObjectRequest(actor_id)
            try:
                response = self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once(str(e))
        self.global_sensors = []

        # destroy player
        for player_id in self.players:
            destroy_object_request = DestroyObjectRequest(player_id)
            try:
                self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once(str(e))
        self.players = []

    def run(self):
        """
        main loop
        """
        rospy.on_shutdown(self.destroy)
        try:
            self.spawn_objects()
        except (rospy.ROSInterruptException, rospy.ServiceException):
            rospy.logwarn(
                "Spawning process has been interrupted. There might be actors that has not been destroyed properly")
        rospy.spin()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    spawn_objects_node = None
    try:
        spawn_objects_node = CarlaSpawnObjects()
        spawn_objects_node.run()
    except RuntimeError as e:
        rospy.logfatal(
            "Exception caught: {}".format(e))
    finally:
        if spawn_objects_node is not None:
            spawn_objects_node.destroy()


if __name__ == '__main__':
    main()
