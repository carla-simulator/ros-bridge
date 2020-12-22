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

import carla_common.transforms as trans

from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose, Point
from carla_msgs.msg import CarlaWorldInfo, CarlaActorList

from carla_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

# ==============================================================================
# -- CarlaSpawnObjects ------------------------------------------------------------
# ==============================================================================

def validate(func):
    def is_valid(self, type_, id_, spawn_point, attributes, parent_uid):
        #if "sensor" in type_ and parent_uid == 0:
        #    if spawn_point is None:
        #        raise RuntimeError("Sensor {} d")

        return func(self, type_, id_, spawn_point, attributes, parent_uid)
    return is_valid


class CarlaSpawnObjects(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        rospy.init_node("carla_spawner", anonymous=True)

        self.objects_definition_file = rospy.get_param("~objects_definition_file")
        self.spawn_sensors_only = rospy.get_param("~spawn_sensors_only", None)

        self._objects = []

        rospy.wait_for_service("/carla/spawn_object")
        rospy.wait_for_service("/carla/destroy_object")

        self.spawn_object_service = rospy.ServiceProxy("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = rospy.ServiceProxy("/carla/destroy_object", DestroyObject)

    def _get_object(self, id_):
        # wait_for_message
        uid = 0
        return uid

    @validate
    def _spawn_object(self, type_, id_, spawn_point, attributes, parent_uid=0):
        if self.spawn_sensors_only:
            return self._get_object(id_)

        spawn_object_request = SpawnObjectRequest()
        spawn_object_request.type = type_
        spawn_object_request.id = id_
        spawn_object_request.attach_to = parent_uid

        if spawn_point is None:
            if "vehicle" in type_ or "walker" in type_:
                spawn_object_request.random_pose = True
        else:
            spawn_object_request.transform = spawn_point

        response = self.spawn_object_service(spawn_object_request)
        if response.id == -1:
            raise RuntimeError("Object {} could not be spawned. {}".format(response.error_string))

        self._objects.append(response.id)
        return response.id

    def _spawn_object_from_dict(self, obj_data, parent_uid):

        def _create_spawn_point_from_param(id_):
            pass

        def _create_spawn_point_from_dict(data):
            x, y, z = data.pop("x", 0), data.pop("y", 0), data.pop("z", 0)
            roll, pitch, yaw = data.pop("roll", 0), data.pop("pitch", 0), data.pop("yaw", 0)
            return Pose(
                Point(x, y, z),
                trans.RPY_to_ros_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))
            )

        def _get_spawn_point():
#            if rospy.has_param("spawn_point_{}".format(id_)):
#                spawn_point = self.create_spawn_point_from_param(id_)
            if obj_data.has_key("spawn_point"):
                spawn_point = _create_spawn_point_from_dict(obj_data.pop("spawn_point"))
            else:
                spawn_point = None
            return spawn_point

        type_ = obj_data.pop("type")
        id_ =  obj_data.pop("id")
        spawn_point = _get_spawn_point()
        attributes = obj_data
        return self._spawn_object(type_, id_, spawn_point, attributes, parent_uid)

    def spawn_objects(self, objects, parent_uid=0):
        for obj_data in objects:
            childs = obj_data.pop("objects", [])
            uid = self._spawn_object_from_dict(obj_data, parent_uid)
            if childs:
                self.spawn_objects(childs, parent_uid=uid)

    def destroy(self):
        """
        destroy all objects
        """
        for uid in self._objects:
            destroy_object_request = DestroyObjectRequest(uid)
            try:
                response = self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once("{} for object with uid: {}".format(str(e), uid))

    def run(self):
        """
        main loop
        """
        rospy.on_shutdown(self.destroy)
        if not os.path.exists(self.objects_definition_file):
            raise RuntimeError(
                "Could not read objects-definition from {}".format(self.objects_definition_file))

        with open(self.objects_definition_file) as handle:
            objects = json.loads(handle.read())

        try:
            self.spawn_objects(objects["objects"], parent_uid=0)
        except rospy.ROSInterruptException:
            rospy.logwarn(
                "Spawning process has been interrupted. There might be actors that has not been destroyed properly")
            pass
        rospy.spin()

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    """
    main function
    """
    spawn_objects_node = None
    spawn_objects_node = CarlaSpawnObjects()
    spawn_objects_node.run()

if __name__ == '__main__':
    main()
