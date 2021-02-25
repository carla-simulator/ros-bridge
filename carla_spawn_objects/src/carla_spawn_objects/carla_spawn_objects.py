#!/usr/bin/env python
#
# Copyright (c) 2019-2021 Intel Corporation
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
import sys
import math
import json

import rospy
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose, Point

import carla_common.transforms as trans
from carla_msgs.msg import CarlaActorList
from carla_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

# ==============================================================================
# -- CarlaSpawnObjects ------------------------------------------------------------
# ==============================================================================


class CarlaSpawner(object):

    """
    """

    def __init__(self):
        rospy.init_node("carla_spawner", anonymous=True)
        rospy.on_shutdown(self.destroy)

        self.spawn_sensors_only = rospy.get_param("~spawn_sensors_only", False)

        self._objects = []
        self._actor_list = {}

        rospy.wait_for_service("/carla/spawn_object")
        rospy.wait_for_service("/carla/destroy_object")

        self.spawn_object_service = rospy.ServiceProxy("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = rospy.ServiceProxy("/carla/destroy_object", DestroyObject)

    def _get_object(self, id_):
        if not self._actor_list:
            actor_info_list = rospy.wait_for_message("/carla/actor_list", CarlaActorList)
            for actor_info in actor_info_list:
                self._actor_list[actor_info.rolename] = actor_info.id

        uid = self._actor_list.get(id_, -1)
        if uid == -1:
            rospy.logerr("Object with role name {} not found".format(id_))
        return uid

    def _spawn_object(self, type_, id_, spawn_point, attributes, parent_uid=0):
        """

        """
        if "vehicle" in type_ or "walker" in type_:
            if self.spawn_sensors_only:
                return self._get_object(id_)

        spawn_object_request = SpawnObjectRequest()
        spawn_object_request.type = type_
        spawn_object_request.id = id_
        spawn_object_request.attach_to = parent_uid
        if spawn_point is None:
            if "vehicle" in type_ or "walker" in type_ or "prop" in type_:
                spawn_object_request.random_pose = True
            elif "sensor" in type_ and "pseudo" not in type_:
                rospy.logerr("Object {} could not be spawned. A valid spawn point should be provided".format(id_))
                return -1
        else:
            spawn_object_request.transform = spawn_point
        for attribute, value in attributes.items():
            spawn_object_request.attributes.append(KeyValue(str(attribute), str(value)))

        response = self.spawn_object_service(spawn_object_request)
        if response.id == -1:
            rospy.logerr("Object {} could not be spawned. {}".format(id_, response.error_string))
        else:
            self._objects.append(response.id)
        return response.id

    def _spawn_object_from_dict(self, obj_data, parent_uid):
        if parent_uid == -1:
            rospy.logwarn("Skipping object {}. Invalid parent".format(obj_data["id"]))
            return -1

        if not obj_data.has_key("type") or not obj_data.has_key("id"):
            rospy.logerr("Mandatory argument missing")
            return -1

        def _create_spawn_point_from_param(spawn_point_param):
            try:
                keys = ["x", "y", "z", "roll", "pitch", "yaw"]
                values = [0 if x == "" else float(x) for x in spawn_point_param.split(",")]
                if len(values) != 6:
                    raise ValueError("Spawn point parameter '{}' malformed".format(spawn_point_param))
            except ValueError as e:
                rospy.logwarn("Spawn point parameter not valid. {}".format(str(e)))
                return None
            else:
                return _create_spawn_point_from_dict(dict(zip(keys, values)))

        def _create_spawn_point_from_dict(data):
            x, y, z = data.get("x", 0), data.get("y", 0), data.get("z", 0)
            roll, pitch, yaw = data.get("roll", 0), data.get("pitch", 0), data.get("yaw", 0)
            return Pose(
                Point(x, y, z),
                trans.RPY_to_ros_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))
            )

        def _get_spawn_point():
            spawn_point_ = None
            if rospy.has_param("~spawn_point_{}".format(id_)):
                spawn_point_param = rospy.get_param("~spawn_point_{}".format(id_))
                spawn_point_ = _create_spawn_point_from_param(spawn_point_param)
            if not spawn_point_ and obj_data.has_key("spawn_point"):
                spawn_point_ = _create_spawn_point_from_dict(obj_data.pop("spawn_point"))
            return spawn_point_

        type_ = obj_data.pop("type")
        id_ =  obj_data.pop("id")
        spawn_point = _get_spawn_point()
        attributes = obj_data
        return self._spawn_object(type_, id_, spawn_point, attributes, parent_uid)

    def spawn_objects(self, objects, parent_uid=0):
        """Spawn objects.
        :param objects: ...
        :type objects: ...
        :param parent_uid: ...
        :type parent_uid: ...
        :return: ...
        :rtype: ...
        """
        for obj_data in objects:
            children = obj_data.pop("objects", [])
            uid = self._spawn_object_from_dict(obj_data, parent_uid)
            if children:
                self.spawn_objects(children, parent_uid=uid)

    def destroy(self):
        """
        Destroy all spawned objects.
        """
        for uid in self._objects:
            destroy_object_request = DestroyObjectRequest(uid)
            try:
                self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once(str(e))


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    spawner = CarlaSpawner()

    objects_definition_file = rospy.get_param("~objects_definition_file")
    if not os.path.exists(objects_definition_file):
        raise RuntimeError(
            "Could not read objects-definition from {}".format(objects_definition_file))

    with open(objects_definition_file) as handle:
        objects = json.loads(handle.read())

    try:
        spawner.spawn_objects(objects["objects"], parent_uid=0)
    except rospy.ROSInterruptException:
        rospy.logwarn(
            "Spawning process has been interrupted. There might be actors that has not been destroyed properly")
        pass
    rospy.spin()


if __name__ == '__main__':
    main()
