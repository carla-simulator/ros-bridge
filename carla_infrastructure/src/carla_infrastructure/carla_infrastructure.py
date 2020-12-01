#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
base class for spawning infrastructure sensors in ROS
"""

from __future__ import print_function

import os
import json
import math
import rospy

import carla_common.transforms as trans

from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point, Pose

from carla_msgs.msg import CarlaWorldInfo
from carla_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

import carla

# ==============================================================================
# -- CarlaInfrastructure ------------------------------------------------------------
# ==============================================================================


class CarlaInfrastructure(object):

    """
    Handles the spawning of the infrastructure sensors
    """

    def __init__(self):
        rospy.init_node('infrastructure', anonymous=True)
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', 2000)
        self.timeout = rospy.get_param('/carla/timeout', 10)
        self.sensor_definition_file = rospy.get_param('~infrastructure_sensor_definition_file')
        self.world = None
        self.sensor_actors = []

        rospy.wait_for_service('/carla/spawn_object')
        self.spawn_object_service = rospy.ServiceProxy("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = rospy.ServiceProxy("/carla/destroy_object", DestroyObject)

    def restart(self):
        """
        Spawns the infrastructure sensors

        :return:
        """
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
        Create the sensors defined by the user and spawn them in the world
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

                if sensor_id in sensor_names:
                    rospy.logfatal(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                    raise NameError(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                sensor_names.append(sensor_id)

                sensor_location = Point(x=sensor_spec.pop("x", 0.0),
                                        y=sensor_spec.pop("y", 0.0),
                                        z=sensor_spec.pop("z", 0.0))
                sensor_rotation = trans.RPY_to_ros_quaternion(
                    roll=math.radians(sensor_spec.pop('roll', 0.0)),
                    pitch=math.radians(sensor_spec.pop('pitch', 0.0)),
                    yaw=math.radians(sensor_spec.pop('yaw', 0.0)))

                spawn_object_request = SpawnObjectRequest()
                spawn_object_request.type = sensor_type
                spawn_object_request.id = sensor_id
                spawn_object_request.attach_to = 0
                spawn_object_request.transform = Pose(sensor_location, sensor_rotation)
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
                    "Sensor {} will not be spawned, {}".format(sensor_id, e))
                raise e

            actors.append(response.id)
        return actors

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        for actor_id in self.sensor_actors:
            destroy_object_request = DestroyObjectRequest(actor_id)
            try:
                response = self.destroy_object_service(destroy_object_request)
            except rospy.ServiceException as e:
                rospy.logwarn_once(str(e))

        self.sensor_actors = []

    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr("Timeout while waiting for world info!")
            raise e
        rospy.loginfo("CARLA world available. Spawn infrastructure...")
        rospy.on_shutdown(self.destroy)
        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
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
    infrastructure = CarlaInfrastructure()
    try:
        infrastructure.run()
    finally:
        if infrastructure is not None:
            infrastructure.destroy()


if __name__ == '__main__':
    main()
