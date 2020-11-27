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
import rospy
from carla_msgs.msg import CarlaWorldInfo

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

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    rospy.logfatal(
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
                rospy.logfatal(
                    "Sensor will not be spawned, the mandatory attribute {} is missing"
                    .format(e))
                raise e

            except IndexError as e:
                rospy.logfatal(
                    "Sensor {} will not be spawned, {}".format(sensor_name, e))
                raise e

            # create sensor
            sensor = self.world.spawn_actor(bp, sensor_transform)
            actors.append(sensor)
        return actors

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        for i, _ in enumerate(self.sensor_actors):
            if self.sensor_actors[i] is not None:
                self.sensor_actors[i].destroy()
                self.sensor_actors[i] = None
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
