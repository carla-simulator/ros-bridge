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

from abc import abstractmethod

import os
import random
import math
import json
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
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
        self.port = rospy.get_param('/carla/port', '2000')
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
        for sensor_spec in sensors:
            try:
                bp = bp_library.find(str(sensor_spec['type']))
                bp.set_attribute('role_name', str(sensor_spec['id']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
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
    infrastructure = CarlaInfrastructure()
    try:
        infrastructure.run()
    finally:
        if infrastructure is not None:
            infrastructure.destroy()


if __name__ == '__main__':
    main()
