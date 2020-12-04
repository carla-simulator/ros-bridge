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
import carla_common.sensors as sensors_factory

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
        self.sensor_actors = sensors_factory.setup_sensors(self.world, json_sensors["sensors"])

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
