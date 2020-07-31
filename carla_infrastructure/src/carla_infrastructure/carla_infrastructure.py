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
        for sensor_spec in sensors:
            try:
                bp = bp_library.find(str(sensor_spec['type']))
                bp.set_attribute('role_name', str(sensor_spec['id']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute(
                        'image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    bp.set_attribute('gamma', str(sensor_spec['gamma']))
                    bp.set_attribute('shutter_speed', str(
                        sensor_spec['shutter_speed']))
                    bp.set_attribute('iso', str(sensor_spec['iso']))
                    bp.set_attribute('fstop', str(sensor_spec['fstop']))
                    bp.set_attribute('min_fstop', str(
                        sensor_spec['min_fstop']))
                    bp.set_attribute('blade_count', str(
                        sensor_spec['blade_count']))
                    bp.set_attribute('exposure_mode', str(
                        sensor_spec['exposure_mode']))
                    bp.set_attribute('exposure_compensation', str(
                        sensor_spec['exposure_compensation']))
                    bp.set_attribute('exposure_min_bright', str(
                        sensor_spec['exposure_min_bright']))
                    bp.set_attribute('exposure_max_bright', str(
                        sensor_spec['exposure_max_bright']))
                    bp.set_attribute('exposure_speed_up', str(
                        sensor_spec['exposure_speed_up']))
                    bp.set_attribute('exposure_speed_down', str(
                        sensor_spec['exposure_speed_down']))
                    bp.set_attribute('calibration_constant', str(
                        sensor_spec['calibration_constant']))
                    bp.set_attribute('focal_distance', str(
                        sensor_spec['focal_distance']))
                    bp.set_attribute('blur_amount', str(
                        sensor_spec['blur_amount']))
                    bp.set_attribute('blur_radius', str(
                        sensor_spec['blur_radius']))
                    bp.set_attribute('motion_blur_intensity', str(
                        sensor_spec['motion_blur_intensity']))
                    bp.set_attribute('motion_blur_max_distortion', str(
                        sensor_spec['motion_blur_max_distortion']))
                    bp.set_attribute('motion_blur_min_object_screen_size', str(
                        sensor_spec['motion_blur_min_object_screen_size']))
                    bp.set_attribute('slope', str(sensor_spec['slope']))
                    bp.set_attribute('toe', str(sensor_spec['toe']))
                    bp.set_attribute('shoulder', str(sensor_spec['shoulder']))
                    bp.set_attribute('black_clip', str(
                        sensor_spec['black_clip']))
                    bp.set_attribute('white_clip', str(
                        sensor_spec['white_clip']))
                    bp.set_attribute('temp', str(sensor_spec['temp']))
                    bp.set_attribute('tint', str(sensor_spec['tint']))
                    bp.set_attribute('chromatic_aberration_intensity', str(
                        sensor_spec['chromatic_aberration_intensity']))
                    bp.set_attribute('chromatic_aberration_offset', str(
                        sensor_spec['chromatic_aberration_offset']))
                    bp.set_attribute('enable_postprocess_effects', str(
                        sensor_spec['enable_postprocess_effects']))
                    bp.set_attribute('lens_circle_falloff', str(
                        sensor_spec['lens_circle_falloff']))
                    bp.set_attribute('lens_circle_multiplier', str(
                        sensor_spec['lens_circle_multiplier']))
                    bp.set_attribute('lens_k', str(sensor_spec['lens_k']))
                    bp.set_attribute('lens_kcube', str(
                        sensor_spec['lens_kcube']))
                    bp.set_attribute('lens_x_size', str(
                        sensor_spec['lens_x_size']))
                    bp.set_attribute('lens_y_size', str(
                        sensor_spec['lens_y_size']))
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
                elif sensor_spec['type'].startswith('sensor.other.imu'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
                elif sensor_spec['type'].startswith('sensor.other.radar'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])

                    bp.set_attribute('horizontal_fov', str(sensor_spec['horizontal_fov']))
                    bp.set_attribute('vertical_fov', str(sensor_spec['vertical_fov']))
                    bp.set_attribute('points_per_second', str(sensor_spec['points_per_second']))
                    bp.set_attribute('range', str(sensor_spec['range']))
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
