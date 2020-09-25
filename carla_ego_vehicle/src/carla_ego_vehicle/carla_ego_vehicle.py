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
import json
import math
import os
import random
import traceback
from abc import abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose  # pylint: disable=import-error
from carla_msgs.msg import CarlaWorldInfo  # pylint: disable=import-error
from ros_compatibility import (
    CompatibleNode,
    destroy_subscription,
    euler_from_quaternion,
    quaternion_from_euler,
)

import carla

ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 2:
    import rclpy  # pylint: disable=import-error
    from ament_index_python.packages import get_package_share_directory  # pylint: disable=import-error


secure_random = random.SystemRandom()

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class CarlaEgoVehicle(CompatibleNode):
    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        super(CarlaEgoVehicle, self).__init__('ego_vehicle_node')
        self.world = None
        self.player = None
        self.player_created = False
        self.sensor_actors = []
        self.actor_spawnpoint = None
        self.timeout = self.get_param('carla/timeout', 2)
        self.host = self.get_param('carla/host', 'localhost')
        self.port = self.get_param('carla/port', 2000)
        self.actor_filter = self.get_param('vehicle_filter', 'vehicle.*')
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        # check argument and set spawn_point
        spawn_point_param = self.get_param('spawn_point')

        if ROS_VERSION == 1:
            self.sensor_definition_file = self.get_param('sensor_definition_file', 'sensors.json')
        elif ROS_VERSION == 2:
            sensor_path = get_package_share_directory('carla_ego_vehicle') + '/config/sensors.json'
            self.sensor_definition_file = self.get_param('sensor_definition_file', sensor_path)
        if spawn_point_param and self.spawn_ego_vehicle():
            self.loginfo("Using ros parameter for spawnpoint: {}".format(spawn_point_param))
            spawn_point = spawn_point_param.split(',')
            if len(spawn_point) != 6:
                raise ValueError("Invalid spawnpoint '{}'".format(spawn_point_param))
            pose = Pose()
            pose.position.x = float(spawn_point[0])
            pose.position.y = -float(spawn_point[1])
            pose.position.z = float(spawn_point[2])
            quat = quaternion_from_euler(
                math.radians(float(spawn_point[3])),
                math.radians(float(spawn_point[4])),
                math.radians(float(spawn_point[5])))
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.actor_spawnpoint = pose

        self.initialpose_subscriber = self.create_subscriber(PoseWithCovarianceStamped,
                                                             "/carla/{}/initialpose".format(
                                                                 self.role_name),
                                                             self.on_initialpose)

        self.loginfo("listening to server {}:{}".format(self.host, self.port))
        self.loginfo("using vehicle filter: {}".format(self.actor_filter))

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        self.startup_subscription = self.create_subscriber(
            CarlaWorldInfo,
            '/carla/world_info',
            self.run,
            1
        )

    # pylint: disable=inconsistent-return-statements
    def spawn_ego_vehicle(self):
        """
        Helper method for condition-checking in self.restart().
        """
        if ROS_VERSION == 1:
            return self.get_param('~spawn_ego_vehicle')
        elif ROS_VERSION == 2:
            return self.get_parameter('spawn_ego_vehicle')

    def on_initialpose(self, initial_pose):
        """
        Callback for /initialpose

        Receiving an initial pose (e.g. from RVIZ '2D Pose estimate') triggers a respawn.

        :return:
        """
        self.actor_spawnpoint = initial_pose.pose.pose
        self.restart()

    def restart(self):
        """
        (Re)spawns the vehicle

        Either at a given actor_spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Get vehicle blueprint.
        blueprint = secure_random.choice(
            self.world.get_blueprint_library().filter(self.actor_filter))
        blueprint.set_attribute('role_name', "{}".format(self.role_name))
        if blueprint.has_attribute('color'):
            color = secure_random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the vehicle.
        if not self.spawn_ego_vehicle():
            actors = self.world.get_actors().filter(self.actor_filter)
            for actor in actors:
                if actor.attributes['role_name'] == self.role_name:
                    self.player = actor
                    break
        else:
            if self.actor_spawnpoint:
                spawn_point = carla.Transform()
                spawn_point.location.x = self.actor_spawnpoint.position.x
                spawn_point.location.y = -self.actor_spawnpoint.position.y
                spawn_point.location.z = self.actor_spawnpoint.position.z + \
                    2  # spawn 2m above ground
                quaternion = [
                    self.actor_spawnpoint.orientation.x,
                    self.actor_spawnpoint.orientation.y,
                    self.actor_spawnpoint.orientation.z,
                    self.actor_spawnpoint.orientation.w
                ]
                _, _, yaw = euler_from_quaternion(quaternion)
                spawn_point.rotation.yaw = -math.degrees(yaw)
                self.loginfo("Spawn {} at x={} y={} z={} yaw={}".format(self.role_name,
                                                                        spawn_point.location.x,
                                                                        spawn_point.location.y,
                                                                        spawn_point.location.z,
                                                                        spawn_point.rotation.yaw))
                if self.player is not None:
                    self.player.set_transform(spawn_point)
                while self.player is None:
                    self.player = self.world.try_spawn_actor(blueprint, spawn_point)
                    self.player_created = True

            else:
                if self.player is not None:
                    spawn_point = self.player.get_transform()
                    spawn_point.location.z += 2.0
                    spawn_point.rotation.roll = 0.0
                    spawn_point.rotation.pitch = 0.0
                    self.player.set_transform(spawn_point)
                while self.player is None:
                    spawn_points = self.world.get_map().get_spawn_points()
                    spawn_point = secure_random.choice(
                        spawn_points) if spawn_points else carla.Transform()
                    self.player = self.world.try_spawn_actor(blueprint, spawn_point)
                    self.player_created = True

        if self.player_created:
            # Read sensors from file
            if not os.path.exists(self.sensor_definition_file):
                raise RuntimeError(
                    "Could not read sensor-definition from {}".format(self.sensor_definition_file))
            json_sensors = None
            with open(self.sensor_definition_file) as handle:
                json_sensors = json.loads(handle.read())

            # Set up the sensors
            self.sensor_actors = self.setup_sensors(json_sensors["sensors"])

            self.player_created = False

    def setup_sensors(self, sensors):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :return:
        """
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                sensor_name = str(sensor_spec['type']) + "/" + str(sensor_spec['id'])
                if sensor_name in sensor_names:
                    self.logfatal(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                    raise NameError(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                sensor_names.append(sensor_name)
                bp = bp_library.find(str(sensor_spec['type']))
                bp.set_attribute('role_name', str(sensor_spec['id']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    try:
                        bp.set_attribute('sensor_tick', str(sensor_spec['sensor_tick']))
                    except KeyError:
                        pass
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                    if sensor_spec['type'].startswith('sensor.camera.rgb'):
                        bp.set_attribute('gamma', str(sensor_spec['gamma']))
                        bp.set_attribute('shutter_speed', str(sensor_spec['shutter_speed']))
                        bp.set_attribute('iso', str(sensor_spec['iso']))
                        bp.set_attribute('fstop', str(sensor_spec['fstop']))
                        bp.set_attribute('min_fstop', str(sensor_spec['min_fstop']))
                        bp.set_attribute('blade_count', str(sensor_spec['blade_count']))
                        bp.set_attribute('exposure_mode', str(sensor_spec['exposure_mode']))
                        bp.set_attribute('exposure_compensation', str(
                            sensor_spec['exposure_compensation']))
                        bp.set_attribute('exposure_min_bright', str(
                            sensor_spec['exposure_min_bright']))
                        bp.set_attribute('exposure_max_bright', str(
                            sensor_spec['exposure_max_bright']))
                        bp.set_attribute('exposure_speed_up', str(sensor_spec['exposure_speed_up']))
                        bp.set_attribute('exposure_speed_down', str(
                            sensor_spec['exposure_speed_down']))
                        bp.set_attribute('calibration_constant', str(
                            sensor_spec['calibration_constant']))
                        bp.set_attribute('focal_distance', str(sensor_spec['focal_distance']))
                        bp.set_attribute('blur_amount', str(sensor_spec['blur_amount']))
                        bp.set_attribute('blur_radius', str(sensor_spec['blur_radius']))
                        bp.set_attribute('motion_blur_intensity', str(
                            sensor_spec['motion_blur_intensity']))
                        bp.set_attribute('motion_blur_max_distortion', str(
                            sensor_spec['motion_blur_max_distortion']))
                        bp.set_attribute('motion_blur_min_object_screen_size', str(
                            sensor_spec['motion_blur_min_object_screen_size']))
                        bp.set_attribute('slope', str(sensor_spec['slope']))
                        bp.set_attribute('toe', str(sensor_spec['toe']))
                        bp.set_attribute('shoulder', str(sensor_spec['shoulder']))
                        bp.set_attribute('black_clip', str(sensor_spec['black_clip']))
                        bp.set_attribute('white_clip', str(sensor_spec['white_clip']))
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
                        bp.set_attribute('lens_kcube', str(sensor_spec['lens_kcube']))
                        bp.set_attribute('lens_x_size', str(sensor_spec['lens_x_size']))
                        bp.set_attribute('lens_y_size', str(sensor_spec['lens_y_size']))
                        bp.set_attribute('bloom_intensity', str(sensor_spec['bloom_intensity']))
                        bp.set_attribute('lens_flare_intensity', str(
                            sensor_spec['lens_flare_intensity']))
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    bp.set_attribute('range', str(sensor_spec['range']))
                    bp.set_attribute('rotation_frequency', str(sensor_spec['rotation_frequency']))
                    bp.set_attribute('channels', str(sensor_spec['channels']))
                    bp.set_attribute('upper_fov', str(sensor_spec['upper_fov']))
                    bp.set_attribute('lower_fov', str(sensor_spec['lower_fov']))
                    bp.set_attribute('points_per_second', str(sensor_spec['points_per_second']))
                    try:
                        bp.set_attribute('sensor_tick', str(sensor_spec['sensor_tick']))
                    except KeyError:
                        pass
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
                    bp.set_attribute('noise_alt_stddev', str(sensor_spec['noise_alt_stddev']))
                    bp.set_attribute('noise_lat_stddev', str(sensor_spec['noise_lat_stddev']))
                    bp.set_attribute('noise_lon_stddev', str(sensor_spec['noise_lon_stddev']))
                    bp.set_attribute('noise_alt_bias', str(sensor_spec['noise_alt_bias']))
                    bp.set_attribute('noise_lat_bias', str(sensor_spec['noise_lat_bias']))
                    bp.set_attribute('noise_lon_bias', str(sensor_spec['noise_lon_bias']))
                elif sensor_spec['type'].startswith('sensor.other.imu'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                    bp.set_attribute('noise_accel_stddev_x',
                                     str(sensor_spec['noise_accel_stddev_x']))
                    bp.set_attribute('noise_accel_stddev_y',
                                     str(sensor_spec['noise_accel_stddev_y']))
                    bp.set_attribute('noise_accel_stddev_z',
                                     str(sensor_spec['noise_accel_stddev_z']))

                    bp.set_attribute('noise_gyro_stddev_x', str(sensor_spec['noise_gyro_stddev_x']))
                    bp.set_attribute('noise_gyro_stddev_y', str(sensor_spec['noise_gyro_stddev_y']))
                    bp.set_attribute('noise_gyro_stddev_z', str(sensor_spec['noise_gyro_stddev_z']))
                    bp.set_attribute('noise_gyro_bias_x', str(sensor_spec['noise_gyro_bias_x']))
                    bp.set_attribute('noise_gyro_bias_y', str(sensor_spec['noise_gyro_bias_y']))
                    bp.set_attribute('noise_gyro_bias_z', str(sensor_spec['noise_gyro_bias_z']))
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
                self.logfatal(
                    "Sensor will not be spawned, because sensor spec is invalid: '{}'".format(e))
                raise e

            # create sensor
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = self.world.spawn_actor(bp, sensor_transform,
                                            attach_to=self.player)
            actors.append(sensor)
        return actors

    @abstractmethod
    def sensors(self):
        """
        return a list of sensors attached
        """
        return []

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        for i, _ in enumerate(self.sensor_actors):
            if self.sensor_actors[i] is not None:
                self.sensor_actors[i].destroy()
                self.sensor_actors[i] = None
        self.sensor_actors = []

        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None
        super(CarlaEgoVehicle, self).destroy()

    def run(self, _msg):
        """
        Called after the Carla world was loaded; spawns the ego vehicle.
        """
        self.loginfo("CARLA world available. Spawn ego vehicle...")
        # The subscription should only call this function once
        destroy_subscription(self.startup_subscription)

        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(self.timeout)
            self.world = client.get_world()
            self.restart()
        except Exception:  # pylint: disable=broad-except
            self.logerr(f"Could not run node: {traceback.format_exc()}")
            self.shutdown()
            return

        self.loginfo("Ego spawned.")


def main():
    """
    main function
    """
    executor = None
    if ROS_VERSION == 2:
        rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor()

    ego_vehicle = CarlaEgoVehicle()
    ego_vehicle.spin(executor)


if __name__ == '__main__':
    main()
