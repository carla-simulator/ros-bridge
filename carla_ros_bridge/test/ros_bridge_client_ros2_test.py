#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
import os
import unittest

import launch
import launch.actions

import launch_testing
import launch_testing.actions

from ament_index_python.packages import get_package_share_directory

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header, String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, NavSatFix, Image, PointCloud2, Imu
from geometry_msgs.msg import Quaternion, Vector3, Pose
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
from visualization_msgs.msg import MarkerArray
from carla_msgs.msg import (CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaWorldInfo,
                            CarlaActorList, CarlaTrafficLightStatusList,
                            CarlaTrafficLightInfoList)


def generate_test_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.tesla.model3'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_names',
            default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                           "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"]
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_ros_bridge') + '/test/test_objects.json'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'passive': launch.substitutions.LaunchConfiguration('passive'),
                'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_spawn_objects.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
            }.items()
        ),
        # Start tests
        launch_testing.actions.ReadyToTest()
    ])
    return ld


TIMEOUT = 30


class TestClock(unittest.TestCase):

    """
    Handles testing of the all nodes
    """

    def test_clock(self):
        """
        Tests clock
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message("/clock", Clock, timeout=TIMEOUT)
            self.assertNotEqual(Clock(), msg)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_vehicle_status(self, proc_output):
        """
        Tests vehicle_status
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus)
            self.assertNotEqual(msg.header, Header())
            self.assertEqual(msg.header.frame_id, 'map')
            self.assertNotEqual(msg.orientation, Quaternion())
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_vehicle_info(self):
        """
        Tests vehicle_info
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/vehicle_info", CarlaEgoVehicleInfo, timeout=TIMEOUT,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.assertNotEqual(msg.id, 0)
            self.assertEqual(msg.type, "vehicle.tesla.model3")
            self.assertEqual(msg.rolename, "ego_vehicle")
            self.assertEqual(len(msg.wheels), 4)
            self.assertNotEqual(msg.max_rpm, 0.0)
            self.assertNotEqual(msg.moi, 0.0)
            self.assertNotEqual(msg.damping_rate_full_throttle, 0.0)
            self.assertNotEqual(msg.damping_rate_zero_throttle_clutch_engaged, 0.0)
            self.assertNotEqual(
                msg.damping_rate_zero_throttle_clutch_disengaged, 0.0)
            self.assertTrue(msg.use_gear_autobox)
            self.assertNotEqual(msg.gear_switch_time, 0.0)
            self.assertNotEqual(msg.mass, 0.0)
            self.assertNotEqual(msg.clutch_strength, 0.0)
            self.assertNotEqual(msg.drag_coefficient, 0.0)
            self.assertNotEqual(msg.center_of_mass, Vector3())
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_odometry(self):
        """
        Tests Odometry
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/odometry", Odometry, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "map")
            self.assertEqual(msg.child_frame_id, "ego_vehicle")
            self.assertNotEqual(msg.pose, Pose())
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_gnss(self):
        """
        Tests Gnss
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/gnss", NavSatFix, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/gnss")
            self.assertNotEqual(msg.latitude, 0.0)
            self.assertNotEqual(msg.longitude, 0.0)
            self.assertNotEqual(msg.altitude, 0.0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_imu(self):
        """
        Tests IMU sensor node
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message("/carla/ego_vehicle/imu", Imu, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/imu")
            self.assertNotEqual(msg.linear_acceleration, 0.0)
            self.assertNotEqual(msg.angular_velocity, 0.0)
            self.assertNotEqual(msg.orientation, 0.0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_camera_info(self):
        """
        Tests camera_info
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/rgb_front/camera_info", CameraInfo, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/rgb_front")
            self.assertEqual(msg.height, 600)
            self.assertEqual(msg.width, 800)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_camera_image(self):
        """
        Tests camera_images
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/rgb_front/image", Image, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/rgb_front")
            self.assertEqual(msg.height, 600)
            self.assertEqual(msg.width, 800)
            self.assertEqual(msg.encoding, "bgra8")
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_dvs_camera_info(self):
        """
        Tests dvs camera info
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/dvs_front/camera_info", CameraInfo, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/dvs_front")
            self.assertEqual(msg.height, 70)
            self.assertEqual(msg.width, 400)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_dvs_camera_image(self):
        """
        Tests dvs camera images
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/dvs_front/image", Image, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/dvs_front")
            self.assertEqual(msg.height, 70)
            self.assertEqual(msg.width, 400)
            self.assertEqual(msg.encoding, "bgr8")
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_dvs_camera_events(self):
        """
        Tests dvs camera events
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/dvs_front/events", PointCloud2, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/dvs_front")
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_lidar(self):
        """
        Tests Lidar sensor node
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/lidar", PointCloud2, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/lidar")
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_semantic_lidar(self):
        """
        Tests semantic_lidar sensor node
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/semantic_lidar", PointCloud2, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/semantic_lidar")
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_radar(self):
        """
        Tests Radar sensor node
        """
        try:
            node = None
            roscomp.init("test_node")
            node = None
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/radar_front", PointCloud2, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "ego_vehicle/radar_front")
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_ego_vehicle_objects(self):
        """
        Tests objects node for ego_vehicle
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/ego_vehicle/objects", ObjectArray, timeout=15)
            self.assertEqual(msg.header.frame_id, "map")
            self.assertEqual(len(msg.objects), 0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_objects(self):
        """
        Tests carla objects
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message("/carla/objects", ObjectArray, timeout=TIMEOUT)
            self.assertEqual(msg.header.frame_id, "map")
            self.assertEqual(len(msg.objects), 1)  # only ego vehicle exists
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_marker(self):
        """
        Tests marker
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message("/carla/markers", MarkerArray, timeout=TIMEOUT)
            self.assertEqual(len(msg.markers), 1)  # only ego vehicle exists

            ego_marker = msg.markers[0]
            self.assertEqual(ego_marker.header.frame_id, "map")
            self.assertNotEqual(ego_marker.id, 0)
            self.assertEqual(ego_marker.type, 1)
            self.assertNotEqual(ego_marker.pose, Pose())
            self.assertNotEqual(ego_marker.scale, Vector3())
            self.assertEqual(ego_marker.color.r, 0.0)
            self.assertEqual(ego_marker.color.g, 255.0)
            self.assertEqual(ego_marker.color.b, 0.0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_map(self):
        """
        Tests map
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/map", String, timeout=TIMEOUT,
                qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.assertNotEqual(len(msg.data), 0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_world_info(self):
        """
        Tests world_info
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/world_info", CarlaWorldInfo, timeout=TIMEOUT,
                qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.assertNotEqual(len(msg.map_name), 0)
            self.assertNotEqual(len(msg.opendrive), 0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_actor_list(self):
        """
        Tests actor_list
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/actor_list", CarlaActorList, timeout=TIMEOUT)
            self.assertNotEqual(len(msg.actors), 0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_traffic_lights(self):
        """
        Tests traffic_lights
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/traffic_lights/status", CarlaTrafficLightStatusList, timeout=TIMEOUT,
                qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.assertNotEqual(len(msg.traffic_lights), 0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()

    def test_traffic_lights_info(self):
        """
        Tests traffic_lights
        """
        try:
            node = None
            roscomp.init("test_node")
            node = CompatibleNode('test_node')
            msg = node.wait_for_message(
                "/carla/traffic_lights/info", CarlaTrafficLightInfoList, timeout=TIMEOUT,
                qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.assertNotEqual(len(msg.traffic_lights), 0)
        finally:
            if node is not None:
                node.destroy_node()
            roscomp.shutdown()
