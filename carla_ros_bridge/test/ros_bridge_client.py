#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
PKG = 'test_roslaunch'

import rospy
import rostest
import unittest
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, NavSatFix, Image, PointCloud2
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaMapInfo, CarlaActorList
from geometry_msgs.msg import Quaternion, Vector3, Pose
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
from visualization_msgs.msg import Marker

class TestClock(unittest.TestCase):
    def test_clock(self):
        rospy.init_node('test_node', anonymous=True)
        clock_msg = rospy.wait_for_message("/clock", Clock, timeout=15)
        self.assertNotEqual(Clock(), clock_msg)

    def test_vehicle_status(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, timeout=15)
        self.assertNotEqual(msg.header, Header()) #todo: check frame-id
        self.assertNotEqual(msg.orientation, Quaternion())

    def test_vehicle_info(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/vehicle_info", CarlaEgoVehicleInfo, timeout=15)
        self.assertNotEqual(msg.id, 0)
        self.assertEqual(msg.type, "vehicle.tesla.model3")
        self.assertEqual(msg.rolename, "ego_vehicle")
        self.assertEqual(len(msg.wheels), 4)
        self.assertNotEqual(msg.max_rpm, 0.0)
        self.assertNotEqual(msg.moi, 0.0)
        self.assertNotEqual(msg.damping_rate_full_throttle, 0.0)
        self.assertNotEqual(msg.damping_rate_zero_throttle_clutch_engaged, 0.0)
        self.assertNotEqual(msg.damping_rate_zero_throttle_clutch_disengaged, 0.0)
        self.assertTrue(msg.use_gear_autobox)
        self.assertNotEqual(msg.gear_switch_time, 0.0)
        self.assertNotEqual(msg.mass, 0.0)
        self.assertNotEqual(msg.clutch_strength, 0.0)
        self.assertNotEqual(msg.drag_coefficient, 0.0)
        self.assertNotEqual(msg.center_of_mass, Vector3())

    def test_odometry(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/odometry", Odometry, timeout=15)
        self.assertEqual(msg.header.frame_id, "map")
        self.assertEqual(msg.child_frame_id, "ego_vehicle")
        self.assertNotEqual(msg.pose, Pose())

    def test_gnss(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/gnss/gnss1/fix", NavSatFix, timeout=15)
        self.assertEqual(msg.header.frame_id, "ego_vehicle/gnss/gnss1")
        self.assertNotEqual(msg.latitude, 0.0)
        self.assertNotEqual(msg.longitude, 0.0)
        self.assertNotEqual(msg.altitude, 0.0)

    def test_camera_info(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/camera/rgb/front/camera_info", CameraInfo, timeout=15)
        self.assertEqual(msg.header.frame_id, "ego_vehicle/camera/rgb/front")
        self.assertEqual(msg.height, 600)
        self.assertEqual(msg.width, 800)

    def test_camera_image(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/camera/rgb/front/image_color", Image, timeout=15)
        self.assertEqual(msg.header.frame_id, "ego_vehicle/camera/rgb/front")
        self.assertEqual(msg.height, 600)
        self.assertEqual(msg.width, 800)
        self.assertEqual(msg.encoding, "bgra8")

    def test_lidar(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/lidar/lidar1/point_cloud", PointCloud2, timeout=15)
        self.assertEqual(msg.header.frame_id, "ego_vehicle/lidar/lidar1")

    def test_ego_vehicle_objects(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/ego_vehicle/objects", ObjectArray, timeout=15)
        self.assertEqual(msg.header.frame_id, "map")
        self.assertEqual(len(msg.objects), 0)

    def test_objects(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/objects", ObjectArray, timeout=15)
        self.assertEqual(msg.header.frame_id, "map")
        self.assertEqual(len(msg.objects), 1) #only ego vehicle exists

    def test_marker(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/marker", Marker, timeout=15)
        self.assertEqual(msg.header.frame_id, "ego_vehicle")
        self.assertNotEqual(msg.id, 0)
        self.assertEqual(msg.type, 1)
        self.assertNotEqual(msg.pose, Pose())
        self.assertNotEqual(msg.scale, Vector3())
        self.assertEqual(msg.color.r, 0.0)
        self.assertEqual(msg.color.g, 255.0)
        self.assertEqual(msg.color.b, 0.0)

    def test_map(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/map", CarlaMapInfo, timeout=15)
        self.assertEqual(msg.header.frame_id, "map")
        self.assertNotEqual(len(msg.map_name), 0)
        self.assertNotEqual(len(msg.opendrive), 0)

    def test_actor_list(self):
        rospy.init_node('test_node', anonymous=True)
        msg = rospy.wait_for_message("/carla/actor_list", CarlaActorList, timeout=15)
        self.assertNotEqual(len(msg.actors), 0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'tests', TestClock)
