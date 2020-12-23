#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Spawns a camera, attached to an ego vehicle.

The pose of the camera can be changed by publishing
to /carla/<ROLENAME>/spectator_position.
"""
import os

import carla
from geometry_msgs.msg import PoseStamped
from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue

import carla
import carla_common.transforms as trans
from transforms3d.euler import quat2euler

from ros_compatibility import (
    CompatibleNode,
    QoSProfile,
    ROSException,
    ROSInterruptException,
    ros_init,
    ServiceException
)

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    from carla_msgs.srv import SpawnObjectRequest, DestroyObjectRequest
# ==============================================================================
# -- CarlaSpectatorCamera ------------------------------------------------------------
# ==============================================================================


class CarlaSpectatorCamera(CompatibleNode):
    """
    Spawns a camera, attached to an ego vehicle.

    The pose of the camera can be changed by publishing
    to /carla/<ROLENAME>/spectator_position.
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaSpectatorCamera, self).__init__('spectator_camera')
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.camera_resolution_x = self.get_param("resolution_x", 800)
        self.camera_resolution_y = self.get_param("resolution_y", 600)
        self.camera_fov = self.get_param("fov", 50)
        self.host = self.get_param('carla/host', '127.0.0.1')
        self.port = self.get_param('carla/port', 2000)
        self.timeout = self.get_param("carla/timeout", 10)
        self.world = None
        self.pose = None
        self.camera_actor = None
        self.ego_vehicle = None

        self.spawn_object_service = self.create_service_client("/carla/spawn_object", SpawnObject)
        self.destroy_object_service = self.create_service_client(
            "/carla/destroy_object", DestroyObject)

        self.create_subscriber(PoseStamped, "/carla/{}/spectator_pose".format(self.role_name),
                               self.pose_received)

    def pose_received(self, pose):
        """
        Move the camera
        """
        if self.pose != pose:
            self.logdebug("Camera pose changed. Updating carla camera")
            self.pose = pose
            transform = self.get_camera_transform()
            if transform and self.camera_actor:
                self.camera_actor.set_transform(transform)

    def get_camera_transform(self):
        """
        Calculate the CARLA camera transform
        """
        if not self.pose:
            self.loginfo("no pose!")
            return None
        if self.pose.header.frame_id != self.role_name:
            self.logwarn("Unsupported frame received. Supported {}, received {}".format(
                self.role_name, self.pose.header.frame_id))
            return None
        sensor_location = carla.Location(x=self.pose.pose.position.x,
                                         y=-self.pose.pose.position.y,
                                         z=self.pose.pose.position.z)
        quaternion = (
            self.pose.pose.orientation.w,
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z
        )
        roll, pitch, yaw = quat2euler(quaternion)
        # rotate to CARLA
        sensor_rotation = carla.Rotation(pitch=math.degrees(roll)-90,
                                         roll=math.degrees(pitch),
                                         yaw=-math.degrees(yaw)-90)
        return carla.Transform(sensor_location, sensor_rotation)

    def create_camera(self, ego_actor):
        """
        Attach the camera to the ego vehicle
        """
        transform = self.get_camera_transform()
        if not transform:
            transform = carla.Transform()

        if ROS_VERSION == 1:
            spawn_object_request = SpawnObjectRequest()
        elif ROS_VERSION == 2:
            spawn_object_request = SpawnObject.Request()
        spawn_object_request.type = "sensor.camera.rgb"
        spawn_object_request.id = "spectator_view"
        spawn_object_request.attach_to = ego_actor.id
        spawn_object_request.transform = trans.carla_transform_to_ros_pose(transform)
        spawn_object_request.random_pose = False
        spawn_object_request.attributes.extend([
            KeyValue(key="image_size_x", value=str(self.camera_resolution_x)),
            KeyValue(key="image_size_y", value=str(self.camera_resolution_y)),
            KeyValue(key="fov", value=str(self.camera_fov))
        ])

        response = self.call_service(self.spawn_object_service, spawn_object_request)
        if response.id == -1:
            raise Exception(response.error_string)

        self.camera_actor = self.world.get_actor(response.id)

    def find_ego_vehicle_actor(self):
        """
        Look for an carla actor with role name
        """
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break
        ego_vehicle_changed = False
        if hero is None and self.ego_vehicle is not None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and \
                self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
            ego_vehicle_changed = True

        if ego_vehicle_changed:
            self.destroy()
            self.loginfo("Ego vehicle changed.")
            self.ego_vehicle = hero
            if self.ego_vehicle:
                self.create_camera(self.ego_vehicle)

    def destroy(self):
        """
        destroy the camera
        """
        if self.camera_actor:
            destroy_object_request = DestroyObjectRequest(self.camera_actor.id)
            try:
                self.call_service(self.destroy_object_service, destroy_object_request)
            except ServiceException as e:
                self.logwarn_once(str(e))

            self.camera_actor = None

    def run(self):
        """
        main loop
        """
        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()

        period = 0.1
        try:
            self.new_timer(period, lambda timer_event=None: self.find_ego_vehicle_actor())
            self.spin()
        except ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main(args=None):
    """
    main function
    """
    ros_init(args)

    carla_spectator_camera = CarlaSpectatorCamera()
    try:
        carla_spectator_camera.run()
    finally:
        if carla_spectator_camera is not None:
            carla_spectator_camera.destroy()


if __name__ == '__main__':
    main()
