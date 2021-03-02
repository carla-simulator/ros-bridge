#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import rospy

try:
    import queue
except ImportError:
    import Queue as queue

import time
from threading import Thread, Lock
import itertools

import carla

import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.spectator import Spectator
from carla_ros_bridge.traffic import Traffic, TrafficLight
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.lidar import Lidar, SemanticLidar
from carla_ros_bridge.radar import Radar
from carla_ros_bridge.gnss import Gnss
from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.imu import ImuSensor
from carla_ros_bridge.ego_vehicle import EgoVehicle
from carla_ros_bridge.collision_sensor import CollisionSensor
from carla_ros_bridge.lane_invasion_sensor import LaneInvasionSensor
from carla_ros_bridge.camera import Camera, RgbCamera, DepthCamera, SemanticSegmentationCamera, DVSCamera
from carla_ros_bridge.object_sensor import ObjectSensor
from carla_ros_bridge.rss_sensor import RssSensor
from carla_ros_bridge.walker import Walker
from carla_ros_bridge.traffic_lights_sensor import TrafficLightsSensor
from carla_ros_bridge.odom_sensor import OdometrySensor
from carla_ros_bridge.speedometer_sensor import SpeedometerSensor
from carla_ros_bridge.tf_sensor import TFSensor
from carla_ros_bridge.marker_sensor import MarkerSensor
from carla_ros_bridge.actor_list_sensor import ActorListSensor
from carla_ros_bridge.opendrive_sensor import OpenDriveSensor
from carla_ros_bridge.actor_control import ActorControl


class ActorFactory(object):

    TIME_BETWEEN_UPDATES = 0.1

    def __init__(self, node, world, sync_mode=False):
        self.node = node
        self.world = world
        self.sync_mode = sync_mode

        self.actors = {}

        self.lock = Lock()
        self.spawn_lock = Lock()

        # id generator for pseudo sensors
        self.id_gen = itertools.count(10000)

    def start(self):
        # create initially existing actors
        self.update()

        self.thread = Thread(target=self._update_thread)
        self.thread.start()

    def _update_thread(self):
        """
        execution loop for async mode actor discovery
        """
        while not self.node.shutdown.is_set():
            time.sleep(ActorFactory.TIME_BETWEEN_UPDATES)
            with self.spawn_lock:
                self.world.wait_for_tick()
                self.update()

    def update(self):
        """
        update the available actors
        """
        # get only carla actors
        previous_actors = set([x.uid for x in self.actors.values() if isinstance(x, Actor)])
        current_actors = set([x.id for x in self.world.get_actors()])

        new_actors = current_actors - previous_actors
        for actor_id in new_actors:
            carla_actor = self.world.get_actor(actor_id)
            self._create_carla_actor(carla_actor)

        deleted_actors = previous_actors - current_actors
        for actor_id in deleted_actors:
            self.destroy(actor_id)

    def clear(self):
        ids = self.actors.keys()
        for id_ in list(ids):
            self.destroy(id_)

    def _create_carla_actor(self, carla_actor):
        parent = None
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self._create_carla_actor(carla_actor.parent)

        parent_id = 0
        if parent is not None:
            parent_id = parent.uid

        name = carla_actor.attributes.get("role_name", "")
        if not name:
            name = str(carla_actor.id)
        return self.create(carla_actor.type_id, name, parent_id, None, carla_actor)

    def create(self, type_id, name, attach_to, spawn_pose, carla_actor=None):
        with self.lock:
            # check that the actor is not already created.
            if carla_actor is not None and carla_actor.id in self.actors:
                return None

            if attach_to != 0:
                if attach_to not in self.actors:
                    raise IndexError("Parent object {} not found".format(attach_to))
                parent = self.actors[attach_to]
            else:
                parent = None

            if carla_actor is not None:
                uid = carla_actor.id
            else:
                uid = next(self.id_gen)

            actor = self._create_object(uid, type_id, name, parent, spawn_pose, carla_actor)
            self.actors[actor.uid] = actor

            rospy.loginfo("Created {}(id={})".format(actor.__class__.__name__, actor.uid))

            return actor

    def destroy(self, actor_id):
        with self.lock:
            # check that the actor is not already removed.
            if actor_id not in self.actors:
                return

            actor = self.actors.pop(actor_id)
            actor.destroy()

            rospy.loginfo("Removed {}(id={})".format(actor.__class__.__name__, actor.uid))

    def get_pseudo_sensor_types(self):
        pseudo_sensors = []
        for cls in PseudoActor.__subclasses__():
            if cls.__name__ != "Actor":
                pseudo_sensors.append(cls.get_blueprint_name())
        return pseudo_sensors

    def _create_object(self, uid, type_id, name, parent, spawn_pose, carla_actor=None):

        if type_id == TFSensor.get_blueprint_name():
            actor = TFSensor(uid=uid, name=name, parent=parent, node=self.node)

        elif type_id == OdometrySensor.get_blueprint_name():
            actor = OdometrySensor(uid=uid,
                                   name=name,
                                   parent=parent,
                                   node=self.node)

        elif type_id == SpeedometerSensor.get_blueprint_name():
            actor = SpeedometerSensor(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=self.node)

        elif type_id == MarkerSensor.get_blueprint_name():
            actor = MarkerSensor(uid=uid,
                                 name=name,
                                 parent=parent,
                                 node=self.node,
                                 actor_list=self.actors)

        elif type_id == ActorListSensor.get_blueprint_name():
            actor = ActorListSensor(uid=uid,
                                    name=name,
                                    parent=parent,
                                    node=self.node,
                                    actor_list=self.actors)

        elif type_id == ObjectSensor.get_blueprint_name():
            actor = ObjectSensor(
                uid=uid,
                name=name,
                parent=parent,
                node=self.node,
                actor_list=self.actors,
            )

        elif type_id == TrafficLightsSensor.get_blueprint_name():
            actor = TrafficLightsSensor(
                uid=uid,
                name=name,
                parent=parent,
                node=self.node,
                actor_list=self.actors,
            )

        elif type_id == OpenDriveSensor.get_blueprint_name():
            actor = OpenDriveSensor(uid=uid,
                                    name=name,
                                    parent=parent,
                                    node=self.node,
                                    carla_map=self.world.get_map())

        elif type_id == ActorControl.get_blueprint_name():
            actor = ActorControl(uid=uid,
                                 name=name,
                                 parent=parent,
                                 node=self.node)

        elif carla_actor.type_id.startswith('traffic'):
            if carla_actor.type_id == "traffic.traffic_light":
                actor = TrafficLight(uid, name, parent, self.node, carla_actor)
            else:
                actor = Traffic(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("vehicle"):
            if carla_actor.attributes.get('role_name')\
                    in self.node.parameters['ego_vehicle']['role_name']:
                actor = EgoVehicle(
                    uid, name, parent, self.node, carla_actor,
                    self.node._ego_vehicle_control_applied_callback)
            else:
                actor = Vehicle(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("sensor"):
            if carla_actor.type_id.startswith("sensor.camera"):
                if carla_actor.type_id.startswith("sensor.camera.rgb"):
                    actor = RgbCamera(uid, name, parent, spawn_pose, self.node,
                                      carla_actor, self.sync_mode)
                elif carla_actor.type_id.startswith("sensor.camera.depth"):
                    actor = DepthCamera(uid, name, parent, spawn_pose,
                                        self.node, carla_actor, self.sync_mode)
                elif carla_actor.type_id.startswith(
                        "sensor.camera.semantic_segmentation"):
                    actor = SemanticSegmentationCamera(uid, name, parent,
                                                       spawn_pose, self.node,
                                                       carla_actor,
                                                       self.sync_mode)
                elif carla_actor.type_id.startswith("sensor.camera.dvs"):
                    actor = DVSCamera(uid, name, parent, spawn_pose, self.node,
                                      carla_actor, self.sync_mode)
                else:
                    actor = Camera(uid, name, parent, spawn_pose, self.node,
                                   carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.lidar"):
                if carla_actor.type_id.endswith("sensor.lidar.ray_cast"):
                    actor = Lidar(uid, name, parent, spawn_pose, self.node,
                                  carla_actor, self.sync_mode)
                elif carla_actor.type_id.endswith(
                        "sensor.lidar.ray_cast_semantic"):
                    actor = SemanticLidar(uid, name, parent, spawn_pose,
                                          self.node, carla_actor,
                                          self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.radar"):
                actor = Radar(uid, name, parent, spawn_pose, self.node,
                              carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.gnss"):
                actor = Gnss(uid, name, parent, spawn_pose, self.node,
                             carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.imu"):
                actor = ImuSensor(uid, name, parent, spawn_pose, self.node,
                                  carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.collision"):
                actor = CollisionSensor(uid, name, parent, spawn_pose,
                                        self.node, carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.rss"):
                actor = RssSensor(uid, name, parent, spawn_pose, self.node,
                                  carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.lane_invasion"):
                actor = LaneInvasionSensor(uid, name, parent, spawn_pose,
                                           self.node, carla_actor,
                                           self.sync_mode)
            else:
                actor = Sensor(uid, name, parent, spawn_pose, self.node,
                               carla_actor, self.sync_mode)
        elif carla_actor.type_id.startswith("spectator"):
            actor = Spectator(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("walker"):
            actor = Walker(uid, name, parent, self.node, carla_actor)
        else:
            actor = Actor(uid, name, parent, self.node, carla_actor)

        return actor
