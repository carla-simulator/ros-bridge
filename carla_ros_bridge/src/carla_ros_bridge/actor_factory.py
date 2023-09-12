#!/usr/bin/env python3
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import itertools
try:
    import queue
except ImportError:
    import Queue as queue
import time
from enum import Enum
from threading import Thread, Lock

import carla
import numpy as np

import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.actor_control import ActorControl
from carla_ros_bridge.actor_list_sensor import ActorListSensor
from carla_ros_bridge.camera import Camera, RgbCamera, DepthCamera, SemanticSegmentationCamera, DVSCamera
from carla_ros_bridge.collision_sensor import CollisionSensor
from carla_ros_bridge.ego_vehicle import EgoVehicle
from carla_ros_bridge.gnss import Gnss
from carla_ros_bridge.imu import ImuSensor
from carla_ros_bridge.lane_invasion_sensor import LaneInvasionSensor
from carla_ros_bridge.lidar import Lidar, SemanticLidar
from carla_ros_bridge.marker_sensor import MarkerSensor
from carla_ros_bridge.object_sensor import ObjectSensor
from carla_ros_bridge.odom_sensor import OdometrySensor
from carla_ros_bridge.opendrive_sensor import OpenDriveSensor
from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.radar import Radar
from carla_ros_bridge.rss_sensor import RssSensor
from carla_ros_bridge.sensor import Sensor
from carla_ros_bridge.spectator import Spectator
from carla_ros_bridge.speedometer_sensor import SpeedometerSensor
from carla_ros_bridge.tf_sensor import TFSensor
from carla_ros_bridge.traffic import Traffic, TrafficLight
from carla_ros_bridge.traffic_lights_sensor import TrafficLightsSensor
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

# to generate a random spawning position or vehicles
import random
secure_random = random.SystemRandom()


class ActorFactory(object):

    TIME_BETWEEN_UPDATES = 0.1

    class TaskType(Enum):
        SPAWN_ACTOR = 0
        SPAWN_PSEUDO_ACTOR = 1
        DESTROY_ACTOR = 2

    def __init__(self, node, world, sync_mode=False):
        self.node = node
        self.world = world
        self.blueprint_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.sync_mode = sync_mode

        self._active_actors = set()
        self.actors = {}

        self._task_queue = queue.Queue()
        self._known_actor_ids = []  # used to immediately reply to spawn_actor/destroy_actor calls

        self.lock = Lock()
        self.spawn_lock = Lock()

        # id generator for pseudo sensors
        self.id_gen = itertools.count(10000)

        self.thread = Thread(target=self._update_thread)

    def start(self):
        # create initially existing actors
        self.update_available_objects()
        self.thread.start()

    def _update_thread(self):
        """
        execution loop for async mode actor discovery
        """
        while not self.node.shutdown.is_set():
            time.sleep(ActorFactory.TIME_BETWEEN_UPDATES)
            self.world.wait_for_tick()
            self.update_available_objects()

    def update_available_objects(self):
        """
        update the available actors
        """
        # The carla.World.get_actors() method does not return actors that has been spawned in the same frame.
        # This is a known bug and will be fixed in future release of CARLA.
        current_actors = set([actor.id for actor in self.world.get_actors()])
        spawned_actors = current_actors - self._active_actors
        destroyed_actors = self._active_actors - current_actors
        self._active_actors = current_actors

        # Create/destroy actors not managed by the bridge. 
        self.lock.acquire()
        for actor_id in spawned_actors:
            carla_actor = self.world.get_actor(actor_id)
            if self.node.parameters["register_all_sensors"] or not isinstance(carla_actor, carla.Sensor):
                self._create_object_from_actor(carla_actor)

        for actor_id in destroyed_actors:
            self._destroy_object(actor_id, delete_actor=False)

        # Create/destroy objects managed by the bridge.
        with self.spawn_lock:
            while not self._task_queue.empty():
                task = self._task_queue.get()
                task_type = task[0]
                actor_id, req = task[1]

                if task_type == ActorFactory.TaskType.SPAWN_ACTOR and not self.node.shutdown.is_set():
                    carla_actor = self.world.get_actor(actor_id)
                    self._create_object_from_actor(carla_actor, req)
                elif task_type == ActorFactory.TaskType.SPAWN_PSEUDO_ACTOR and not self.node.shutdown.is_set():
                    self._create_object(actor_id, req.type, req.id, req.attach_to, req.transform)
                elif task_type == ActorFactory.TaskType.DESTROY_ACTOR:
                    self._destroy_object(actor_id, delete_actor=True)

        self.lock.release()

    def update_actor_states(self, frame_id, timestamp):
        """
        update the state of all known actors
        """
        with self.lock:
            for actor_id in self.actors:
                try:
                    self.actors[actor_id].update(frame_id, timestamp)
                except RuntimeError as e:
                    self.node.logwarn("Update actor {}({}) failed: {}".format(
                        self.actors[actor_id].__class__.__name__, actor_id, e))
                    continue

    def clear(self):
        for _, actor in self.actors.items():
            actor.destroy()
        self.actors.clear()

    def spawn_actor(self, req):
        """
        spawns an object

        No object instances are created here. Instead carla-actors are created,
        and pseudo objects are appended to a list to get created later.
        """
        with self.spawn_lock:
            if "pseudo" in req.type:
                # only allow spawning pseudo objects if parent actor already exists in carla
                if req.attach_to != 0:
                    carla_actor = self.world.get_actor(req.attach_to)
                    if carla_actor is None:
                        raise IndexError("Parent actor {} not found".format(req.attach_to))
                id_ = next(self.id_gen)
                self._task_queue.put((ActorFactory.TaskType.SPAWN_PSEUDO_ACTOR, (id_, req)))
            else:
                id_ = self._spawn_carla_actor(req)
                self._task_queue.put((ActorFactory.TaskType.SPAWN_ACTOR, (id_, req)))
            self._known_actor_ids.append(id_)
        return id_

    def destroy_actor(self, uid):

        def get_objects_to_destroy(uid):
            objects_to_destroy = []
            if uid in self._known_actor_ids:
                objects_to_destroy.append(uid)
                self._known_actor_ids.remove(uid)

            # remove actors that have the actor to be removed as parent.
            for actor in list(self.actors.values()):
                if actor.parent is not None and actor.parent.uid == uid:
                    objects_to_destroy.extend(get_objects_to_destroy(actor.uid))

            return objects_to_destroy

        with self.spawn_lock:
            objects_to_destroy = set(get_objects_to_destroy(uid))
            for obj in objects_to_destroy:
                self._task_queue.put((ActorFactory.TaskType.DESTROY_ACTOR, (obj, None)))
        return objects_to_destroy

    def _spawn_carla_actor(self, req):
        """
        spawns an actor in carla
        """
        if "*" in req.type:
            blueprint = secure_random.choice(
                self.blueprint_lib.filter(req.type))
        else:
            blueprint = self.blueprint_lib.find(req.type)
        blueprint.set_attribute('role_name', req.id)
        for attribute in req.attributes:
            blueprint.set_attribute(attribute.key, attribute.value)
        if req.random_pose is False:
            transform = trans.ros_pose_to_carla_transform(req.transform)
        else:
            # get a random pose
            transform = secure_random.choice(
                self.spawn_points) if self.spawn_points else carla.Transform()

        attach_to = None
        if req.attach_to != 0:
            attach_to = self.world.get_actor(req.attach_to)
            if attach_to is None:
                raise IndexError("Parent actor {} not found".format(req.attach_to))

        carla_actor = self.world.spawn_actor(blueprint, transform, attach_to)
        return carla_actor.id

    def _create_object_from_actor(self, carla_actor, req=None):
        """
        create a object for a given carla actor
        Creates also the object for its parent, if not yet existing
        """
        parent = None
        # the transform relative to the map
        relative_transform = trans.carla_transform_to_ros_pose(carla_actor.get_transform())
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self._create_object_from_actor(carla_actor.parent)
            if req is not None:
                relative_transform = req.transform
            else:
                # calculate relative transform to the parent
                actor_transform_matrix = trans.ros_pose_to_transform_matrix(relative_transform)
                parent_transform_matrix = trans.ros_pose_to_transform_matrix(
                    trans.carla_transform_to_ros_pose(carla_actor.parent.get_transform()))
                relative_transform_matrix = np.matrix(
                    parent_transform_matrix).getI() * np.matrix(actor_transform_matrix)
                relative_transform = trans.transform_matrix_to_ros_pose(relative_transform_matrix)

        parent_id = 0
        if parent is not None:
            parent_id = parent.uid

        name = carla_actor.attributes.get("role_name", "")
        if not name:
            name = str(carla_actor.id)
        obj = self._create_object(carla_actor.id, carla_actor.type_id, name,
                                  parent_id, relative_transform, carla_actor)
        return obj

    def _destroy_object(self, actor_id, delete_actor):
        if actor_id not in self.actors:
            return
        actor = self.actors[actor_id]
        del self.actors[actor_id]
        carla_actor = None
        if isinstance(actor, Actor):
            carla_actor = actor.carla_actor
        actor.destroy()
        if carla_actor and delete_actor:
            carla_actor.destroy()
        self.node.loginfo("Removed {}(id={})".format(actor.__class__.__name__, actor.uid))

    def get_pseudo_sensor_types(self):
        pseudo_sensors = []
        for cls in PseudoActor.__subclasses__():
            if cls.__name__ != "Actor":
                pseudo_sensors.append(cls.get_blueprint_name())
        return pseudo_sensors

    def _create_object(self, uid, type_id, name, attach_to, spawn_pose, carla_actor=None):
        # check that the actor is not already created.
        if carla_actor is not None and carla_actor.id in self.actors:
            return None

        if attach_to != 0:
            if attach_to not in self.actors:
                raise IndexError("Parent object {} not found".format(attach_to))

            parent = self.actors[attach_to]
        else:
            parent = None

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
                                 actor_list=self.actors,
                                 world=self.world)

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

        self.actors[actor.uid] = actor
        self.node.loginfo("Created {}(id={})".format(actor.__class__.__name__, actor.uid))

        return actor
