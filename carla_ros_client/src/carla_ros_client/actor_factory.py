#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import itertools
import time
from threading import Thread, Lock, Event

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ros_client.actor import Actor
from carla_ros_client.lane_invasion_sensor import LaneInvasionSensor
from carla_ros_client.sensor import Sensor
from carla_ros_client.vehicle import Vehicle

# to generate a random spawning position or vehicles
import random
secure_random = random.SystemRandom()


class ActorFactory(CompatibleNode):

    TIME_BETWEEN_UPDATES = 0.1

    def __init__(self):
        """
        Constructor

        """
        super(ActorFactory, self).__init__("carla_ros_client_node")
        self.carla_world = None
        self._active_actors = set()
        self.actors = {}

        self._known_actor_ids = []  # used to immediately reply to spawn_actor/destroy_actor calls

        self.lock = Lock()
        self.shutdown = Event()

        # id generator for pseudo sensors
        self.id_gen = itertools.count(10000)

        self.thread = Thread(target=self._update_thread)

    def start(self, carla_client):
        """
        Initialize the factory and start the update thread
        :param carla_client: carla client object
        :type carla_client: carla.Client
        """
        self.loginfo("Starting...")
        
        self.carla_client = carla_client
        self.carla_world = self.carla_client.get_world()

        # create initially existing actors
        self.update_available_objects()
        self.thread.start()
        self.loginfo("Running...")

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        self.loginfo("Shutting down...")
        self.shutdown.set()

        self.thread.join()

        for _, actor in self.actors.items():
            actor.destroy()
        self.actors.clear()

        self.loginfo("Destroying ros node...")
        super(ActorFactory, self).destroy()

    def _update_thread(self):
        """
        execution loop for async mode actor discovery
        """
        while not self.shutdown.is_set() and roscomp.ok():
            if self.carla_world.get_settings().synchronous_mode:
                self.carla_world.tick()
            else:
                self.carla_world.wait_for_tick()
            self.update_available_objects()

    def update_available_objects(self):
        """
        update the available actors
        """
        # The carla.World.get_actors() method does not return actors that has been spawned in the same frame.
        # This is a known bug and will be fixed in future release of CARLA.
        current_actors = set([actor.id for actor in self.carla_world.get_actors()])
        spawned_actors = current_actors - self._active_actors
        destroyed_actors = self._active_actors - current_actors
        self._active_actors = current_actors

        # Create/destroy actors not managed by the client. 
        self.lock.acquire()
        for actor_id in spawned_actors:
            carla_actor = self.carla_world.get_actor(actor_id)
            self._create_object_from_actor(carla_actor)

        for actor_id in destroyed_actors:
            self._destroy_object(actor_id, delete_actor=False)

        self.lock.release()

    def _create_object_from_actor(self, carla_actor):
        """
        create a object for a given carla actor
        Creates also the object for its parent, if not yet existing
        """
        parent = None
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self._create_object_from_actor(carla_actor.parent)

        parent_id = 0
        if parent is not None:
            parent_id = parent.uid

        name = carla_actor.attributes.get("role_name", "")
        if not name:
            name = str(carla_actor.id)
        obj = self._create_object(carla_actor.id, carla_actor.type_id, name,
                                  parent_id, carla_actor)
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
        self.loginfo("Removed {}(id={})".format(actor.__class__.__name__, actor.uid))

    def _create_object(self, uid, type_id, name, attach_to, carla_actor=None):
        # check that the actor is not already created.
        if carla_actor is not None and carla_actor.id in self.actors:
            return None

        if attach_to != 0:
            if attach_to not in self.actors:
                raise IndexError("Parent object {} not found".format(attach_to))

            parent = self.actors[attach_to]
        else:
            parent = None

        log_actor_creation = True
        if carla_actor.type_id.startswith("vehicle"):
            actor = Vehicle(uid, name, parent, self, carla_actor)
        elif carla_actor.type_id.startswith("sensor.other.lane_invasion"):
            actor = LaneInvasionSensor(uid, name, parent, self, carla_actor)
        else:
            actor = Actor(uid, name, parent, self, carla_actor)
            log_actor_creation = False  # generic actor, do not log creation

        self.actors[actor.uid] = actor
        if log_actor_creation:
            self.loginfo("Created {}(id={})".format(actor.__class__.__name__, actor.uid))

        return actor
