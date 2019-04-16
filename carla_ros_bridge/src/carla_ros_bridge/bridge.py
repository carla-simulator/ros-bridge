#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosbridge class:

Class that handle communication between CARLA and ROS
"""
import threading
import time


from carla_ros_bridge.parent import Parent
from carla_ros_bridge.map import Map
import carla_ros_bridge.object_sensor as ObjectSensor


# these imports have to be at the end to resolve cyclic dependency
from carla_ros_bridge.actor import Actor   
from carla_ros_bridge.spectator import Spectator  
from carla_ros_bridge.sensor import Sensor       
from carla_ros_bridge.traffic import Traffic, TrafficLight   
from carla_ros_bridge.vehicle import Vehicle    
from carla_ros_bridge.lidar import Lidar    
from carla_ros_bridge.gnss import Gnss    
from carla_ros_bridge.ego_vehicle import EgoVehicle  
from carla_ros_bridge.collision_sensor import CollisionSensor  
from carla_ros_bridge.lane_invasion_sensor import LaneInvasionSensor  
from carla_ros_bridge.camera import RgbCamera, DepthCamera, SemanticSegmentationCamera


class TestActor():
    def __init__(self, id, name, parent):
        self.id = name
        self.name = name
        self.parent = parent
        
class CarlaRosBridge(object):

    """
    Carla Ros bridge
    """

    def __init__(self, carla_world, binding):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param binding: binding
        """
        self.actors = {}
        self.carla_world = carla_world
        self.params = binding.get_parameters()

        self.binding = binding
        self.timestamp_last_run = 0.0
        self.actor_list = []

        # register callback to create/delete actors
        self.update_child_actors_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_update_child_actors)

        # register callback to update actors
        self.update_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_time_tick)

        self.map = Map(carla_world=self.carla_world, topic='/map')

    def destroy(self):
        """
        Function (virtual) to destroy this object.

        Lock the update mutex.
        Remove all publisher.
        Finally forward call to super class.

        :return:
        """
        self.update_child_actors_lock.acquire()
        self.update_lock.acquire()
        self.get_binding().signal_shutdown("")
        self.actor_list = []
        self.get_binding().loginfo("Exiting Bridge")

    def get_param(self, key, default=None):
        """
        Function (override) to query global parameters passed from the outside.

        :param key: the key of the parameter
        :type key: string
        :param default: the default value of the parameter to return if key is not found
        :type default: string
        :return: the parameter string
        :rtype: string
        """
        return self.params.get(key, default)

    def _carla_time_tick(self, carla_timestamp):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates.

        After successful locking the update mutex
        (only perform trylock to respect bridge processing time)
        the clock and the children are updated.
        Finally the messages collected to be published are sent out.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not self.get_binding().is_shutdown():
            if self.update_lock.acquire(False):
                if self.timestamp_last_run < carla_timestamp.elapsed_seconds:
                    self.timestamp_last_run = carla_timestamp.elapsed_seconds
                    self.binding.update_clock(carla_timestamp)
                    self.update()
                    self.get_binding().publish_objects('/carla/objects', ObjectSensor.get_filtered_objectarray(self, None))
                    self.binding.send_msgs()
                self.update_lock.release()

    def _carla_update_child_actors(self, _):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates of the actors

        After successful locking the mutex
        (only perform trylock to respect bridge processing time)
        the existing actors are updated.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not self.get_binding().is_shutdown():
            if self.update_child_actors_lock.acquire(False):
                self.update_actors()
                # actors are only created/deleted around once per second
                time.sleep(1)
                self.update_child_actors_lock.release()

    def update_actors(self):
        carla_actors = self.carla_world.get_actors()
        #Add new actors
        for actor in carla_actors:
            if actor.id not in self.actors.keys():
                self.create_actor(actor)

        #create list of carla actors ids
        carla_actor_ids = []
        for actor in carla_actors:
            carla_actor_ids.append(actor.id)

        #remove non-existing actors
        for actor_id in self.actors.keys():
            id_to_delete = None
            if actor_id not in carla_actor_ids:
                id_to_delete = actor_id

            if id_to_delete:
                self.get_binding().logwarn("Remove actor {}".format(id_to_delete))
                del self.actors[id_to_delete]

    def create_actor(self, carla_actor):
        parent = None
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self.create_actor(carla_actor.parent)
        
        actor = None
        if carla_actor.type_id.startswith('traffic'):
            if carla_actor.type_id == "traffic.traffic_light":
                actor = TrafficLight(carla_actor, parent, self.binding)
            else:
                actor = Traffic(carla_actor, parent, self.binding)
        elif carla_actor.type_id.startswith("vehicle"):
            if carla_actor.attributes.get('role_name') in self.get_param('ego_vehicle').get('role_name'):
                actor = EgoVehicle(carla_actor, parent, self.binding)
            else:
                actor = Vehicle(carla_actor, parent, self.binding)
        elif carla_actor.type_id.startswith("sensor"):
            if carla_actor.type_id.startswith("sensor.camera"):
                if carla_actor.type_id.startswith("sensor.camera.rgb"):
                    actor = RgbCamera(carla_actor, parent, self.binding)
                elif carla_actor.type_id.startswith("sensor.camera.depth"):
                    actor = DepthCamera(carla_actor, parent, self.binding)
                elif carla_actor.type_id.startswith("sensor.camera.semantic_segmentation"):
                    actor = SemanticSegmentationCamera(carla_actor, parent, self.binding)
                else:
                    actor = Camera(carla_actor, parent, self.binding)
            if carla_actor.type_id.startswith("sensor.lidar"):
                actor = Lidar(carla_actor, parent, self.binding)
            if carla_actor.type_id.startswith("sensor.other.gnss"):
                actor = Gnss(carla_actor, parent, self.binding)
            if carla_actor.type_id.startswith("sensor.other.collision"):
                actor = CollisionSensor(carla_actor, parent, self.binding)
            if carla_actor.type_id.startswith("sensor.other.lane_invasion"):
                actor = LaneInvasionSensor(carla_actor, parent, self.binding)
            else:
                actor = Sensor(carla_actor, parent, self.binding)
        elif carla_actor.type_id.startswith("spectator"):
            actor = Spectator(carla_actor, parent, self.binding)
        else:
            actor = Actor(carla_actor, parent, self.binding)
        
        self.get_binding().logwarn("Created Actor-{}(id={}, parent_id={},"
                            " type={}, prefix={}, attributes={}".format(
                                actor.__class__.__name__, actor.get_id(),
                                actor.get_parent_id(), carla_actor.type_id,
                                actor.get_topic_prefix(), carla_actor.attributes))
        with self.update_lock:
            self.actors[carla_actor.id] = actor
        return actor

    def run(self):
        """
        Run the bridge functionality.

        Registers on shutdown callback and spins

        :return:
        """
        self.get_binding().on_shutdown(self.on_shutdown)
        self.get_binding().spin()

    def on_shutdown(self):
        """
        Function to be called on shutdown.

        This function is registered as shutdown handler.

        """
        self.get_binding().loginfo("Shutdown requested")
        self.destroy()

    def get_actor_list(self):
        """
        Function (override) to provide actor list

        :return: the list of actors
        :rtype: list
        """
        return self.actor_list

    def get_filtered_objectarray(self, filtered_id):
        """
        get objectarray of available actors, except the one with the filtered id

        :return: objectarray of actors
        :rtype: derived_object_msgs.ObjectArray
        """
        return ObjectSensor.get_filtered_objectarray(self, filtered_id)

    def get_binding(self):
        """
        """
        return self.binding

    def update(self):
        """
        Virtual (non abstract) function to update this object.

        Override this function if the derived class has to perform
        some additional update tasks. But don't forget to forward the update
        call to the super class, ensuring that this concrete function is called.

        The update part of the parent class consists
        of updating the children of this by:

        update the exising children

        :return:
        """
        for actor_id in self.actors.keys():
            try:
                self.actors[actor_id].update()
            except RuntimeError as e:
                self.get_binding().logwarn("Update actor {}({}) failed: {}".format(
                    actor.__class__.__name__, actor_id, e))
                continue