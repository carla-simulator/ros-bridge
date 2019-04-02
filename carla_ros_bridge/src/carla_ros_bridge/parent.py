#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Parent factory class for carla.Actor lifecycle handling
"""

from abc import abstractmethod

import threading
import rospy
from std_msgs.msg import Header


class Parent(object):

    """
    Factory class to create actors and manage lifecycle of the children objects
    """

    def __init__(self, carla_id, carla_world, frame_id):
        """
        Constructor

        :param carla_id: unique carla_id of this parent object
            carla_id > 0: carla actor ids (see also carla.Actor)
            carla_id == 0: used by the (root) bridge object
            carla_id == -1: used by the map object
        :type carla_id: int64
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param frame_id: the ROS tf frame id of this object
        :type frame_id: string
        """
        self.carla_id = carla_id
        self.carla_world = carla_world
        self.frame_id = frame_id
        self.child_actors = {}
        self.dead_child_actors = []
        self.new_child_actors = {}
        self.update_child_actor_list_lock = threading.Lock()

    def destroy(self):
        """
        Function (virtual) to destroy this object.

        Recursively calls destroy() of all children.
        Remove the reference to the carla.World object.
        Finally remove the references to the children object.

        :return:
        """
        with self.update_child_actor_list_lock:
            for dummy_actor_id, actor in self.child_actors.iteritems():
                actor.destroy()
                actor = None
            self.child_actors.clear()
        self.carla_world = None

    def get_frame_id(self):
        """
        Getter for the frame id of this.

        :return: the ROS tf frame id of this object
        :rtype: string
        """
        return self.frame_id

    def get_id(self):
        """
        Getter for the carla_id of this.

        :return: unique carla_id of this parent object
        :rtype: int64
        """
        return self.carla_id

    def get_carla_world(self):
        """
        Getter for the carla world object of this.

        :return: the carla world
        :rtype: carla.World
        """
        return self.carla_world

    def get_new_child_actors(self):
        """
        Private function to get children actors of this parent.

        :return:
        """
        for actor in self.get_actor_list():
            if ((actor.parent and actor.parent.id == self.carla_id)
                    or (actor.parent is None and self.carla_id == 0)):
                if actor.id not in self.child_actors:
                    if self.get_param("challenge_mode"):
                        # in challenge mode only the ego vehicle and its sensors are created
                        if actor.type_id.startswith("vehicle") and \
                                (actor.attributes.get('role_name') ==
                                 self.get_param('ego_vehicle').get('role_name')):
                            self.new_child_actors[actor.id] = EgoVehicle.create_actor(
                                carla_actor=actor, parent=self)
                        elif actor.type_id.startswith("sensor"):
                            self.new_child_actors[actor.id] = Sensor.create_actor(
                                carla_actor=actor, parent=self)
                    else:
                        if actor.type_id.startswith('traffic'):
                            self.new_child_actors[actor.id] = Traffic.create_actor(
                                carla_actor=actor, parent=self)
                        elif actor.type_id.startswith("vehicle"):
                            if actor.attributes.get('role_name') in \
                                    self.get_param('ego_vehicle').get('role_name'):
                                self.new_child_actors[actor.id] = EgoVehicle.create_actor(
                                    carla_actor=actor, parent=self)
                            else:
                                self.new_child_actors[actor.id] = Vehicle.create_actor(
                                    carla_actor=actor, parent=self)
                        elif actor.type_id.startswith("sensor"):
                            self.new_child_actors[actor.id] = Sensor.create_actor(
                                carla_actor=actor, parent=self)
                        elif actor.type_id.startswith("spectator"):
                            self.new_child_actors[actor.id] = Spectator(
                                carla_actor=actor, parent=self)
                        else:
                            self.new_child_actors[actor.id] = Actor(
                                carla_actor=actor, parent=self)

    def get_dead_child_actors(self):
        """
        Private function to detect non existing children actors

        :return:
        """
        for child_actor_id, child_actor in self.child_actors.iteritems():
            if not child_actor.carla_actor.is_alive:
                rospy.loginfo(
                    "Detected non alive child Actor(id={})".format(child_actor_id))
                self.dead_child_actors.append(child_actor)
            else:
                found_actor = False
                for actor in self.carla_world.get_actors():
                    if actor.id == child_actor_id:
                        found_actor = True
                        break
                if not found_actor:
                    rospy.loginfo(
                        "Detected not existing child Actor(id={})".format(child_actor_id))
                    self.dead_child_actors.append(child_actor)

    def update_child_actors(self):
        """
        Virtual (non abstract) function to update the children of this object.

        The update part of the parent class consists
        of updating the children of this by:

        create new child actors
        destroy dead children
        update the exising children

        :return:
        """
        self.get_new_child_actors()
        self.get_dead_child_actors()

        if len(self.dead_child_actors) > 0 or len(self.new_child_actors) > 0:
            with self.update_child_actor_list_lock:
                for actor in self.dead_child_actors:
                    actor_id = actor.carla_actor.id
                    self.child_actors[actor_id].destroy()
                    del self.child_actors[actor_id]
                self.dead_child_actors = []

                for actor_id, actor in self.new_child_actors.iteritems():
                    self.child_actors[actor_id] = actor
                self.new_child_actors.clear()

        for dummy_actor_id, actor in self.child_actors.iteritems():
            actor.update_child_actors()

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
        with self.update_child_actor_list_lock:
            for actor_id, actor in self.child_actors.iteritems():
                try:
                    actor.update()
                except RuntimeError as e:
                    rospy.logwarn("Update actor {}({}) failed: {}".format(
                        actor.__class__.__name__, actor_id, e))
                    continue

    def get_msg_header(self):
        """
        Helper function to create a ROS message Header

        :return: prefilled Header object
        """
        header = Header()
        header.stamp = self.get_current_ros_time()
        header.frame_id = self.get_frame_id()
        return header

    @abstractmethod
    def get_current_ros_time(self):
        """
        Pure virtual function to query the current ROS time from
        the carla_ros_bridge.CarlaRosBridge parent root.

        Is intended to be implemented by the directly derived classes:
        carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!

        :return: The latest received ROS time of the bridge
        :rtype: rospy.Time
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def publish_ros_message(self, topic, msg, is_latched=False):
        """
        Pure virtual function to publish ROS messages via
        the carla_ros_bridge.CarlaRosBridge parent root.

        Is intended to be implemented by the directly derived classes:
        carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!

        :param topic: ROS topic to publish the message on
        :type topic: string
        :param msg: the ROS message to be published
        :type msg: a valid ROS message type
        :return:
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def get_param(self, key, default=None):
        """
        Pure virtual function to query global parameters passed from the outside.

        Is intended to be implemented by the directly derived classes:
        carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!

        :param key: the key of the parameter
        :type key: string
        :param default: the default value of the parameter to return if key is not found
        :type default: string
        :return: the parameter string
        :rtype: string
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def topic_name(self):
        """
        Pure virtual function to get the topic name of the current entity.

        Is intended to be implemented by the directly derived classes:
        carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge.
        Be aware: Its not intended that classes further down in the class hierarchy override this!

        :return: the final topic name of this
        :rtype: string
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def get_actor_list(self):
        """
        Pure virtual function to get the list of actors

        :return: the list of actors
        :rtype: list
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")

    @abstractmethod
    def get_filtered_objectarray(self, filtered_id):
        """
        Pure virtual function to get objectarray of available actors, except
        the one with the filtered id

        :return: objectarray of actors
        :rtype: derived_object_msgs.ObjectArray
        """
        raise NotImplementedError(
            "This function is re-implemented by"
            "carla_ros_bridge.Child and carla_ros_bridge.CarlaRosBridge"
            "If this error becomes visible the class hierarchy is somehow broken")


# these imports have to be at the end to resolve cyclic dependency
from carla_ros_bridge.actor import Actor         # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.spectator import Spectator  # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.sensor import Sensor       # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.traffic import Traffic     # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.vehicle import Vehicle     # noqa, pylint: disable=wrong-import-position
from carla_ros_bridge.ego_vehicle import EgoVehicle  # noqa, pylint: disable=wrong-import-position
