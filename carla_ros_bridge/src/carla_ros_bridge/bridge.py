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
from carla_ros_bridge.binding.ros_binding import RosBinding


class CarlaRosBridge(Parent):

    """
    Carla Ros bridge
    """

    def __init__(self, carla_world, params):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        self.params = params
        super(CarlaRosBridge, self).__init__(
            carla_id=0, carla_world=carla_world, frame_id='/map')

        self.binding = RosBinding()
        self.timestamp_last_run = 0.0
        self.actor_list = []

        # register callback to create/delete actors
        self.update_child_actors_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_update_child_actors)

        # register callback to update actors
        self.update_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_time_tick)

#         self.publishers['/carla/objects'] = rospy.Publisher(
#             '/carla/objects', ObjectArray, queue_size=10)
#         self.object_array = ObjectArray()

        self.map = Map(carla_world=self.carla_world, parent=self, topic='/map')

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
        self.actor_list = []
        self.get_binding().loginfo("Exiting Bridge")
        super(CarlaRosBridge, self).destroy()

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

    def topic_name(self):
        """
        Function (override) to get the topic name of this root entity.

        The topic root '/carla' is returned by this bridge class.

        :return: the final topic name of this
        :rtype: string
        """
        return "/carla"

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
                    self._prepare_msgs()
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
                # cache actor_list once during this update-loop
                self.actor_list = self.carla_world.get_actors()
                self.update_child_actors()
                # actors are only created/deleted around once per second
                time.sleep(1)
                self.update_child_actors_lock.release()

    def _prepare_msgs(self):
        """
        Private function to prepare tf and object message to be sent out

        :return:
        """
        # self.get_binding().publish_message(
        #    '/carla/objects', ObjectSensor.get_filtered_objectarray(self, None))

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
