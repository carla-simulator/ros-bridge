#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla sensors
"""

from abc import abstractmethod
from threading import Lock

import ros_compatibility as roscomp

from carla_ros_client.actor import Actor


class Sensor(Actor):

    """
    Actor implementation details for sensors
    """

    def __init__(self,  # pylint: disable=too-many-arguments
                 uid,
                 name,
                 parent,
                 node,
                 carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_client.Parent
        :param node: node-handle
        :type node: carla_ros_client.CarlaRosClient
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(Sensor, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     node=node,
                                     carla_actor=carla_actor)

        self._callback_active = Lock()

    def listen(self):
        self.carla_actor.listen(self._callback_sensor_data)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Stop listening to the carla.Sensor actor.
        Finally forward call to super class.

        :return:
        """
        self._callback_active.acquire()
        if self.carla_actor.is_listening:
            self.carla_actor.stop()
        super(Sensor, self).destroy()

    def _callback_sensor_data(self, carla_sensor_data):
        """
        Callback function called whenever new sensor data is received

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        if not self._callback_active.acquire(False):
            # if acquire fails, sensor is currently getting destroyed
            return

        try:
            self.sensor_data_updated(carla_sensor_data)
        except roscomp.exceptions.ROSException:
            if roscomp.ok():
                self.node.logwarn(
                    "Sensor {}: Error while executing sensor_data_updated().".format(self.uid))
        self._callback_active.release()

    @abstractmethod
    def sensor_data_updated(self, carla_sensor_data):
        """
        Pure-virtual function to transform the received carla sensor data
        into a corresponding ROS message

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        raise NotImplementedError(
            "This function has to be implemented by the derived classes")
