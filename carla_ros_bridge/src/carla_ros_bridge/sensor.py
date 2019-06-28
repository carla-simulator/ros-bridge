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

import threading

import rospy
import carla_ros_bridge.transforms as trans


from carla_ros_bridge.actor import Actor


class Sensor(Actor):

    """
    Actor implementation details for sensors
    """

    def __init__(self, carla_actor, parent, communication, prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        if prefix is None:
            prefix = 'sensor'
        super(Sensor, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     communication=communication,
                                     prefix=prefix)

        self.current_sensor_data = None
        self.update_lock = threading.Lock()
        if self.__class__.__name__ != "Sensor":
            self.carla_actor.listen(self._callback_sensor_data)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Stop listening to the carla.Sensor actor.
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroy Sensor(id={})".format(self.get_id()))
        if self.carla_actor.is_listening:
            self.carla_actor.stop()
        if self.update_lock.acquire():
            self.current_sensor_data = None
        super(Sensor, self).destroy()

    def _callback_sensor_data(self, carla_sensor_data):
        """
        Callback function called whenever new sensor data is received

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        if not rospy.is_shutdown():
            if self.update_lock.acquire(False):
                self.current_sensor_data = carla_sensor_data
                self.publish_transform()
                self.sensor_data_updated(carla_sensor_data)
                self.update_lock.release()

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

    def get_current_ros_transform(self):

        tf_msg = super(Sensor, self).get_current_ros_transform()
        # sensor data has its own transform
        tf_msg.transform = trans.carla_transform_to_ros_transform(
            self.current_sensor_data.transform)
        return tf_msg
