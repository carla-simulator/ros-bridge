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
try:
    import queue
except ImportError:
    import Queue as queue


import rospy

from carla_ros_bridge.actor import Actor
import carla_common.transforms as trans


class Sensor(Actor):

    """
    Actor implementation details for sensors
    """

    def __init__(self,  # pylint: disable=too-many-arguments
                 carla_actor,
                 parent,
                 node,
                 synchronous_mode,
                 is_event_sensor=False,  # only relevant in synchronous_mode:
                 # if a sensor only delivers data on special events,
                 # do not wait for it. That means you might get data from a
                 # sensor, that belongs to a different frame
                 prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        :param prefix: the topic prefix to be used for this actor
        :type prefix: string
        """
        if prefix is None:
            prefix = 'sensor'
        super(Sensor, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     node=node,
                                     prefix=prefix)

        self.synchronous_mode = synchronous_mode
        self.queue = queue.Queue()
        self.next_data_expected_time = None
        self.sensor_tick_time = None
        self.is_event_sensor = is_event_sensor
        try:
            self.sensor_tick_time = float(carla_actor.attributes["sensor_tick"])
            rospy.logdebug("Sensor tick time is {}".format(self.sensor_tick_time))
        except (KeyError, ValueError):
            self.sensor_tick_time = None

    def listen(self):
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
        super(Sensor, self).destroy()

    def _callback_sensor_data(self, carla_sensor_data):
        """
        Callback function called whenever new sensor data is received

        :param carla_sensor_data: carla sensor data object
        :type carla_sensor_data: carla.SensorData
        """
        if not rospy.is_shutdown():
            if self.synchronous_mode:
                if self.sensor_tick_time:
                    self.next_data_expected_time = carla_sensor_data.timestamp + \
                        float(self.sensor_tick_time)
                self.queue.put(carla_sensor_data)
            else:
                self.publish_transform(self.get_ros_transform(
                    trans.carla_transform_to_ros_transform(carla_sensor_data.transform)))
                self.sensor_data_updated(carla_sensor_data)

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

    def _update_synchronous_event_sensor(self, frame):
        while True:
            try:
                carla_sensor_data = self.queue.get(block=False)
                if carla_sensor_data.frame != frame:
                    rospy.logwarn("{}({}): Received event for frame {}"
                                  " (expected {}). Process it anyways.".format(
                                      self.__class__.__name__,
                                      self.get_id(),
                                      carla_sensor_data.frame,
                                      frame))
                rospy.logdebug("{}({}): process {}".format(
                    self.__class__.__name__, self.get_id(), frame))
                self.publish_transform(
                    self.get_ros_transform(
                        trans.carla_transform_to_ros_transform(carla_sensor_data.transform)))
                self.sensor_data_updated(carla_sensor_data)
            except queue.Empty:
                return

    def _update_synchronous_sensor(self, frame, timestamp):
        while not self.next_data_expected_time or\
            (not self.queue.empty() or
             self.next_data_expected_time and
             self.next_data_expected_time < timestamp):
            while True:
                try:
                    carla_sensor_data = self.queue.get(timeout=1.0)
                    if carla_sensor_data.frame == frame:
                        rospy.logdebug("{}({}): process {}".format(
                            self.__class__.__name__, self.get_id(), frame))
                        self.publish_transform(
                            self.get_ros_transform(
                                trans.carla_transform_to_ros_transform(
                                    carla_sensor_data.transform)))
                        self.sensor_data_updated(carla_sensor_data)
                        return
                    elif carla_sensor_data.frame < frame:
                        rospy.logwarn("{}({}): skipping old frame {}, expected {}".format(
                            self.__class__.__name__,
                            self.get_id(),
                            carla_sensor_data.frame,
                            frame))
                except queue.Empty:
                    if not rospy.is_shutdown():
                        rospy.logwarn("{}({}): Expected Frame {} not received".format(
                            self.__class__.__name__, self.get_id(), frame))
                    return

    def update(self, frame, timestamp):
        if self.synchronous_mode:
            if self.is_event_sensor:
                self._update_synchronous_event_sensor(frame)
            else:
                self._update_synchronous_sensor(frame, timestamp)

        super(Sensor, self).update(frame, timestamp)
