#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla imu sensor
"""
import math

import os
ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    from tf.transformations import quaternion_from_euler
elif ROS_VERSION == 2:
    from transformations.transformations import quaternion_from_euler
else:
    raise NotImplementedError("Make sure you have a valid ROS_VERSION env variable set.")

from sensor_msgs.msg import Imu

from carla_ros_bridge.sensor import Sensor
import carla_ros_bridge.transforms as trans


class ImuSensor(Sensor):
    """
    Actor implementation details for imu sensor
    """

    def __init__(self, carla_actor, parent, communication, synchronous_mode, sensor_name="IMU"):
        """
        Constructor

        :param carla_actor : carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param communication: communication-handle
        :type communication: carla_ros_bridge.communication
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(ImuSensor,
              self).__init__(carla_actor=carla_actor, parent=parent, communication=communication,
                             synchronous_mode=synchronous_mode,
                             prefix="imu/" + carla_actor.attributes.get('role_name'),
                             sensor_name=sensor_name)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_imu_measurement):
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param carla_imu_measurement: carla imu measurement object
        :type carla_imu_measurement: carla.IMUMeasurement
        """
        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(timestamp=carla_imu_measurement.timestamp)

        imu_msg.angular_velocity.x = carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        imu_rotation = carla_imu_measurement.transform.rotation

        quat = quaternion_from_euler(math.radians(imu_rotation.roll),
                                     math.radians(imu_rotation.pitch),
                                     math.radians(imu_rotation.yaw))
        imu_msg.orientation = trans.numpy_quaternion_to_ros_quaternion(quat)
        self.publish_message(self.get_topic_prefix(), imu_msg)
