#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla imu sensor
"""

import rospy

from sensor_msgs.msg import Imu

from carla_ros_bridge.sensor import Sensor
import carla_common.transforms as trans


class ImuSensor(Sensor):

    """
    Actor implementation details for imu sensor
    """

    def __init__(self, carla_actor, parent, node, synchronous_mode):
        """
        Constructor

        :param carla_actor : carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(ImuSensor, self).__init__(carla_actor=carla_actor,
                                        parent=parent,
                                        node=node,
                                        synchronous_mode=synchronous_mode,
                                        prefix="imu/" + carla_actor.attributes.get('role_name'))

        self.imu_publisher = rospy.Publisher(self.get_topic_prefix(), Imu, queue_size=10)
        self.listen()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_imu_measurement):
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param carla_imu_measurement: carla imu measurement object
        :type carla_imu_measurement: carla.IMUMeasurement
        """
        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(timestamp=carla_imu_measurement.timestamp)

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        imu_rotation = carla_imu_measurement.transform.rotation

        quat = trans.carla_rotation_to_numpy_quaternion(imu_rotation)
        imu_msg.orientation = trans.numpy_quaternion_to_ros_quaternion(quat)
        self.imu_publisher.publish(imu_msg)
