#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions to convert transforms from carla to ROS coordinate system
"""

import math
import numpy

import tf
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel


def carla_location_to_numpy_vector(carla_location):
    """
    Convert a carla location to a ROS vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_location.x,
        -carla_location.y,
        carla_location.z
    ])


def carla_location_to_ros_vector3(carla_location):
    """
    Convert a carla location to a ROS vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS vector3
    :rtype: geometry_msgs.msg.Vector3
    """
    ros_translation = Vector3()
    ros_translation.x = carla_location.x
    ros_translation.y = -carla_location.y
    ros_translation.z = carla_location.z

    return ros_translation


def carla_location_to_ros_point(carla_location):
    """
    Convert a carla location to a ROS point

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS point
    :rtype: geometry_msgs.msg.Point
    """
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = -carla_location.y
    ros_point.z = carla_location.z

    return ros_point


def numpy_quaternion_to_ros_quaternion(numpy_quaternion):
    """
    Convert a quaternion from transforms to a ROS msg quaternion

    :param numpy_quaternion: a numpy quaternion
    :type numpy_quaternion: numpy.array
    :return: a ROS quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    ros_quaternion = Quaternion()
    ros_quaternion.x = numpy_quaternion[0]
    ros_quaternion.y = numpy_quaternion[1]
    ros_quaternion.z = numpy_quaternion[2]
    ros_quaternion.w = numpy_quaternion[3]
    return ros_quaternion


def carla_rotation_to_RPY(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)


def carla_rotation_to_numpy_quaternion(carla_rotation):
    """
    Convert a carla rotation to a numpy quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 4 elements (quaternion)
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    return quat


def carla_rotation_to_ros_quaternion(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a ROS quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    quat = carla_rotation_to_numpy_quaternion(carla_rotation)
    ros_quaternion = numpy_quaternion_to_ros_quaternion(quat)

    return ros_quaternion


def carla_rotation_to_numpy_rotation_matrix(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3x3 elements
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    numpy_array = tf.transformations.euler_matrix(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix


def carla_rotation_to_directional_numpy_vector(carla_rotation):
    """
    Convert a carla rotation (as orientation) into a numpy directional vector

    ros_quaternion = np_quaternion_to_ros_quaternion(quat)
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3 elements as directional vector
        representation of the orientation
    :rtype: numpy.array
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    directional_vector = numpy.array([1, 0, 0])
    rotated_directional_vector = rotation_matrix.dot(directional_vector)
    return rotated_directional_vector


def carla_vector_to_ros_vector_rotated(carla_vector, carla_rotation):
    """
    Rotate carla vector, return it as ros vector

    :param carla_vector: the carla vector
    :type carla_vector: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: rotated ros vector
    :rtype: Vector3
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    tmp_array = rotation_matrix.dot(numpy.array([carla_vector.x, carla_vector.y, carla_vector.z]))
    ros_vector = Vector3()
    ros_vector.x = tmp_array[0]
    ros_vector.y = -tmp_array[1]
    ros_vector.z = tmp_array[2]
    return ros_vector


def carla_velocity_to_ros_twist(carla_linear_velocity, carla_angular_velocity, carla_rotation=None):
    """
    Convert a carla velocity to a ROS twist

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :param carla_angular_velocity: the carla angular velocity
    :type carla_angular_velocity: carla.Vector3D
    :param carla_rotation: the carla rotation. If None, no rotation is executed
    :type carla_rotation: carla.Rotation
    :return: a ROS twist (with rotation)
    :rtype: geometry_msgs.msg.Twist
    """
    ros_twist = Twist()
    if carla_rotation:
        ros_twist.linear = carla_vector_to_ros_vector_rotated(carla_linear_velocity, carla_rotation)
    else:
        ros_twist.linear = carla_location_to_ros_vector3(carla_linear_velocity)
    ros_twist.angular.x = math.radians(carla_angular_velocity.x)
    ros_twist.angular.y = -math.radians(carla_angular_velocity.y)
    ros_twist.angular.z = -math.radians(carla_angular_velocity.z)
    return ros_twist


def carla_velocity_to_numpy_vector(carla_velocity):
    """
    Convert a carla velocity to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_velocity.x,
        -carla_velocity.y,
        carla_velocity.z
    ])


def carla_acceleration_to_ros_accel(carla_acceleration):
    """
    Convert a carla acceleration to a ROS accel

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)
    The angular accelerations remain zero.

    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :return: a ROS accel
    :rtype: geometry_msgs.msg.Accel
    """
    ros_accel = Accel()
    ros_accel.linear.x = carla_acceleration.x
    ros_accel.linear.y = -carla_acceleration.y
    ros_accel.linear.z = carla_acceleration.z

    return ros_accel


def carla_transform_to_ros_transform(carla_transform):
    """
    Convert a carla transform to a ROS transform

    See carla_location_to_ros_vector3() and carla_rotation_to_ros_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a ROS transform
    :rtype: geometry_msgs.msg.Transform
    """
    ros_transform = Transform()

    ros_transform.translation = carla_location_to_ros_vector3(
        carla_transform.location)
    ros_transform.rotation = carla_rotation_to_ros_quaternion(
        carla_transform.rotation)

    return ros_transform


def carla_transform_to_ros_pose(carla_transform):
    """
    Convert a carla transform to a ROS pose

    See carla_location_to_ros_point() and carla_rotation_to_ros_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a ROS pose
    :rtype: geometry_msgs.msg.Pose
    """
    ros_pose = Pose()

    ros_pose.position = carla_location_to_ros_point(
        carla_transform.location)
    ros_pose.orientation = carla_rotation_to_ros_quaternion(
        carla_transform.rotation)

    return ros_pose


def carla_location_to_pose(carla_location):
    """
    Convert a carla location to a ROS pose

    See carla_location_to_ros_point() for details.
    pose quaternion remains zero.

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS pose
    :rtype: geometry_msgs.msg.Pose
    """
    ros_pose = Pose()
    ros_pose.position = carla_location_to_ros_point(carla_location)
    return ros_pose
