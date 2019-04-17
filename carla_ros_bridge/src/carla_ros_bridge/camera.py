#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle Carla camera sensors
"""
import math
import carla
from carla_ros_bridge.sensor import Sensor

# todo: remove ros-dependency
#import tf


class Camera(Sensor):

    """
    Sensor implementation details for cameras
    """

    def __init__(self, carla_actor, parent, binding, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera'
        super(Camera, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     binding=binding,
                                     topic_prefix=topic_prefix)

        if self.__class__.__name__ == "Camera":
            self.get_binding().logwarn("Created Unsupported Camera Actor"
                                       "(id={}, parent_id={}, type={}, attributes={})".format(
                                           self.get_id(), self.get_parent_id(),
                                           self.carla_actor.type_id, self.carla_actor.attributes))

    def sensor_data_updated(self, carla_image):
        """
        Function (override) to publish camera data

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        raise NotImplementedError(
            "This function has to be re-implemented by derived classes")

    def publish_transform(self):
        """
        The camera transformation has to be altered to look at the same axis
        as the opencv projection in order to get easy depth cloud for RGBD camera

        """
        transform = self.current_sensor_data.transform
        # TODO: find ROS independent solution
        #roll = -math.radians(transform.rotation.roll)
        #pitch = -math.radians(transform.rotation.pitch)
        #yaw = -math.radians(transform.rotation.yaw)
        #quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # quat_swap = tf.transformations.quaternion_from_matrix(
        #    [[0, 0, 1, 0],
        #     [-1, 0, 0, 0],
        #     [0, -1, 0, 0],
        #     [0, 0, 0, 1]])
        #quat = tf.transformations.quaternion_multiply(quat, quat_swap)
        #roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        #transform.rotation.roll = -math.degrees(roll)
        #transform.rotation.pitch = -math.degrees(pitch)
        #transform.rotation.yaw = -math.degrees(yaw)
        self.get_binding().publish_transform(self.get_topic_prefix(), transform)


class RgbCamera(Camera):

    """
    Camera implementation details for rgb camera
    """

    def __init__(self, carla_actor, parent, binding):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        topic_prefix = 'camera/rgb/' + carla_actor.attributes.get('role_name')
        super(RgbCamera, self).__init__(carla_actor=carla_actor,
                                        parent=parent,
                                        binding=binding,
                                        topic_prefix=topic_prefix)

    def sensor_data_updated(self, carla_image):
        """
        Function (override) to publish camera data

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        self.get_binding().publish_rgb_camera(self.get_topic_prefix(),
                                              carla_image, self.carla_actor.attributes)


class DepthCamera(Camera):

    """
    Camera implementation details for depth camera
    """

    def __init__(self, carla_actor, parent, binding):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        topic_prefix = 'camera/depth/' + carla_actor.attributes.get('role_name')
        super(DepthCamera, self).__init__(carla_actor=carla_actor,
                                          parent=parent,
                                          binding=binding,
                                          topic_prefix=topic_prefix)

    def sensor_data_updated(self, carla_image):
        """
        Function (override) to publish camera data

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        self.get_binding().publish_depth_camera(
            self.get_topic_prefix(),
            carla_image,
            self.carla_actor.attributes)


class SemanticSegmentationCamera(Camera):

    """
    Camera implementation details for segmentation camera
    """

    def __init__(self, carla_actor, parent, binding):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        topic_prefix = 'camera/semantic_segmentation/' + carla_actor.attributes.get('role_name')
        super(
            SemanticSegmentationCamera, self).__init__(carla_actor=carla_actor,
                                                       parent=parent,
                                                       binding=binding,
                                                       topic_prefix=topic_prefix)

    def sensor_data_updated(self, carla_image):
        """
        Function (override) to publish camera data

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        carla_image.convert(carla.ColorConverter.CityScapesPalette)
        self.get_binding().publish_semantic_segmentation_camera(
            self.get_topic_prefix(), carla_image, self.carla_actor.attributes)
