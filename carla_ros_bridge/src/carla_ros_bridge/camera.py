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
from abc import abstractmethod

import math
import numpy

import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

import carla
from carla_ros_bridge.sensor import Sensor
import carla_common.transforms as trans


class Camera(Sensor):

    """
    Sensor implementation details for cameras
    """

    # global cv bridge to convert image between opencv and ros
    cv_bridge = CvBridge()

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):  # pylint: disable=too-many-arguments
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Camera, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     relative_spawn_pose=relative_spawn_pose,
                                     node=node,
                                     carla_actor=carla_actor,
                                     synchronous_mode=synchronous_mode)

        if self.__class__.__name__ == "Camera":
            rospy.logwarn("Created Unsupported Camera Actor"
                          "(id={}, type={}, attributes={})".format(self.get_id(),
                                                                   self.carla_actor.type_id,
                                                                   self.carla_actor.attributes))
        else:
            self._build_camera_info()

        self.camera_info_publisher = rospy.Publisher(self.get_topic_prefix() +
                                                     '/camera_info',
                                                     CameraInfo,
                                                     queue_size=10)

        self.camera_image_publisher = rospy.Publisher(self.get_topic_prefix() + '/' + 'image',
                                                      Image,
                                                      queue_size=10)

    def _build_camera_info(self):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        # store info without header
        camera_info.header = None
        camera_info.width = int(self.carla_actor.attributes['image_size_x'])
        camera_info.height = int(self.carla_actor.attributes['image_size_y'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(self.carla_actor.attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        self._camera_info = camera_info

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_camera_data):
        """
        Function (override) to transform the received carla camera data
        into a ROS image message
        """
        img_msg = self.get_ros_image(carla_camera_data)

        cam_info = self._camera_info
        cam_info.header = img_msg.header

        self.camera_info_publisher.publish(cam_info)
        self.camera_image_publisher.publish(img_msg)

    def get_ros_transform(self, transform=None, frame_id=None, child_frame_id=None):
        """
        Function (override) to modify the tf messages sent by this camera.

        The camera transformation has to be altered to look at the same axis
        as the opencv projection in order to get easy depth cloud for RGBD camera

        :return: the filled tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Camera, self).get_ros_transform(transform, frame_id, child_frame_id)
        rotation = tf_msg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(
            quat)
        return tf_msg

    def get_ros_image(self, carla_camera_data):
        """
        Function to transform the received carla camera data into a ROS image message
        """
        if ((carla_camera_data.height != self._camera_info.height) or
                (carla_camera_data.width != self._camera_info.width)):
            rospy.logerr(
                "Camera{} received image not matching configuration".format(self.get_prefix()))
        image_data_array, encoding = self.get_carla_image_data_array(
            carla_camera_data)
        img_msg = Camera.cv_bridge.cv2_to_imgmsg(image_data_array, encoding=encoding)
        # the camera data is in respect to the camera's own frame
        img_msg.header = self.get_msg_header(timestamp=carla_camera_data.timestamp)

        return img_msg

    @abstractmethod
    def get_carla_image_data_array(self, carla_camera_data):
        """
        Virtual function to convert the carla camera data to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """
        raise NotImplementedError(
            "This function has to be re-implemented by derived classes")


class RgbCamera(Camera):

    """
    Camera implementation details for rgb camera
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(RgbCamera, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        self.listen()

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The RGB camera provides a 4-channel int8 color format (bgra).

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        return carla_image_data_array, 'bgra8'


class DepthCamera(Camera):

    """
    Camera implementation details for depth camera
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(DepthCamera, self).__init__(uid=uid,
                                          name=name,
                                          parent=parent,
                                          relative_spawn_pose=relative_spawn_pose,
                                          node=node,
                                          carla_actor=carla_actor,
                                          synchronous_mode=synchronous_mode)

        self.listen()

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The depth camera raw image is converted to a linear depth image
        having 1-channel float32.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        # color conversion within C++ code is broken, when transforming a
        #  4-channel uint8 color pixel into a 1-channel float32 grayscale pixel
        # therefore, we do it on our own here
        #
        # @todo: After fixing https://github.com/carla-simulator/carla/issues/1041
        # the final code in here should look like:
        #
        # carla_image.convert(carla.ColorConverter.Depth)
        #
        # carla_image_data_array = numpy.ndarray(
        #    shape=(carla_image.height, carla_image.width, 1),
        #    dtype=numpy.float32, buffer=carla_image.raw_data)
        #
        bgra_image = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        # Apply (R + G * 256 + B * 256 * 256) / (256**3 - 1) * 1000
        # according to the documentation:
        # https://carla.readthedocs.io/en/latest/cameras_and_sensors/#camera-depth-map
        scales = numpy.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
        depth_image = numpy.dot(bgra_image, scales).astype(numpy.float32)

        # actually we want encoding '32FC1'
        # which is automatically selected by cv bridge with passthrough
        return depth_image, 'passthrough'


class SemanticSegmentationCamera(Camera):

    """
    Camera implementation details for segmentation camera
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(
            SemanticSegmentationCamera, self).__init__(uid=uid,
                                                       name=name,
                                                       parent=parent,
                                                       relative_spawn_pose=relative_spawn_pose,
                                                       node=node,
                                                       synchronous_mode=synchronous_mode,
                                                       carla_actor=carla_actor)

        self.listen()

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The segmentation camera raw image is converted to the city scapes palette image
        having 4-channel uint8.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        carla_image.convert(carla.ColorConverter.CityScapesPalette)
        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)
        return carla_image_data_array, 'bgra8'


class DVSCamera(Camera):

    """
    Sensor implementation details for dvs cameras
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):  # pylint: disable=too-many-arguments
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(DVSCamera, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        self._dvs_events = None
        self.dvs_camera_publisher = rospy.Publisher(self.get_topic_prefix() +
                                                    '/events',
                                                    PointCloud2,
                                                    queue_size=10)

        self.listen()

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_dvs_event_array):
        """
        Function to transform the received DVS event array into a ROS message

        :param carla_dvs_event_array: dvs event array object
        :type carla_image: carla.DVSEventArray
        """
        super(DVSCamera, self).sensor_data_updated(carla_dvs_event_array)

        header = self.get_msg_header(timestamp=carla_dvs_event_array.timestamp)
        fields = [
            PointField('x', 0, PointField.UINT16, 1),
            PointField('y', 2, PointField.UINT16, 1),
            PointField('t', 4, PointField.FLOAT64, 1),
            PointField('pol', 12, PointField.INT8, 1),
        ]

        dvs_events_msg = create_cloud(header, fields, self._dvs_events.tolist())
        self.dvs_camera_publisher.publish(dvs_events_msg)

    # pylint: disable=arguments-differ
    def get_carla_image_data_array(self, carla_dvs_event_array):
        """
        Function (override) to convert the carla dvs event array to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The carla.DVSEventArray is converted into a 3-channel int8 color image format (bgr).

        :param carla_dvs_event_array: dvs event array object
        :type carla_dvs_event_array: carla.DVSEventArray
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """
        self._dvs_events = numpy.frombuffer(carla_dvs_event_array.raw_data,
                                            dtype=numpy.dtype([
                                                ('x', numpy.uint16),
                                                ('y', numpy.uint16),
                                                ('t', numpy.int64),
                                                ('pol', numpy.bool)
                                            ]))
        carla_image_data_array = numpy.zeros(
            (carla_dvs_event_array.height, carla_dvs_event_array.width, 3),
            dtype=numpy.uint8)
        # Blue is positive, red is negative
        carla_image_data_array[self._dvs_events[:]['y'], self._dvs_events[:]['x'],
                               self._dvs_events[:]['pol'] * 2] = 255

        return carla_image_data_array, 'bgr8'
