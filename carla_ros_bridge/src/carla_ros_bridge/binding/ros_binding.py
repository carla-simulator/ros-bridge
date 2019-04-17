#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
ROS binding
"""

import math
import numpy
import rospy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from carla_ros_bridge.binding.binding import VehicleClass

import carla_ros_bridge.binding.ros_transforms as ros_trans
import carla_ros_bridge.binding.transforms as trans

from cv_bridge import CvBridge

from nav_msgs.msg import Odometry

from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleInfoWheel
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaMapInfo
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent

from sensor_msgs.msg import NavSatFix, CameraInfo


from sensor_msgs.point_cloud2 import create_cloud_xyz32

from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Header, Bool
from derived_object_msgs.msg import Object, ObjectArray
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker


from carla import VehicleControl


class RosBinding(object):

    """
    Carla Ros bridge
    """

    # global cv bridge to convert image between opencv and ros
    cv_bridge = CvBridge()

    def __init__(self):
        """
        Constructor
        """
        rospy.init_node("carla_bridge", anonymous=True)
        self.ros_timestamp = rospy.Time()
        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.publishers = {}
        self.subscribers = {}
        self.vehicle_control_callbacks = {}
        self.vehicle_autopilot_callbacks = {}
        self.camera_info_map = {}
        self.parameters = rospy.get_param('carla')

        self.publishers['clock'] = rospy.Publisher(
            'clock', Clock, queue_size=10)
        self.publishers['tf'] = rospy.Publisher(
            'tf', TFMessage, queue_size=100)

    def get_parameters(self):
        """
        get parameters
        """
        return self.parameters

    def get_msg_header(self, frame_id, timestamp=None):
        """
        Helper function to create a ROS message Header

        :param use_parent_frame: per default the header.frame_id is set
          to the frame of the Child's parent. If this is set to False,
          the Child's own frame is used as basis.
        :return: prefilled Header object
        """
        header = Header()
        header.frame_id = frame_id
        if timestamp:
            header.stamp = timestamp
        else:
            header.stamp = self.ros_timestamp
        return header

    def publish_message(self, topic, msg, is_latched=False):
        """
        Function (override) to publish ROS messages.

        Store the message in a list (waiting for their publication)
        with their associated publisher.
        If required corresponding publishers are created automatically.

        Messages for /tf topics and /carla/objects are handle differently
        in order to publish all transforms, objects in the same message

        :param topic: ROS topic to publish the message on
        :type topic: string
        :param msg: the ROS message to be published
        :type msg: a valid ROS message type
        :return:
        """
        if topic == 'tf':
            # transform are merged in same message
            self.tf_to_publish.append(msg)
        else:
            if topic not in self.publishers:
                self.publishers[topic] = rospy.Publisher(
                    topic, type(msg), queue_size=10, latch=is_latched)
            self.msgs_to_publish.append((self.publishers[topic], msg))

    def send_msgs(self):
        """
        Function to actually send the collected ROS messages out via the publisher

        :return:
        """
        #prepare tf message
        tf_msg = TFMessage(self.tf_to_publish)
        try:
            self.publishers['tf'].publish(tf_msg)
        except rospy.ROSSerializationException as error:
            print(tf_msg)
            rospy.logwarn("Failed to serialize message on publishing: {}".format(error))

        for publisher, msg in self.msgs_to_publish:
            try:
                publisher.publish(msg)
            except rospy.ROSSerializationException as error:
                rospy.logwarn("Failed to serialize message on publishing: {}".format(error))
            except Exception as error:  # pylint: disable=broad-except
                rospy.logwarn("Failed to publish message: {}".format(error))
        self.msgs_to_publish = []
        self.tf_to_publish = []

    def update_clock(self, carla_timestamp):
        """
        Private function to perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.ros_timestamp = rospy.Time.from_sec(
            carla_timestamp.elapsed_seconds)
        self.publish_message('clock', Clock(self.ros_timestamp))


    def publish_gnss(self, topic, carla_gnss_event):
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_event: carla gnss event object
        :type carla_gnss_event: carla.GnssEvent
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = self.get_msg_header(
            topic,
            timestamp=rospy.Time.from_sec(carla_gnss_event.timestamp))
        navsatfix_msg.latitude = carla_gnss_event.latitude
        navsatfix_msg.longitude = carla_gnss_event.longitude
        navsatfix_msg.altitude = carla_gnss_event.altitude
        self.publish_message(topic + "/fix", navsatfix_msg)

    def get_camera_info(self, topic_base, attributes):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        if topic_base not in self.camera_info_map:
            #create non-existing camera info
            camera_info = CameraInfo()
            # store info without header
            camera_info.header = None
            camera_info.width = int(attributes['image_size_x'])
            camera_info.height = int(attributes['image_size_y'])
            camera_info.distortion_model = 'plumb_bob'
            cx = camera_info.width / 2.0
            cy = camera_info.height / 2.0
            fx = camera_info.width / (
                2.0 * math.tan(float(attributes['fov']) * math.pi / 360.0))
            fy = fx
            camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            camera_info.D = [0, 0, 0, 0, 0]
            camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
            camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
            self.camera_info_map[topic_base] = camera_info
        return self.camera_info_map[topic_base]

    def publish_rgb_camera(self, topic, carla_image, attributes):
        """
        Function (override) to transform the received carla image data
        into a ROS image message

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """

        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        img_msg = RosBinding.cv_bridge.cv2_to_imgmsg(carla_image_data_array, encoding='bgra8')
        self.publish_camera(topic, attributes, carla_image, img_msg, "image_color")


    def publish_semantic_segmentation_camera(self, topic, carla_image, attributes):
        """
        Function (override) to transform the received carla image data
        into a ROS image message

        The segmentation camera raw image is converted to the city scapes palette image
        having 4-channel uint8.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        img_msg = RosBinding.cv_bridge.cv2_to_imgmsg(carla_image_data_array, encoding='bgra8')
        self.publish_camera(topic, attributes, carla_image, img_msg, "image_segmentation")

    def publish_depth_camera(self, topic, carla_image, attributes):
        """
        Function (override) to transform the received carla image data
        into a ROS image message

        The depth camera raw image is converted to a linear depth image
        having 1-channel float32.

        :param carla_image: carla image object
        :type carla_image: carla.Image
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
        img_msg = RosBinding.cv_bridge.cv2_to_imgmsg(depth_image, encoding='passthrough')

        self.publish_camera(topic, attributes, carla_image, img_msg, "image_depth")

    def publish_camera(self, topic, attributes, carla_image, image_message, image_topic):
        """
        publish the previously created image, including the camera_info
        """
        cam_info = self.get_camera_info(topic, attributes)
        if (carla_image.height != cam_info.height) or (carla_image.width != cam_info.width):
            rospy.logerr(
                "Camera{} received image not matching configuration".format(topic))

        # the camera data is in respect to the camera's own frame
        image_message.header = self.get_msg_header(
            topic,
            timestamp=rospy.Time.from_sec(carla_image.timestamp))
        cam_info.header = image_message.header

        self.publish_message(topic + '/camera_info', cam_info)
        self.publish_message(topic + '/' + image_topic, image_message)

    def publish_collision(self, topic, collision_event):
        """
        Function to wrap the collision event into a ros messsage

        :param collision_event: carla collision event object
        :type collision_event: carla.CollisionEvent
        """
        collision_msg = CarlaCollisionEvent()
        collision_msg.header = self.get_msg_header(
            "map",
            timestamp=rospy.Time.from_sec(collision_event.timestamp))
        collision_msg.other_actor_id = collision_event.other_actor.id
        collision_msg.normal_impulse.x = collision_event.normal_impulse.x
        collision_msg.normal_impulse.y = collision_event.normal_impulse.y
        collision_msg.normal_impulse.z = collision_event.normal_impulse.z

        self.publish_message(topic, collision_msg)

    def register_vehicle_control_subscriber(self, topic, callback):
        """
        register a subscriber for vehicle control
        """
        self.vehicle_control_callbacks[topic] = callback
        self.subscribers[topic] = rospy.Subscriber(
            topic + "/vehicle_control_cmd",
            CarlaEgoVehicleControl,
            lambda data: self.vehicle_control_command_updated(topic, data))

    def vehicle_control_command_updated(self, topic, data):
        """
        callback when a vehicle control command was received
        """
        vehicle_control = VehicleControl()
        vehicle_control.hand_brake = data.hand_brake
        vehicle_control.brake = data.brake
        vehicle_control.steer = data.steer
        vehicle_control.throttle = data.throttle
        vehicle_control.reverse = data.reverse
        self.vehicle_control_callbacks[topic](vehicle_control)

    def register_vehicle_autopilot_subscriber(self, topic, callback):
        """
        register a subscriber for vehicle autopilot setting
        """
        self.vehicle_autopilot_callbacks[topic] = callback
        self.subscribers[topic] = rospy.Subscriber(
            topic + "/enable_autopilot",
            Bool,
            lambda data: self.enable_autopilot_updated(topic, data))

    def enable_autopilot_updated(self, topic, data):
        """
        callback when a autopilot setting was received
        """
        self.vehicle_autopilot_callbacks[topic](data.data)

    def publish_lane_invasion(self, topic, lane_invasion_event):
        """
        publish a lane invasion event
        """
        lane_invasion_msg = CarlaLaneInvasionEvent()
        lane_invasion_msg.header = self.get_msg_header(
            topic,
            timestamp=rospy.Time.from_sec(lane_invasion_event.timestamp))
        for marking in lane_invasion_event.crossed_lane_markings:
            lane_invasion_msg.crossed_lane_markings.append(marking.type)
        self.publish_message(topic, lane_invasion_msg)

    def publish_lidar(self, topic_prefix, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        header = self.get_msg_header(
            topic_prefix,
            timestamp=rospy.Time.from_sec(carla_lidar_measurement.timestamp))

        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 3), 3))
        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        # we need a copy here, because the data are read only in carla numpy
        # array
        lidar_data = -lidar_data
        # we also need to permute x and y
        lidar_data = lidar_data[..., [1, 0, 2]]
        point_cloud_msg = create_cloud_xyz32(header, lidar_data)
        self.publish_message(topic_prefix + "/point_cloud", point_cloud_msg)

    def publish_ego_vehicle_info(self, topic, type_id, rolename, vehicle_physics):
        """
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'

        :return:
        """
        vehicle_info = CarlaEgoVehicleInfo()
        vehicle_info.type = type_id
        vehicle_info.rolename = rolename

        for wheel in vehicle_physics.wheels:
            wheel_info = CarlaEgoVehicleInfoWheel()
            wheel_info.tire_friction = wheel.tire_friction
            wheel_info.damping_rate = wheel.damping_rate
            wheel_info.steer_angle = math.radians(wheel.steer_angle)
            wheel_info.disable_steering = wheel.disable_steering
            vehicle_info.wheels.append(wheel_info)

        vehicle_info.max_rpm = vehicle_physics.max_rpm
        vehicle_info.max_rpm = vehicle_physics.max_rpm
        vehicle_info.moi = vehicle_physics.moi
        vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
        vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
            vehicle_physics.damping_rate_zero_throttle_clutch_engaged
        vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
            vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
        vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
        vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
        vehicle_info.clutch_strength = vehicle_physics.clutch_strength
        vehicle_info.mass = vehicle_physics.mass
        vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
        vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
        vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
        vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

        self.publish_message(topic, vehicle_info, True)

    def publish_ego_vehicle_status(self, topic, velocity, transform, current_control, acceleration):
        """
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'

        :return:
        """
        ros_pose = ros_trans.carla_transform_to_ros_pose(transform)

        vehicle_status = CarlaEgoVehicleStatus()
        vehicle_status.header.stamp = self.ros_timestamp
        vehicle_status.velocity = trans.get_vector_abs(velocity)
        vehicle_status.acceleration = trans.get_vector_abs(acceleration)
        vehicle_status.orientation = ros_pose.orientation
        vehicle_status.control.throttle = current_control.throttle
        vehicle_status.control.steer = current_control.steer
        vehicle_status.control.brake = current_control.brake
        vehicle_status.control.hand_brake = current_control.hand_brake
        vehicle_status.control.reverse = current_control.reverse
        vehicle_status.control.gear = current_control.gear
        vehicle_status.control.manual_gear_shift = current_control.manual_gear_shift
        self.publish_message(topic + "/vehicle_status", vehicle_status)

        # @todo: do we still need this?
        odometry = Odometry(header=self.get_msg_header(topic))
        #TODOodometry.child_frame_id = self.get_frame_id()
        odometry.pose.pose = ros_pose
        odometry.twist.twist = ros_trans.carla_velocity_to_ros_twist(velocity)

        self.publish_message(topic + "/odometry", odometry)

    def publish_map(self, carla_map):
        """
        publish the map info
        """
        open_drive_msg = CarlaMapInfo(header=self.get_msg_header("map"))
        open_drive_msg.map_name = carla_map.name
        open_drive_msg.opendrive = carla_map.to_opendrive()
        self.publish_message('/carla/map', open_drive_msg, is_latched=True)

    def publish_transform(self, frame_id, transform):
        """
        Helper function to send a ROS tf message of this child

        :return:
        """
        tf_msg = TransformStamped()
        tf_msg.header = self.get_msg_header("map")#TODO: valid
        tf_msg.child_frame_id = frame_id
        tf_msg.transform = ros_trans.carla_transform_to_ros_transform(transform)
        self.publish_message('tf', tf_msg)

    @classmethod
    def logdebug(cls, log):
        """
        log in debug-level
        """
        rospy.logdebug(log)

    @classmethod
    def loginfo(cls, log):
        """
        log in info-level
        """
        rospy.loginfo(log)

    @classmethod
    def logwarn(cls, log):
        """
        log in warn-level
        """
        rospy.logwarn(log)

    @classmethod
    def on_shutdown(cls, callback):
        """
        register a callback the gets executed on shutdown
        """
        rospy.on_shutdown(callback)

    @classmethod
    def spin(cls):
        """
        execute the main loop
        """
        rospy.spin()

    @classmethod
    def is_shutdown(cls):
        """
        check for shutdown
        """
        return rospy.is_shutdown()

    @classmethod
    def signal_shutdown(cls, text):
        """
        signal shutdown
        """
        return rospy.signal_shutdown(text)

    def publish_marker(self, frame_id, bounding_box, color, marker_id):
        """
        publish a marker
        """
        marker = Marker(header=self.get_msg_header(frame_id))
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.3
        marker.id = marker_id
        marker.text = "id = {}".format(marker.id)
        marker.type = Marker.CUBE

        marker.pose = ros_trans.carla_location_to_pose(bounding_box.location)
        marker.scale.x = bounding_box.extent.x * 2.0
        marker.scale.y = bounding_box.extent.y * 2.0
        marker.scale.z = bounding_box.extent.z * 2.0
        self.publish_message('/carla/vehicle_marker', marker)

    def publish_objects(self, topic, objects):
        """
        publish objects
        """
        ros_objects = ObjectArray(header=self.get_msg_header("map"))
        for obj in objects:
            vehicle_object = Object(header=self.get_msg_header("map"))
            vehicle_object.id = obj['id']
            vehicle_object.pose = ros_trans.carla_transform_to_ros_pose(obj['transform'])
            vehicle_object.twist = ros_trans.carla_velocity_to_ros_twist(obj['velocity'])
            vehicle_object.accel = ros_trans.carla_acceleration_to_ros_accel(obj['accel'])
            vehicle_object.shape.type = SolidPrimitive.BOX
            vehicle_object.shape.dimensions.extend([
                obj['bounding_box'].extent.x * 2.0,
                obj['bounding_box'].extent.y * 2.0,
                obj['bounding_box'].extent.z * 2.0])

            # Classification if available in attributes
            if obj['classification'] != VehicleClass.UNKNOWN:
                vehicle_object.object_classified = True
                vehicle_object.classification = obj['classification']
                vehicle_object.classification_certainty = 1.0
                vehicle_object.classification_age = obj['classification_age']
            ros_objects.objects.append(vehicle_object)
        self.publish_message(topic, ros_objects)
