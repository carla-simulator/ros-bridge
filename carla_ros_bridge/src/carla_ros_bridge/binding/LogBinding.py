#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Log binding (for testing)
"""

import signal, os
import math
import numpy
from carla_ros_bridge.binding.binding import VehicleClass
import carla_ros_bridge.binding.transforms as trans

from carla import VehicleControl

import time

class LogBinding(object):

    """
    Log binding
    """

    def __init__(self):
        """
        Constructor
        """
        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.publishers = {}
        self.subscribers = {}
        self.vehicle_control_callbacks = {}
        self.vehicle_autopilot_callbacks = {}
        self.parameters = {
            'host': 'localhost',
            'port': 2000,
            'ego_vehicle': {
                'role_name' : ['ego_vehicle', 'hero']
                }
            }
        self.publishers['tf'] = {}
        self.shutdown_callbacks = []
        self.is_shutdown_requested = False
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        self.is_shutdown_requested = True

    def get_parameters(self):
        """
        get parameters
        """
        return self.parameters

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
            self.msgs_to_publish.append((topic, msg))

    def send_msgs(self):
        """
        Function to actually send the collected ROS messages out via the publisher

        :return:
        """
        print("------------------------------------------------------------------------------")

        for topic, msg in self.msgs_to_publish:
            print("<<{}>>: {}".format(topic, msg))
        self.msgs_to_publish = []

        print("<<tf>>: {}".format(self.tf_to_publish))
        self.tf_to_publish = []

    def update_clock(self, carla_timestamp):
        """
        Private function to perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.publish_message('clock', carla_timestamp)

    def publish_gnss(self, topic, carla_gnss_event):
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_event: carla gnss event object
        :type carla_gnss_event: carla.GnssEvent
        """
        self.publish_message(topic, carla_gnss_event)

    def publish_rgb_camera(self, topic, carla_image, attributes):
        """
        Function (override) to transform the received carla image data
        into a ROS image message

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        self.publish_message(topic, attributes, carla_image)

    def publish_semantic_segmentation_camera(self, topic, carla_image, attributes):
        """
        Function (override) to transform the received carla image data
        into a ROS image message

        The segmentation camera raw image is converted to the city scapes palette image
        having 4-channel uint8.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        self.publish_message(topic, attributes, carla_image)

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
        self.publish_message(topic, attributes, carla_image)

    def publish_collision(self, topic, collision_event):
        """
        Function to wrap the collision event into a ros messsage

        :param collision_event: carla collision event object
        :type collision_event: carla.CollisionEvent
        """
        self.publish_message(topic, collision_event)

    def register_vehicle_control_subscriber(self, topic, callback):
        """
        register a subscriber for vehicle control
        """
        print("* registered vehicle control subscriber")

    def vehicle_control_command_updated(self, topic, data):
        """
        callback when a vehicle control command was received
        """
        #not reachable for log binding
        pass

    def register_vehicle_autopilot_subscriber(self, topic, callback):
        """
        register a subscriber for vehicle autopilot setting
        """
        print("* registered vehicle autopilot subscriber")

    def enable_autopilot_updated(self, topic, data):
        """
        callback when a autopilot setting was received
        """
        #not reachable for log binding
        pass

    def publish_lane_invasion(self, topic, lane_invasion_event):
        """
        publish a lane invasion event
        """
        self.publish_message(topic, lane_invasion_event)

    def publish_lidar(self, topic_prefix, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        self.publish_message(topic_prefix, carla_lidar_measurement)

    def publish_ego_vehicle_info(self, topic, type_id, rolename, vehicle_physics):
        """
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'

        :return:
        """
        vehicle_info = {}
        vehicle_info['type_id'] = type_id
        vehicle_info['rolename'] = rolename
        vehicle_info['vehicle_physics'] = vehicle_physics
        self.publish_message(topic, vehicle_info, True)

    def publish_ego_vehicle_status(self, topic, velocity, transform, current_control, acceleration):
        """
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'

        :return:
        """
        odometry = {}
        odometry['velocity'] = velocity
        odometry['transform'] = transform
        odometry['current_control'] = current_control
        odometry['acceleration'] = acceleration
        self.publish_message(topic, odometry)

    def publish_map(self, carla_map):
        """
        publish the map info
        """
        self.publish_message('/carla/map', carla_map, is_latched=True)

    def publish_transform(self, frame_id, transform):
        """
        Helper function to send a ROS tf message of this child

        :return:
        """
        tf_msg = {}
        tf_msg['frame_id'] = frame_id
        tf_msg['transform'] = transform
        self.publish_message('tf', tf_msg)

    @classmethod
    def logdebug(cls, log):
        """
        log in debug-level
        """
        print("=debug= {}".format(log))

    @classmethod
    def loginfo(cls, log):
        """
        log in info-level
        """
        print("=info= {}".format(log))

    @classmethod
    def logwarn(cls, log):
        """
        log in warn-level
        """
        print("=warn= {}".format(log))

    def on_shutdown(self, callback):
        """
        register a callback the gets executed on shutdown
        """
        self.shutdown_callbacks.append(callback)

    def spin(self):
        """
        execute the main loop
        """
        try:
            while not self.is_shutdown_requested:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            for cb in self.shutdown_callbacks:
                cb()

    def is_shutdown(cls):
        """
        check for shutdown
        """
        return cls.is_shutdown_requested

    @classmethod
    def signal_shutdown(cls, text):
        """
        signal shutdown
        """
        cls.is_shutdown_requested = True

    def publish_marker(self, frame_id, bounding_box, color, marker_id):
        """
        publish a marker
        """
        marker = {}
        marker['frame_id'] = frame_id
        marker['bounding_box'] = bounding_box
        marker['color'] = color
        marker['marker_id'] = marker_id
        self.publish_message('/carla/vehicle_marker', marker)

    def publish_objects(self, topic, objects):
        """
        publish objects
        """
        self.publish_message(topic, objects)
