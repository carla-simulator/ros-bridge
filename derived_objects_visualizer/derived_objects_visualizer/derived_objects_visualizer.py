import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from message_filters import Subscriber
from derived_object_msgs.msg import ObjectWithCovarianceArray, ObjectWithCovariance
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class DerivedObjectsVisualizer(Node):
    def __init__(self):
        super().__init__('derived_objects_visualizer')

        self.declare_parameter("objects_topic", value="/carla/world/objects_with_covariance")

        self.object_subscription = Subscriber(self, ObjectWithCovarianceArray, self.get_parameter(
            "objects_topic").get_parameter_value().string_value, qos_profile=self.get_qos_objects_lidar())
        self.object_subscription.registerCallback(self.object_callback)

        self.object_publisher = self.create_publisher(MarkerArray, self.get_parameter(
            "objects_topic").get_parameter_value().string_value + '/marker_objects', qos_profile=self.get_qos_marker_objects())

    def get_qos_objects_lidar(self):
        # volatile, best_effort should be ok for sensor data input
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        return qos

    def get_qos_marker_objects(self):
        # most compatible publisher durability: transient local
        # most compatible publisher reliabilty: reliable
        # queue size per default: 1
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        return qos

    def get_object_color(self, object):
        color = ColorRGBA()
        color.a = 1.0
        # white
        color.r = 1.0
        color.g = 1.0
        color.b = 1.0
        if object.classification == ObjectWithCovariance.CLASSIFICATION_PEDESTRIAN:
            # yellow
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        elif object.classification == ObjectWithCovariance.CLASSIFICATION_BIKE:
            # orange
            color.r = 1.0
            color.g = 0.65
            color.b = 0.0
        elif (object.classification == ObjectWithCovariance.CLASSIFICATION_CAR) or (object.classification == ObjectWithCovariance.CLASSIFICATION_TRUCK) or (object.classification == ObjectWithCovariance.CLASSIFICATION_MOTORCYCLE) or (object.classification == ObjectWithCovariance.CLASSIFICATION_OTHER_VEHICLE):
            # blue
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0

        return color

    def object_callback(self, msg):
        marker_array = MarkerArray()

        for object in msg.objects:
            # print(object)
            marker = Marker()
            marker.header = object.header
            marker.ns = "Objects"
            marker.type = Marker.CUBE
            marker.lifetime = rclpy.duration.Duration(seconds=1.).to_msg()

            marker.id = object.id
            marker.pose.position.x = object.pose.pose.position.x
            marker.pose.position.y = object.pose.pose.position.y
            marker.pose.position.z = object.pose.pose.position.z
            marker.pose.orientation.x = object.pose.pose.orientation.x
            marker.pose.orientation.y = object.pose.pose.orientation.y
            marker.pose.orientation.z = object.pose.pose.orientation.z
            marker.pose.orientation.w = object.pose.pose.orientation.w

            marker.scale.x = object.shape.dimensions[0]
            marker.scale.y = object.shape.dimensions[1]
            marker.scale.z = object.shape.dimensions[2]

            marker.color = self.get_object_color(object)

            marker.action = Marker.ADD

            if len(object.polygon.points) > 0:
                print('polygon')

            marker_array.markers.append(marker)

            # Heading
            marker = Marker()
            marker.header = object.header
            marker.ns = "Heading"
            marker.type = Marker.ARROW
            marker.lifetime = rclpy.duration.Duration(seconds=1.).to_msg()

            marker.id = object.id
            marker.pose.position.x = object.pose.pose.position.x
            marker.pose.position.y = object.pose.pose.position.y
            marker.pose.position.z = object.pose.pose.position.z
            marker.pose.orientation.x = object.pose.pose.orientation.x
            marker.pose.orientation.y = object.pose.pose.orientation.y
            marker.pose.orientation.z = object.pose.pose.orientation.z
            marker.pose.orientation.w = object.pose.pose.orientation.w

            marker.scale.x = max(object.shape.dimensions[0], 1.0)
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color = self.get_object_color(object)

            marker.action = Marker.ADD

            marker_array.markers.append(marker)

            # Speed
            marker = Marker()
            marker.header = object.header
            marker.ns = "speed"

            marker.type = Marker.LINE_STRIP
            marker.lifetime = rclpy.duration.Duration(seconds=1.).to_msg()
            marker.pose.position.x = 0.
            marker.pose.position.y = 0.
            marker.pose.position.z = 0.
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 0.
            marker.pose.orientation.w = 1.

            marker.color.g = 1.0
            marker.color.a = 1.0
            marker.action = Marker.ADD

            marker.id = object.id

            marker.scale.x = 0.2
            marker.scale.y = 0.
            marker.scale.z = 0.

            point = Point()
            point = object.pose.pose.position
            marker.points.append(point)

            point2 = Point()
            point2.x = point.x + object.twist.twist.linear.x*10.0
            point2.y = point.y + object.twist.twist.linear.y*10.0
            point2.z = point.z

            marker.points.append(point2)
            marker_array.markers.append(marker)

            marker = Marker()
            marker.header = object.header
            marker.ns = "IDs"
            marker.lifetime = rclpy.duration.Duration(seconds=1.).to_msg()
            marker.type = Marker.TEXT_VIEW_FACING

            marker.id = object.id
            marker.pose.position.x = object.pose.pose.position.x
            marker.pose.position.y = object.pose.pose.position.y
            marker.pose.position.z = object.pose.pose.position.z + 2.0

            marker.scale.z = 1.0
            marker.color.g = 1.0
            marker.color.a = 1.0

            speed = round(math.sqrt(object.twist.twist.linear.x**2 + object.twist.twist.linear.y**2),2)
            marker.text = str(object.id) + "  -  " + str(speed) + "m/s"
            marker.action = Marker.ADD

            marker_array.markers.append(marker)

        self.object_publisher.publish(marker_array)

    def before_shutdown(self):
        pass
