from copy import deepcopy
import math

import numpy as np
import rclpy
from enum import IntEnum
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from message_filters import Subscriber
from derived_object_msgs.msg import ObjectArray, Object
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaActorList, CarlaActorInfo
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


# Define CityObjectLabel enum for environment object filtering based on CityObjectLabel values in the CarlaActorList message
class CityObjectLabel(IntEnum):
    NONE = CarlaActorInfo.CITYOBJECTLABEL_NONE
    # cityscape labels
    ROADS = CarlaActorInfo.CITYOBJECTLABEL_ROADS
    SIDEWALKS = CarlaActorInfo.CITYOBJECTLABEL_SIDEWALKS
    BUILDINGS = CarlaActorInfo.CITYOBJECTLABEL_BUILDINGS
    WALLS = CarlaActorInfo.CITYOBJECTLABEL_WALLS
    FENCES = CarlaActorInfo.CITYOBJECTLABEL_FENCES
    POLES = CarlaActorInfo.CITYOBJECTLABEL_POLES
    TRAFFIC_LIGHT = CarlaActorInfo.CITYOBJECTLABEL_TRAFFICLIGHT
    TRAFFIC_SIGNS = CarlaActorInfo.CITYOBJECTLABEL_TRAFFICSIGNS
    VEGETATION = CarlaActorInfo.CITYOBJECTLABEL_VEGETATION
    TERRAIN = CarlaActorInfo.CITYOBJECTLABEL_TERRAIN
    SKY = CarlaActorInfo.CITYOBJECTLABEL_SKY
    PEDESTRIANS = CarlaActorInfo.CITYOBJECTLABEL_PEDESTRIANS
    RIDER = CarlaActorInfo.CITYOBJECTLABEL_RIDER
    CAR = CarlaActorInfo.CITYOBJECTLABEL_CAR
    TRUCK = CarlaActorInfo.CITYOBJECTLABEL_TRUCK
    BUS = CarlaActorInfo.CITYOBJECTLABEL_BUS
    TRAIN = CarlaActorInfo.CITYOBJECTLABEL_TRAIN
    MOTORCYCLE = CarlaActorInfo.CITYOBJECTLABEL_MOTORCYCLE
    BICYCLE = CarlaActorInfo.CITYOBJECTLABEL_BICYCLE
    # additional labels for Carla environment objects
    STATIC = CarlaActorInfo.CITYOBJECTLABEL_STATIC
    DYNAMIC = CarlaActorInfo.CITYOBJECTLABEL_DYNAMIC
    OTHER = CarlaActorInfo.CITYOBJECTLABEL_OTHER
    WATER = CarlaActorInfo.CITYOBJECTLABEL_WATER
    ROADLINES = CarlaActorInfo.CITYOBJECTLABEL_ROADLINES
    GROUND = CarlaActorInfo.CITYOBJECTLABEL_GROUND
    BRIDGE = CarlaActorInfo.CITYOBJECTLABEL_BRIDGE
    RAILTRACK = CarlaActorInfo.CITYOBJECTLABEL_RAILTRACK
    GUARDRAIL = CarlaActorInfo.CITYOBJECTLABEL_GUARDRAIL
    ANY = CarlaActorInfo.CITYOBJECTLABEL_ANY

class DerivedObjectsVisualizer(Node):
    def __init__(self):
        super().__init__('derived_objects_visualizer')

        self.declare_parameter("objects_topic", value="/carla/objects_with_covariance")
        self.declare_parameter("lifetime_seconds", value=1.0)
        self.lifetime = self.get_parameter("lifetime_seconds").get_parameter_value().double_value

        # allow filtering of the ego vehicle by its id, which is published in the CarlaEgoVehicleInfo message.
        # If the ego vehicle topic is not set, the ego vehicle will not be filtered and visualized like the other objects.
        self.declare_parameter("ego_vehicle_vehicle_info_topic", value="")
        self.ego_vehicle_vehicle_info_topic = self.get_parameter(
            "ego_vehicle_vehicle_info_topic").get_parameter_value().string_value

        self.declare_parameter("actor_list_topic", value="")
        self.actor_list_topic = self.get_parameter("actor_list_topic").get_parameter_value().string_value
        self.wait_for_actor_list = False
        self.actor_list_subscription = None
        if self.actor_list_topic != "":
            self.wait_for_actor_list = True
            self.actor_list_subscription = Subscriber(
                self, CarlaActorList, self.actor_list_topic,
                qos_profile=self.get_qos_objects())
            self.actor_list_subscription.registerCallback(self.actor_list_callback)

        self.declare_parameter("environment_objects_labels_filter_positive", value="")
        self.environment_objects_labels_filter_positive = []
        labels_str = self.get_parameter("environment_objects_labels_filter_positive").get_parameter_value().string_value
        if labels_str != "":
            labels_list = [label.strip() for label in labels_str.split(",")]
            for label in labels_list:
                try:
                    self.environment_objects_labels_filter_positive.append(CityObjectLabel[label.upper()])
                except KeyError:
                    self.get_logger().warn(f"Invalid CityObjectLabel '{label}' in environment_objects_labels_filter_positive parameter. Skipping this label.")

        self.declare_parameter("environment_objects_labels_filter_negative", value="")
        self.environment_objects_labels_filter_negative = []
        self.environment_objects_labels_filter_negative.append(CityObjectLabel.ANY)
        self.environment_objects_labels_filter_negative.append(CityObjectLabel.NONE)
        self.environment_objects_labels_filter_negative.append(CityObjectLabel.SKY)
        labels_str = self.get_parameter("environment_objects_labels_filter_negative").get_parameter_value().string_value
        if labels_str != "":
            labels_list = [label.strip() for label in labels_str.split(",")]
            for label in labels_list:
                try:
                    self.environment_objects_labels_filter_negative.append(CityObjectLabel[label.upper()])
                except KeyError:
                    self.get_logger().warn(f"Invalid CityObjectLabel '{label}' in environment_objects_labels_filter_negative parameter. Skipping this label.")

        self.objects = None
        self.object_to_label_map = {}
        self.object_to_id_map = {}

        self.ego_vehicle_id = None
        if self.ego_vehicle_vehicle_info_topic != "":
            self.ego_vehicle_info_subscription = Subscriber(
                self, CarlaEgoVehicleInfo,
                self.ego_vehicle_vehicle_info_topic,
                qos_profile=self.get_qos_ego_vehicle_info())
            self.ego_vehicle_info_subscription.registerCallback(self.ego_vehicle_info_callback)

        self.object_subscription = Subscriber(self, ObjectArray, self.get_parameter(
            "objects_topic").get_parameter_value().string_value, qos_profile=self.get_qos_objects())
        self.object_subscription.registerCallback(self.object_callback)

        self.object_publisher = self.create_publisher(MarkerArray, self.get_parameter(
            "objects_topic").get_parameter_value().string_value + '/marker_objects', qos_profile=self.get_qos_marker_objects())

    def get_qos_ego_vehicle_info(self):
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        return qos

    def get_qos_objects(self):
        # transient local to support also environment objects, traffic_lights, etc. which are published by carla with transient local durability
        # reliable to ensure that we don't miss the message
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        return qos

    def get_qos_marker_objects(self):
        # most compatible publisher durability: transient local
        # most compatible publisher reliabilty: reliable
        # queue size per default: 1
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        return qos

    def get_object_color(self, object, city_object_label=None):
        color = ColorRGBA()
        color.a = 1.0
        # white
        color.r = 1.0
        color.g = 1.0
        color.b = 1.0
        if object.classification == Object.CLASSIFICATION_PEDESTRIAN:
            # yellow
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        elif object.classification == Object.CLASSIFICATION_BIKE:
            # orange
            color.r = 1.0
            color.g = 0.65
            color.b = 0.0
        elif (object.classification == Object.CLASSIFICATION_CAR) or (object.classification == Object.CLASSIFICATION_TRUCK) or (object.classification == Object.CLASSIFICATION_MOTORCYCLE) or (object.classification == Object.CLASSIFICATION_OTHER_VEHICLE):
            # blue
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
        elif (object.classification == Object.CLASSIFICATION_SIGN) or (object.classification == Object.CLASSIFICATION_BARRIER):
            # red
            color.a = 0.2
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        elif city_object_label is not None:
            color.a = 0.2
            if city_object_label == CityObjectLabel.ROADS:
                # grey
                color.r = 0.5
                color.g = 0.5
                color.b = 0.5
            elif city_object_label == CityObjectLabel.SIDEWALKS:
                # light grey
                color.r = 0.7
                color.g = 0.7
                color.b = 0.7
            elif city_object_label == CityObjectLabel.BUILDINGS:
                # white
                color.r = 1.0
                color.g = 1.0
                color.b = 1.0
            elif city_object_label == CityObjectLabel.WALLS:
                # dark grey
                color.r = 0.3
                color.g = 0.3
                color.b = 0.3
            elif city_object_label == CityObjectLabel.FENCES:
                # brown
                color.r = 0.6
                color.g = 0.3
                color.b = 0.0
            elif city_object_label == CityObjectLabel.POLES:
                # purple
                color.r = 0.5
                color.g = 0.0
                color.b = 0.5
            elif city_object_label == CityObjectLabel.TRAFFIC_LIGHT:
                # red
                color.r = 1.0
                color.g = 0.0
                color.b = 0.0
            elif city_object_label == CityObjectLabel.TRAFFIC_SIGNS:
                # red
                color.r = 1.0
                color.g = 0.0
                color.b = 0.0
            elif city_object_label == CityObjectLabel.VEGETATION:
                # green
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
            elif city_object_label == CityObjectLabel.TERRAIN:
                # brown
                color.r = 0.6
                color.g = 0.3
                color.b = 0.0
            elif city_object_label == CityObjectLabel.WATER:
                # blue
                color.r = 0.0
                color.g = 0.0
                color.b = 1.0
            elif city_object_label == CityObjectLabel.ROADLINES:
                # yellow
                color.r = 1.0
                color.g = 1.0
                color.b = 0.0
            elif city_object_label == CityObjectLabel.GROUND:
                # brown
                color.r = 0.6
                color.g = 0.3
                color.b = 0.0
            elif city_object_label == CityObjectLabel.BRIDGE:
                # grey
                color.r = 0.5
                color.g = 0.5
                color.b = 0.5
            elif city_object_label == CityObjectLabel.RAILTRACK:
                # dark grey
                color.r = 0.3
                color.g = 0.3
                color.b = 0.3
            elif city_object_label == CityObjectLabel.GUARDRAIL:
                # light grey
                color.r = 0.7
                color.g = 0.7
                color.b = 0.7
        return color

    def ego_vehicle_info_callback(self, msg):
        self.ego_vehicle_id = msg.id
        pass

    def actor_list_callback(self, msg):
        next_environment_object_id = np.iinfo(np.int32).max
        for actor in msg.actors:
            if len(self.environment_objects_labels_filter_positive) > 0:
                if actor.city_object_label not in self.environment_objects_labels_filter_positive:
                    self.get_logger().debug(f"Filtering out environment object with ID {np.uint64(actor.id)} and label {CityObjectLabel(actor.city_object_label).name} because it's not in the positive filter list")
                    continue
            elif actor.city_object_label in self.environment_objects_labels_filter_negative:
                self.get_logger().debug(f"Filtering out environment object with ID {np.uint64(actor.id)} and label {CityObjectLabel(actor.city_object_label).name}")
                continue

            # the 64 bit unreal ID depends on the blueprint/mesh of the object
            # multiple instances of the same object will have the same 64 bit ID,
            # but the city object label for these are identical as well
            self.object_to_label_map[np.uint64(actor.id)] = actor.city_object_label

            # on 32 bit IDs we reserve for every environment object a unique ID starting from int32 max and counting downwards,
            # to avoid conflicts with the actor IDs which are also 32 bit but can be up to around 100k and usually start from 1 and count upwards.
            # Since we only visualize a subset of the environment objects based on their city object label, we don't need to reserve IDs for all
            # environment objects, but only for the ones which are actually visualized based on the filtering.
            if np.uint64(actor.id) in self.object_to_id_map:
                self.object_to_id_map[np.uint64(actor.id)].append(next_environment_object_id)
            else:
                self.object_to_id_map[np.uint64(actor.id)] = [next_environment_object_id]

            self.get_logger().debug(f"Adding environment object with int64 ID {np.uint64(actor.id)}, mapped to int32 ID {next_environment_object_id} and label {CityObjectLabel(actor.city_object_label).name} to visualization")

            next_environment_object_id -= 1


        self.get_logger().warn(f"Found {len(self.object_to_label_map)} relevant environment objects for visualization")
        self.wait_for_actor_list = False
        if self.objects is not None:
            self.get_logger().info("Received actor list after objects. Will process objects now.")
            self.create_object_markers(self.objects)

    def object_callback(self, msg):
        self.objects = msg.objects
        if self.wait_for_actor_list:
            self.get_logger().info("Received objects before actor list was fully processed. Will process objects once actor list is ready.")
            return
        self.create_object_markers(msg.objects)

    def create_object_markers(self, objects):
        marker_array = MarkerArray()

        # since environment objects are usually not updated, we don't need to store which one gets with of the reserved IDs
        object_to_id_map = deepcopy(self.object_to_id_map)

        for object in objects:

            if self.ego_vehicle_id is not None and object.id == self.ego_vehicle_id:
                # skip the ego vehicle, as it might be already visualized by the robot state publisher or similar
                continue

            city_object_label = None
            object_id = object.id

            if self.actor_list_subscription:
                # here comes the 'trick' on the 64 bit environment object IDs:
                # the upper 32 bits correspond to the classification age,
                # the lower 32 bits correspond to the object ID.
                # This is necessary because Object.msg only supports 32 bit while environment objects share the 64 bit unreal object id
                int64_object_id = np.uint64(object.classification_age)<< np.uint64(32) | np.uint64(object.id)
                if int64_object_id in self.object_to_label_map:
                    city_object_label = self.object_to_label_map[int64_object_id]
                else:
                    # skip objects which are not in the actor list, as those were filtered out
                    continue

                if int64_object_id in object_to_id_map:
                    object_id = object_to_id_map[int64_object_id].pop(0)
                else:
                    self.get_logger().warn(f"Object with int64 ID {int64_object_id} not found in object_to_id_map. This should not happen, as the object should have been filtered out if it was not in the actor list. Object details: classification age {object.classification_age}, object id {object.id}, object type {object.classification}, city object label {city_object_label}")

            marker = Marker()
            marker.header = object.header
            marker.ns = "Objects"
            marker.type = Marker.CUBE
            marker.lifetime = rclpy.duration.Duration(seconds=self.lifetime).to_msg()

            marker.id = object_id
            if ( marker.id < 0):
                self.get_logger().warn(f"Object with ID {object_id} has a negative ID. This should not happen. Object details: classification age {object.classification_age}, object id {object.id}, object type {object.classification}, city object label {city_object_label}")
            marker.pose.position.x = object.pose.position.x
            marker.pose.position.y = object.pose.position.y
            marker.pose.position.z = object.pose.position.z
            marker.pose.orientation.x = object.pose.orientation.x
            marker.pose.orientation.y = object.pose.orientation.y
            marker.pose.orientation.z = object.pose.orientation.z
            marker.pose.orientation.w = object.pose.orientation.w

            if len(object.shape.dimensions) == 3:
                marker.scale.x = object.shape.dimensions[0]
                marker.scale.y = object.shape.dimensions[1]
                marker.scale.z = object.shape.dimensions[2]
            else:
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

            marker.color = self.get_object_color(object, city_object_label)

            marker.action = Marker.ADD

            marker_array.markers.append(marker)

            # moving objects get an additional arrow marker to indicate their heading and a line marker to indicate their speed, as well as a text marker to indicate their ID and speed value
            if (object.classification == Object.CLASSIFICATION_PEDESTRIAN) or \
                (object.classification == Object.CLASSIFICATION_BIKE) or \
                (object.classification == Object.CLASSIFICATION_CAR) or \
                (object.classification == Object.CLASSIFICATION_TRUCK) or \
                (object.classification == Object.CLASSIFICATION_MOTORCYCLE) or \
                (object.classification == Object.CLASSIFICATION_OTHER_VEHICLE):
                # Heading
                marker = Marker()
                marker.header = object.header
                marker.ns = "Heading"
                marker.type = Marker.ARROW
                marker.lifetime = rclpy.duration.Duration(seconds=1.).to_msg()

                marker.id = object_id
                marker.pose.position.x = object.pose.position.x
                marker.pose.position.y = object.pose.position.y
                marker.pose.position.z = object.pose.position.z
                marker.pose.orientation.x = object.pose.orientation.x
                marker.pose.orientation.y = object.pose.orientation.y
                marker.pose.orientation.z = object.pose.orientation.z
                marker.pose.orientation.w = object.pose.orientation.w

                marker.scale.x = max(object.shape.dimensions[0], 1.0)
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                marker.color = self.get_object_color(object)

                marker.action = Marker.ADD

                marker_array.markers.append(marker)

                # Speed and ID only for the active actors
                if ( city_object_label != CityObjectLabel.NONE):
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

                    marker.id = object_id

                    marker.scale.x = 0.2
                    marker.scale.y = 0.
                    marker.scale.z = 0.

                    point = Point()
                    point = object.pose.position
                    marker.points.append(point)

                    point2 = Point()
                    point2.x = point.x + object.twist.linear.x*10.0
                    point2.y = point.y + object.twist.linear.y*10.0
                    point2.z = point.z

                    marker.points.append(point2)
                    marker_array.markers.append(marker)

                    marker = Marker()
                    marker.header = object.header
                    marker.ns = "IDs"
                    marker.lifetime = rclpy.duration.Duration(seconds=1.).to_msg()
                    marker.type = Marker.TEXT_VIEW_FACING

                    marker.id = object_id
                    marker.pose.position.x = object.pose.position.x
                    marker.pose.position.y = object.pose.position.y
                    marker.pose.position.z = object.pose.position.z + 2.0

                    marker.scale.z = 1.0
                    marker.color.g = 1.0
                    marker.color.a = 1.0

                    speed = round(math.sqrt(object.twist.linear.x**2 + object.twist.linear.y**2),2)
                    marker.text = str(object.id) + "  -  " + str(speed) + "m/s"
                    marker.action = Marker.ADD

                    marker_array.markers.append(marker)

        self.object_publisher.publish(marker_array)

    def before_shutdown(self):
        pass
