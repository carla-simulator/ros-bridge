#!/usr/bin/env python
#
# Copyright (c) Institute for Automotive Engineering (ika), RWTH Aachen University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Handle an IdealObjectSensor
"""
import ros_compatibility as roscomp
ROS_VERSION = roscomp.get_ros_version()

import math
import sys
import carla
import carla_common.transforms as trans
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker
from carla_ros_bridge.object_sensor import ObjectSensor

from derived_object_msgs.msg import ObjectArray
from geometry_msgs.msg import Point

import tf2_ros

if ROS_VERSION == 2:
    from rclpy.time import Time
    from rclpy.duration import Duration

class IdealObjectSensor(ObjectSensor):

    """
    IdealObjectSensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, actor_list, world, attributes, tf_buffer):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identifying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        :param world: current carla world object
        :type world: carla.World
        :param attributes: attributes of IdealObjectSensor
        :type attributes: diagnostic_msgs/KeyValue[]
        :param tf_buffer: shared transform buffer owned by the bridge node
        :type tf_buffer: tf2_ros.Buffer
        """
        super(IdealObjectSensor, self).__init__(uid=uid,
                                                      name=name,
                                                      parent=parent,
                                                      node=node,
                                                      actor_list=actor_list, 
                                                      world=world,
                                                      attributes=attributes)
        self.node = node

        # A ground-relative spawn point makes the transform of this sensor report a height
        # above the terrain, while the targets are reported by CARLA at their absolute
        # altitude and need to be brought into the same frame of reference before they are compared.
        # The bridge resolves the ground altitude once when the sensor is spawned, so that this
        # sensor uses the same ground as the actors it is mounted next to.
        self._ground_offset = 0.0

        # static vehicles as visibility targets, see _get_static_targets()
        self._static_targets = None

        # Skip init if ROS_VERSION is 1
        if ROS_VERSION == 1:
            self.node.logwarn("IdealObjectSensor is not supported for ROS_VERSION 1")
            return

        if tf_buffer is None:
            raise ValueError("IdealObjectSensor requires a shared tf_buffer")
        self.tf_buffer = tf_buffer

        # Set up TransformBroadcaster to publish sensor transform
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        # Extract (relative) spawn pose
        self.relative_spawn_pose = relative_spawn_pose

        if str(self._attribute_value(
                attributes, "ground_relative_z", "false")).lower() == "true":
            ground_altitude = self._attribute_value(attributes, "ground_altitude", 0.0)
            try:
                self._ground_offset = float(ground_altitude)
            except (TypeError, ValueError):
                self.node.logwarn(
                    "ground_altitude attribute for IdealObjectSensor is invalid! "
                    "Using default value of {}.".format(self._ground_offset))

        # Set default values, boundaries and unit for sensor parameters so that they are available when needed
        attributes_dict = {
            "range":                        {"default": 100.0,  "lower_boundary": 0},
            "left_fov":                     {"default": -180.0, "lower_boundary": -180, "upper_boundary": 0},
            "right_fov":                    {"default": 180.0,  "lower_boundary": 0,    "upper_boundary": 180},
            "upper_fov":                    {"default": 90.0,   "lower_boundary": 0,    "upper_boundary": 90},
            "lower_fov":                    {"default": -90.0,  "lower_boundary": -90,  "upper_boundary": 0},
            "min_corner_amount":            {"default": 1,      "lower_boundary": 1,    "upper_boundary": 8},
            "target_center_range_margin":   {"default": 10.0,   "lower_boundary": 0}, # 10 Meters based on the length of a truck
            "enable_occlusion_filter":      {"default": False},
            "hit_point_blanking_radius":    {"default": 0.0,    "lower_boundary": 0}
        }
        # Extract and check attributes and set default values if not available or values are not set in parameter boundaries
        for key, current_dict in attributes_dict.items():
            default = current_dict.get("default")
            attribute = next((attribute for attribute in attributes if attribute.key == key), None)

            if attribute is None:
                setattr(self, key, default)
                self.node.logwarn(
                    "No {} attribute found for IdealObjectSensor. Using default value of {}.".format(
                        key, default))
                continue

            try:
                if key == "enable_occlusion_filter":
                    value = attribute.value.strip().lower()
                    if value not in ("true", "1", "yes", "false", "0", "no"):
                        raise ValueError
                    setattr(self, key, value in ("true", "1", "yes"))
                    continue

                value = float(attribute.value)
                if key == "min_corner_amount":
                    if not value.is_integer():
                        raise ValueError
                    value = int(value)

                if value < current_dict.get("lower_boundary") or (
                        "upper_boundary" in current_dict.keys() and value > current_dict.get("upper_boundary")):
                    setattr(self, key, default)
                    self.node.logwarn(
                        "{} attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {}.".format(
                            key, default))
                    continue

                setattr(self, key, value)
            except ValueError:
                setattr(self, key, default)
                self.node.logwarn(
                    "{} attribute for IdealObjectSensor is invalid! Using default value of {}.".format(
                        key, default))

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(IdealObjectSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)

    @staticmethod
    def _attribute_value(attributes, key, default=None):
        """
        Get the value of a spawn attribute
        :param attributes: attributes of the sensor
        :type attributes: diagnostic_msgs/KeyValue[]
        :param key: name of the attribute
        :return: the value of the attribute, or default if it is not set
        """
        return next((attribute.value for attribute in attributes if attribute.key == key),
                    default)

    @staticmethod
    def _lower(carla_location, ground_offset):
        """
        Lower a location by the ground offset.
        """
        return carla.Location(carla_location.x, carla_location.y,
                              carla_location.z - ground_offset)

    def _absolute(self, carla_location):
        """
        Undo the ground offset, returning the location at its absolute altitude.
        """
        if not self._ground_offset:
            return carla_location
        return self._lower(carla_location, -self._ground_offset)

    def _ground_relative(self, carla_location):
        """
        Bring a location of a target into the ground-relative frame this sensor is expressed in.

        CARLA reports the targets at their absolute altitude, while a ground-relative
        spawn point makes the transform of this sensor report a height above the
        terrain. Lowering the targets rather than raising the sensor keeps them
        consistent with the sensor transform the field-of-view check is made against.

        :param carla_location: the location of the target
        :return: the location, lowered by the ground offset
        """
        if not self._ground_offset:
            return carla_location
        return self._lower(carla_location, self._ground_offset)

    def _ground_relative_corners(self, carla_corners):
        """
        Bring the corners of a target's bounding box into the ground-relative frame, see
        _ground_relative().
        """
        if not self._ground_offset:
            return carla_corners
        return [self._lower(corner, self._ground_offset) for corner in carla_corners]

    def _update_ground_offset_from_parent(self):
        """
        Update the ground offset inherited from a flattened parent actor.

        CARLA continues to report absolute target altitudes when ``ignore_altitude``
        flattens a vehicle or walker transform to z=0. An attached pseudo sensor inherits
        that flattened frame, so it has to use the current altitude of the parent as its
        ground offset. Updating the existing value also makes the correction follow a
        moving parent over changing road elevation.
        """
        if not self.node.parameters['ignore_altitude']:
            return

        parent = self.parent
        while parent is not None:
            if isinstance(parent, (Vehicle, Walker)):
                self._ground_offset = parent.carla_actor.get_location().z
                return
            parent = parent.parent

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.ideal_objects"

    def calculate_azimuth_elevation(self, target_point_in_sensor_frame, distance):
        """
        :param target_point_in_sensor_frame: the target point in the sensor frame as (x, y, z)
        :param distance: the distance between sensor and target point
        :return: azimuth and elevation of the target point in degrees
        """
        if distance <= 0.0:
            return 0.0, 0.0

        # Get location values of pose
        dx, dy, dz = target_point_in_sensor_frame

        # Calculate azimuth and elevation between target and sensor based on sensor frame
        azimuth = math.degrees(math.atan2(dy, dx))
        elevation_ratio = max(-1.0, min(1.0, dz / distance))
        elevation = math.degrees(math.asin(elevation_ratio))

        return azimuth, elevation

    def is_in_center_range(self, carla_location_sensor_in_carla_map,
                           carla_location_target_in_carla_map):
        """
        FILTER 1: Filter targets that are far outside the sensor range based on the target center.

        It is checked before the bounding-box corners of a target are computed, so that targets
        far away are cheap.
        """
        distance = carla_location_sensor_in_carla_map.distance(carla_location_target_in_carla_map)
        return distance <= self.range + self.target_center_range_margin

    def check_visibility(self, carla_location_sensor_in_carla_map,
                         carla_corners_target_in_carla_map, carla_map_to_sensor):
        """
        Check FILTER 2 to 4 for a target that passed FILTER 1 (see is_in_center_range()).

        :param carla_map_to_sensor: transform from carla_map to the sensor frame, see
            _get_carla_map_to_sensor()
        """

        # FILTER 2
        # Filter corners that are outside the sensor range and return if not enough corners are visible
        corner_list_filter_2 = list()

        for corner in carla_corners_target_in_carla_map:
            corner_distance = carla_location_sensor_in_carla_map.distance(corner)
            if corner_distance > self.range:
                continue
            corner_list_filter_2.append([corner, corner_distance])

        if len(corner_list_filter_2) < self.min_corner_amount:
            return False

        # FILTER 3
        # Filter corners outside the sensor FOV and return if not enough corners are visible
        corner_list_filter_3 = list()

        for corner, corner_distance in corner_list_filter_2:
            # Get position of corner in sensor frame (ROS environment)
            ros_corner_in_sensor_frame = self._to_sensor_frame(corner, carla_map_to_sensor)

            # Calculate azimuth and elevation
            azimuth, elevation = self.calculate_azimuth_elevation(
                ros_corner_in_sensor_frame, corner_distance)

            # Check if corner is inside the sensor FOV
            if azimuth < self.left_fov: continue
            if azimuth > self.right_fov: continue
            if elevation > self.upper_fov: continue
            if elevation < self.lower_fov: continue

            corner_list_filter_3.append(corner)

        if len(corner_list_filter_3) < self.min_corner_amount:
            return False

        if not self.enable_occlusion_filter:
            return True

        # FILTER 4
        # Filter corners that are occluded by other objects and return if not enough corners are visible
        corner_list_filter_4 = list()
        carla_location_sensor = self._absolute(carla_location_sensor_in_carla_map)

        for corner in corner_list_filter_3:
            hit = False
            # Send ray from corner to sensor and check for objects
            hit_points = self.world.cast_ray(self._absolute(corner), carla_location_sensor)
            if hit_points:
                for hit_point in hit_points:
                    # Skip hit points with the label "Roads"
                    if hit_point.label is carla.CityObjectLabel.Roads:
                        continue
                    # Skip hit points with the label "NONE"
                    if hit_point.label is carla.CityObjectLabel.NONE:
                        continue
                    # Skip hit points near to the sensor location within a defined hit point blanking radius
                    if hit_point.location.distance(carla_location_sensor) <= self.hit_point_blanking_radius:
                        continue
                    # All other hits are relevant --> current corner is not visible, continue with next corner
                    hit = True
                    break
                if hit: continue
            corner_list_filter_4.append(corner)

        if len(corner_list_filter_4) < self.min_corner_amount:
            return False

        return True

    @staticmethod
    def _get_carla_map_to_sensor(ros_tf_carla_map_to_sensor):
        """
        Get the rotation matrix and the translation of a ROS transform from carla_map to the
        sensor frame.

        This is the computation of tf2_geometry_msgs.do_transform_point(), which is done once
        per update instead of for every corner of every target.

        :return: rotation matrix as rows and translation
        """
        rotation = ros_tf_carla_map_to_sensor.transform.rotation
        translation = ros_tf_carla_map_to_sensor.transform.translation
        w, x, y, z = rotation.w, rotation.x, rotation.y, rotation.z
        norm = w * w + x * x + y * y + z * z
        if norm < sys.float_info.epsilon:
            matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        else:
            # the quaternion does not need to be normalized
            s = 2.0 / norm
            wx, wy, wz = w * x * s, w * y * s, w * z * s
            xx, xy, xz = x * x * s, x * y * s, x * z * s
            yy, yz, zz = y * y * s, y * z * s, z * z * s
            matrix = ((1.0 - (yy + zz), xy - wz, xz + wy),
                      (xy + wz, 1.0 - (xx + zz), yz - wx),
                      (xz - wy, yz + wx, 1.0 - (xx + yy)))
        return matrix, (translation.x, translation.y, translation.z)

    @staticmethod
    def _to_sensor_frame(carla_location, carla_map_to_sensor):
        """
        Transform a CARLA location in carla_map into a ROS point in the sensor frame.

        :param carla_map_to_sensor: see _get_carla_map_to_sensor()
        :return: the point in the sensor frame as (x, y, z)
        """
        (row_x, row_y, row_z), (tx, ty, tz) = carla_map_to_sensor
        # from CARLA's left-handed to ROS' right-handed coordinate system
        x, y, z = carla_location.x, -carla_location.y, carla_location.z
        return (row_x[0] * x + row_x[1] * y + row_x[2] * z + tx,
                row_y[0] * x + row_y[1] * y + row_y[2] * z + ty,
                row_z[0] * x + row_z[1] * y + row_z[2] * z + tz)

    def _get_static_targets(self):
        """
        Get the static vehicles of the map as visibility targets: tuples of the location of the
        center, the corners of the bounding box, and the object message, in CARLA's absolute frame.

        Static vehicles do not move, so their geometry is computed only once.
        """
        if self._static_targets is None:
            self._static_targets = [
                (self._get_environment_object_transform(vehicle).location,
                 list(self._get_environment_object_world_vertices(vehicle)),
                 vehicle_obj)
                for vehicle, vehicle_obj in self._get_static_vehicles()]
        return self._static_targets

    def get_ros_transform(self, timestamp):
        # Get transform of idealObjectSensor
        if not self.relative_spawn_pose:
            self.node.logwarn("{}: No relative spawn pose defined.".format(self.get_prefix()))
            return
        if self.parent is not None:
            frame_id = self.parent.get_prefix()
        else:
            frame_id = "carla_map"
        child_frame_id = self.get_prefix()

        transform = tf2_ros.TransformStamped()
        transform.header.stamp = roscomp.ros_timestamp(sec=timestamp + self.node.parameters["start_unix_time_stamp"], from_sec=True)
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id

        transform.transform.translation.x = self.relative_spawn_pose.position.x
        transform.transform.translation.y = self.relative_spawn_pose.position.y
        transform.transform.translation.z = self.relative_spawn_pose.position.z

        transform.transform.rotation.x = self.relative_spawn_pose.orientation.x
        transform.transform.rotation.y = self.relative_spawn_pose.orientation.y
        transform.transform.rotation.z = self.relative_spawn_pose.orientation.z
        transform.transform.rotation.w = self.relative_spawn_pose.orientation.w

        return transform

    def publish_tf(self, timestamp):
        # Publish transform of idealObjectSensor
        transform = self.get_ros_transform(timestamp)
        if transform is None:
            return
        try:
            self._tf_broadcaster.sendTransform(transform)
            self.tf_buffer.set_transform(transform, "carla_ros_bridge")
        except roscomp.exceptions.ROSException:
            if roscomp.ok():
                self.node.logwarn("Sensor {} failed to send transform.".format(self.uid))

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update carla_map sends:
        - tf global frame
        :return:
        """

        # Skip update if ROS_VERSION is 1
        if ROS_VERSION == 1:
            self.node.logwarn("IdealObjectSensor is not supported for ROS_VERSION 1")
            return

        # Publish transform of IdealObjectSensor at timestamp
        self.publish_tf(timestamp)

        if not self._should_publish(timestamp):
            return

        # Generate object array to publish sensor data
        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="carla_map", timestamp=timestamp)

        self._update_ground_offset_from_parent()

        # Get ROS transform from IdealObjectSensor to carla_map and vice versa
        sensor_frame = self.get_prefix()
        time_latest_tf = Time(seconds=0)
        duration_timeout = Duration(seconds=0)
        try:
            ros_tf_carla_map_to_sensor = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map', time_latest_tf, duration_timeout)
            ros_tf_sensor_to_carla_map = self.tf_buffer.lookup_transform('carla_map', sensor_frame, time_latest_tf, duration_timeout)
        except Exception as e:
            self.node.loginfo("{}: Could not transform {} to {} at the Frame {}: {}".format(
                self.__class__.__name__, sensor_frame, 'carla_map', frame, e))
            return

        # Extract sensor location in carla_map from ROS transform and convert into geometry_msgs/Point
        ros_point_sensor_in_carla_map = Point(
            x=ros_tf_sensor_to_carla_map.transform.translation.x,
            y=ros_tf_sensor_to_carla_map.transform.translation.y,
            z=ros_tf_sensor_to_carla_map.transform.translation.z
        )
        carla_location_sensor_in_carla_map = trans.ros_point_to_carla_location(ros_point_sensor_in_carla_map)

        carla_map_to_sensor = self._get_carla_map_to_sensor(ros_tf_carla_map_to_sensor)

        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():

            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):

                    # Get CARLA target location in carla_map
                    carla_location_target_in_carla_map = self._ground_relative(
                        actor.carla_actor.get_location())
                    if not self.is_in_center_range(carla_location_sensor_in_carla_map,
                                                   carla_location_target_in_carla_map):
                        continue

                    # Get corners from target BoundingBox
                    carla_tf_carla_map_to_target = actor.carla_actor.get_transform()
                    bounding_box = actor.carla_actor.bounding_box
                    carla_corners_target_in_carla_map = self._ground_relative_corners(
                        bounding_box.get_world_vertices(carla_tf_carla_map_to_target))

                    # Check visibility of the target
                    if self.check_visibility(carla_location_sensor_in_carla_map,
                                             carla_corners_target_in_carla_map,
                                             carla_map_to_sensor):
                        ros_objects.objects.append(actor.get_object_info(frame))

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            header = self.get_msg_header("carla_map")
            for carla_location_target_in_carla_map, carla_corners_target_in_carla_map, vehicle_obj \
                    in self._get_static_targets():

                carla_location_target_in_carla_map = self._ground_relative(
                    carla_location_target_in_carla_map)
                if not self.is_in_center_range(carla_location_sensor_in_carla_map,
                                               carla_location_target_in_carla_map):
                    continue
                carla_corners_target_in_carla_map = self._ground_relative_corners(
                    carla_corners_target_in_carla_map)

                # Check visibility of the target
                if self.check_visibility(carla_location_sensor_in_carla_map,
                                         carla_corners_target_in_carla_map,
                                         carla_map_to_sensor):
                    vehicle_obj.header = header
                    ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)
