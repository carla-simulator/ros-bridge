#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
BasicAgent implements a basic agent that navigates scenes to reach a given
target destination. This agent respects traffic lights and other vehicles.
"""

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Pose, Vector3
from derived_object_msgs.msg import ObjectArray
from carla_msgs.msg import CarlaActorList, CarlaWalkerControl
from tf.transformations import euler_from_quaternion
#from shapely.geometry import Point, Polygon
import shapely

#todo remove?
from misc import is_within_distance_ahead, compute_magnitude_angle, distance_vehicle

class RotatedRectangle(object):

    """
    This class contains method to draw rectangle and calculate the distance
    """

    def __init__(self, c_x, c_y, width, height, angle):
        self.c_x = c_x
        self.c_y = c_y
        self.w = width      # pylint: disable=invalid-name
        self.h = height     # pylint: disable=invalid-name
        self.angle = angle

    def get_contour(self):
        """
        create contour
        """
        w = self.w
        h = self.h
        c = shapely.geometry.box(-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.c_x, self.c_y)

    def intersection(self, other):
        """
        Obtain a intersection point between two contour.
        """
        return self.get_contour().intersection(other.get_contour())


class BasicAgent(object):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects other vehicles.
    """

    # minimum distance to target waypoint before switching to next
    MIN_DISTANCE = 0.5

    def __init__(self, role_name, vehicle_id, avoid_risk=True):
        """
        """        
        self._proximity_threshold = 10.0  # meters
        self._vehicle_id = vehicle_id
        self._vehicle_location = None
        self._vehicle_yaw = None
        self._waypoints = []
        self._avoid_risk = avoid_risk
        self._current_pose = Pose()

        if self._avoid_risk:
            self._vehicle_id_list = []
            self._lights_id_list = []
            self._actors_subscriber = rospy.Subscriber(
                "/carla/actor_list", CarlaActorList, self.actors_updated)
            self._objects = []
            self._objects_subscriber = rospy.Subscriber(
                "/carla/objects".format(role_name), ObjectArray, self.objects_updated)

        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)

    def odometry_updated(self, odo):
        """
        callback on new odometry
        """
        self._current_pose = odo.pose.pose
        
        self._vehicle_location = odo.pose.pose.position
        quaternion = (
            odo.pose.pose.orientation.x,
            odo.pose.pose.orientation.y,
            odo.pose.pose.orientation.z,
            odo.pose.pose.orientation.w
        )
        _, _, self._vehicle_yaw = euler_from_quaternion(quaternion)

    def actors_updated(self, actors):
        """
        callback on new actor list
        Only used if risk should be avoided.
        """
        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        self._vehicle_id_list = []
        self._lights_id_list = []
        for actor in actors.actors:
            if "vehicle" in actor.type:
                self._vehicle_id_list.append(actor.id)

    def objects_updated(self, objects):
        """
        callback on new objects
        Only used if risk should be avoided.
        """
        self._objects = objects.objects

    def run_step(self, target_speed):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """
        finished = False

        # is there an obstacle in front of us?
        hazard_detected = False

        if self._avoid_risk:
            # check possible obstacles
            vehicle_state, vehicle = self._is_vehicle_hazard(  # pylint: disable=unused-variable
                self._vehicle_id_list, self._objects)
            if vehicle_state:
                rospy.loginfo('=== Vehicle blocking ahead [{}])'.format(vehicle))
                hazard_detected = True

        if hazard_detected:
            control = self.emergency_stop()
        else:
            # standard local planner behavior
            control,finished = self.execute_next_step(target_speed, self._current_pose)

        return control,finished

    def set_global_plan(self, current_plan):
        """
        set a global plan to follow
        """
        self._waypoints = []
        for elem in current_plan:
            self._waypoints.append(elem.pose)

    def execute_next_step(self, target_speed, current_pose):
        """
        Execute one step of local planning
        """
        if not self._waypoints or rospy.is_shutdown():
            control = CarlaWalkerControl()
            control.speed = 0.0

            rospy.loginfo("Route finished.")
            return control,True

        control = CarlaWalkerControl()

        direction = Vector3()
        direction.x = self._waypoints[0].position.x - current_pose.position.x
        direction.y = self._waypoints[0].position.y - current_pose.position.y
        direction_norm = math.sqrt(direction.x**2 + direction.y**2)
        if direction_norm > BasicAgent.MIN_DISTANCE:
            control.speed = target_speed
            control.direction.x = direction.x / direction_norm
            control.direction.y = direction.y / direction_norm
        else:
            self._waypoints = self._waypoints[1:]
            if self._waypoints:
                rospy.loginfo("next waypoint: {} {}".format(self._waypoints[0].position.x, self._waypoints[0].position.y))

        return control,False

    def _is_vehicle_hazard(self, vehicle_list, objects):
        """
        """
        
        is_hazard = False
        for adversary in vehicle_list:
            if adversary.id != self._vehicle_id:
                vehicle_object = None
                for obj in objects:
                    if obj.id == adversary.id:
                        vehicle_object = obj
                        break
                
                if not vehicle_object:
                    continue

                distance = distance_vehicle(self._vehicle_location, vehicle_object.pose.position)
                if distance < 50:
                    print("NEARBY {}".format(vehicle_object.id))
                    # adversary_bbox = adversary.bounding_box
                    # adversary_transform = adversary.get_transform()
                    # adversary_loc = adversary_transform.location
                    # adversary_yaw = adversary_transform.rotation.yaw
                    # overlap_adversary = RotatedRectangle(
                    #     adversary_loc.x, adversary_loc.y,
                    #     2 * margin * adversary_bbox.extent.x, 2 * margin * adversary_bbox.extent.y, adversary_yaw)
                    # overlap_actor = RotatedRectangle(
                    #     actor_location.x, actor_location.y,
                    #     2 * margin * actor_bbox.extent.x * extension_factor, 2 * margin * actor_bbox.extent.y, actor_yaw)
                    # overlap_area = overlap_adversary.intersection(overlap_actor).area
                    # if overlap_area > 0:
                    #     is_hazard = True
                    #     break

        # ego_vehicle_location = self._vehicle_location

        # ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_location)
        # if not ego_vehicle_waypoint:
        #     if not rospy.is_shutdown():
        #         rospy.logwarn("Could not get waypoint for ego vehicle.")
        #     return (False, None)

        # for target_vehicle_id in vehicle_list:
        #     # do not account for the ego vehicle
        #     if target_vehicle_id == self._vehicle_id:
        #         continue

        #     target_vehicle_location = None
        #     for elem in objects:
        #         if elem.id == target_vehicle_id:
        #             target_vehicle_location = elem.pose
        #             break

        #     if not target_vehicle_location:
        #         rospy.logwarn("Location of vehicle {} not found".format(target_vehicle_id))
        #         continue

        #     # if the object is not in our lane it's not an obstacle
        #     target_vehicle_waypoint = self.get_waypoint(target_vehicle_location.position)
        #     if not target_vehicle_waypoint:
        #         if not rospy.is_shutdown():
        #             rospy.logwarn("Could not get waypoint for target vehicle.")
        #         return (False, None)
        #     if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
        #             target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
        #         #rospy.loginfo("Vehicle {} is not in our lane".format(target_vehicle_id))
        #         continue

        #     if is_within_distance_ahead(target_vehicle_location.position, self._vehicle_location,
        #                                 math.degrees(self._vehicle_yaw),
        #                                 self._proximity_threshold):
        #         return (True, target_vehicle_id)

        return (False, None)

    def emergency_stop(self):  # pylint: disable=no-self-use
        """
        Send an emergency stop command to the vehicle
        :return:
        """
        control = CarlaWalkerControl()
        control.speed = 0.0

        return control
