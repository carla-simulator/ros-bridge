#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base class for agent
"""

from carla_waypoint_types.srv import GetWaypoint  # pylint: disable=import-error
from carla_msgs.msg import CarlaTrafficLightStatusList, CarlaWorldInfo  # pylint: disable=import-error
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaTrafficLightStatus  # pylint: disable=import-error
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from enum import Enum
import math
import time
from transforms3d.euler import quat2euler
from ros_compatibility import (
    ros_ok,
    ServiceException,
    QoSProfile,
    latch_on,
    get_service_request,
    ROS_VERSION)

if ROS_VERSION == 1:
    from misc import is_within_distance_ahead, compute_magnitude_angle   # pylint: disable=relative-import
elif ROS_VERSION == 2:
    from carla_ad_agent.misc import is_within_distance_ahead, compute_magnitude_angle   # pylint: disable=relative-import
    from rclpy import spin_once


class AgentState(Enum):
    """
    AGENT_STATE represents the possible states of a roaming agent
    """
    NAVIGATING = 1
    BLOCKED_BY_VEHICLE = 2
    BLOCKED_RED_LIGHT = 3


class Agent(object):
    """
    Base class for agent
    """

    def __init__(self, role_name, vehicle_id, avoid_risk, node):
        """
        """
        self.node = node
        self._proximity_threshold = 10.0  # meters
        self._map_name = None
        self._vehicle_location = None
        self._vehicle_yaw = None
        self._vehicle_id = vehicle_id
        self._last_traffic_light = None
        self._target_route_point = None

        self._odometry_subscriber = self.node.create_subscriber(
            Odometry, "/carla/{}/odometry".format(role_name), self.odometry_updated)
        # wait for first odometry update
        self.node.logwarn('Agent waiting for odometry  message')
        while self._vehicle_location is None:
            time.sleep(0.05)
            if ROS_VERSION == 2:
                spin_once(node, timeout_sec=0)
        self.node.logwarn('Odometry message received')

        if avoid_risk:
            self._get_waypoint_client = node.create_service_client(
                '/carla_waypoint_publisher/{}/get_waypoint'.format(role_name), GetWaypoint)

            self._traffic_lights = []
            self._traffic_light_status_subscriber = node.create_subscriber(CarlaTrafficLightStatusList,
                                                                           "/carla/traffic_lights/status", self.traffic_lights_updated,
                                                                           qos_profile=QoSProfile(depth=10, durability=latch_on))
            self._target_point_subscriber = node.create_subscriber(
                Marker, "/carla/{}/next_target".format(role_name), self.target_point_updated)
            world_info = node.wait_for_one_message(
                "/carla/world_info", CarlaWorldInfo, qos_profile=QoSProfile(depth=1, durability=latch_on))
            self._map_name = world_info.map_name

    def traffic_lights_updated(self, traffic_lights):
        """
        callback on new traffic light list
        Only used if risk should be avoided.
        """
        self._traffic_lights = traffic_lights.traffic_lights

    def target_point_updated(self, new_target_point):
        self._target_route_point = new_target_point.pose.position

    def odometry_updated(self, odo):
        """
        callback on new odometry
        """
        self._vehicle_location = odo.pose.pose.position
        quaternion = (
            odo.pose.pose.orientation.w,
            odo.pose.pose.orientation.x,
            odo.pose.pose.orientation.y,
            odo.pose.pose.orientation.z
        )
        _, _, self._vehicle_yaw = quat2euler(quaternion)

    def _is_light_red(self, lights_list):
        """
        Method to check if there is a red light affecting us. This version of
        the method is compatible with both European and US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                 - bool_flag is True if there is a traffic light in RED
                   affecting us and False otherwise
                 - traffic_light is the object itself or None if there is no
                   red traffic light affecting us
        """
        if self._map_name == 'Town01' or self._map_name == 'Town02':
            return self._is_light_red_europe_style(lights_list)
        else:
            return self._is_light_red_us_style(lights_list)

    def _is_light_red_europe_style(self, lights_list):
        """
        This method is specialized to check European style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                 - bool_flag is True if there is a traffic light in RED
                  affecting us and False otherwise
                 - traffic_light is the object itself or None if there is no
                   red traffic light affecting us
        """
        if self._vehicle_location is None:
            # no available self location yet
            return (False, None)

        ego_vehicle_location = get_service_request(GetWaypoint)
        ego_vehicle_location.location = self._vehicle_location
        ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_location)
        if not ego_vehicle_waypoint:
            if ros_ok():
                self.node.logwarn("Could not get waypoint for ego vehicle.")
            return (False, None)

        for traffic_light in lights_list:
            object_waypoint = traffic_light[1]
            if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                    object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue
            if is_within_distance_ahead(object_waypoint.pose.position, ego_vehicle_location.location,
                                        math.degrees(self._vehicle_yaw),
                                        self._proximity_threshold):
                traffic_light_state = CarlaTrafficLightStatus.RED
                for status in self._traffic_lights:
                    if status.id == traffic_light[0]:
                        traffic_light_state = status.state
                        break
                if traffic_light_state == CarlaTrafficLightStatus.RED:
                    return (True, traffic_light[0])

        return (False, None)

    def get_waypoint(self, location):
        """
        Helper to get waypoint for location via ros service.
        Only used if risk should be avoided.
        """
        if not ros_ok():
            return None
        try:
            response = self.node.call_service(self._get_waypoint_client, location)
            return response.waypoint
        except ServiceException as e:
            if ros_ok():
                self.node.logwarn("Service call 'get_waypoint' failed: {}".format(str(e)))

    def _is_light_red_us_style(self, lights_list):  # pylint: disable=too-many-branches
        """
        This method is specialized to check US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                 - bool_flag is True if there is a traffic light in RED
                   affecting us and False otherwise
                 - traffic_light is the object itself or None if there is no
                   red traffic light affecting us
        """
        if self._vehicle_location is None:
            # no available self location yet
            return (False, None)

        ego_vehicle_location = get_service_request(GetWaypoint)
        ego_vehicle_location.location = self._vehicle_location
        ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_location)
        if not ego_vehicle_waypoint:
            if ros_ok():
                self.node.logwarn("Could not get waypoint for ego vehicle.")
            return (False, None)

        if ego_vehicle_waypoint.is_junction:
            # It is too late. Do not block the intersection! Keep going!
            return (False, None)

        if self._target_route_point is not None:
            request = get_service_request(GetWaypoint)
            request.location = self._target_route_point
            target_waypoint = self.get_waypoint(request)
            if not target_waypoint:
                if ros_ok():
                    self.node.logwarn("Could not get waypoint for target route point.")
                return (False, None)
            if target_waypoint.is_junction:
                min_angle = 180.0
                sel_magnitude = 0.0  # pylint: disable=unused-variable
                sel_traffic_light = None
                for traffic_light in lights_list:
                    loc = traffic_light[1]
                    magnitude, angle = compute_magnitude_angle(loc.pose.position,
                                                               ego_vehicle_location.location,
                                                               math.degrees(self._vehicle_yaw))
                    if magnitude < 60.0 and angle < min(25.0, min_angle):
                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light[0]
                        min_angle = angle

                if sel_traffic_light is not None:
                    # print('=== Magnitude = {} | Angle = {} | ID = {}'.format(
                    #     sel_magnitude, min_angle, sel_traffic_light))

                    if self._last_traffic_light is None:
                        self._last_traffic_light = sel_traffic_light

                    state = None
                    for status in self._traffic_lights:
                        if status.id == sel_traffic_light:
                            state = status.state
                            break
                    if state is None:
                        self.node.logwarn(
                            "Couldn't get state of traffic light {}".format(
                                sel_traffic_light))
                        return (False, None)

                    if state == CarlaTrafficLightStatus.RED:
                        return (True, self._last_traffic_light)
                else:
                    self._last_traffic_light = None

        return (False, None)

    def _is_vehicle_hazard(self, vehicle_list, objects):
        """
        Check if a given vehicle is an obstacle in our way. To this end we take
        into account the road and lane the target vehicle is on and run a
        geometry test to check if the target vehicle is under a certain distance
        in front of our ego vehicle.

        WARNING: This method is an approximation that could fail for very large
         vehicles, which center is actually on a different lane but their
         extension falls within the ego vehicle lane.

        :param vehicle_list: list of potential obstacle to check
        :return: a tuple given by (bool_flag, vehicle), where
                 - bool_flag is True if there is a vehicle ahead blocking us
                   and False otherwise
                 - vehicle is the blocker object itself
        """
        if self._vehicle_location is None:
            # no available self location yet
            return (False, None)

        ego_vehicle_location = get_service_request(GetWaypoint)
        ego_vehicle_location.location = self._vehicle_location
        ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_location)
        if not ego_vehicle_waypoint:
            if ros_ok():
                self.node.logwarn("Could not get waypoint for ego vehicle.")
            return (False, None)

        for target_vehicle_id in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle_id == self._vehicle_id:
                continue

            target_vehicle_location = None
            for elem in objects:
                if elem.id == target_vehicle_id:
                    target_vehicle_location = elem.pose
                    break

            if not target_vehicle_location:
                self.node.logwarn("Location of vehicle {} not found".format(target_vehicle_id))
                continue

            # if the object is not in our lane it's not an obstacle
            request = get_service_request(GetWaypoint)
            request.location = target_vehicle_location.position
            target_vehicle_waypoint = self.get_waypoint(request)
            if not target_vehicle_waypoint:
                if ros_ok():
                    self.node.logwarn("Could not get waypoint for target vehicle.")
                return (False, None)
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                    target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            if is_within_distance_ahead(target_vehicle_location.position, self._vehicle_location,
                                        math.degrees(self._vehicle_yaw),
                                        self._proximity_threshold):
                return (True, target_vehicle_id)

        return (False, None)

    def emergency_stop(self):  # pylint: disable=no-self-use
        """
        Send an emergency stop command to the vehicle
        :return:
        """
        control = CarlaEgoVehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False
        return control
