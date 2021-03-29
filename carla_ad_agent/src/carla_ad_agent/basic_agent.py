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

from carla_waypoint_types.srv import GetActorWaypoint  # pylint: disable=import-error
from carla_msgs.msg import CarlaActorList  # pylint: disable=import-error
from derived_object_msgs.msg import ObjectArray  # pylint: disable=import-error
import math
from ros_compatibility import (
    ros_ok,
    ServiceException,
    QoSProfile,
    latch_on,
    get_service_request,
    ROS_VERSION)

if ROS_VERSION == 1:
    from agent import Agent, AgentState  # pylint: disable=relative-import
elif ROS_VERSION == 2:
    from rclpy.callback_groups import ReentrantCallbackGroup
    from carla_ad_agent.agent import Agent, AgentState  # pylint: disable=relative-import


class BasicAgent(Agent):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects traffic lights and other vehicles.
    """

    def __init__(self, role_name, ego_vehicle_id, node, avoid_risk=True):
        """
        """
        super(BasicAgent, self).__init__(role_name, ego_vehicle_id, avoid_risk, node)
        self.node = node
        self._avoid_risk = avoid_risk
        self._proximity_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING

        if ROS_VERSION == 1:
            cb_group = None
        elif ROS_VERSION == 2:
            cb_group = ReentrantCallbackGroup()

        if self._avoid_risk:
            self._vehicle_id_list = []
            self._lights_id_list = []
            self._objects = []
            self._actors_subscriber = self.node.create_subscriber(CarlaActorList, "/carla/actor_list",
                                                                  self.actors_updated, callback_group=cb_group)
            self._objects_subscriber = self.node.create_subscriber(ObjectArray,
                                                                   "/carla/{}/objects".format(role_name), self.objects_updated)
            self._get_actor_waypoint_client = self.node.create_service_client(
                '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(role_name),
                GetActorWaypoint, callback_group=cb_group)

    def get_actor_waypoint(self, actor_id):
        """
        helper method to get waypoint for actor via ros service
        Only used if risk should be avoided.
        """
        try:
            request = get_service_request(GetActorWaypoint)
            request.id = actor_id
            response = self.node.call_service(self._get_actor_waypoint_client, request)
            return response.waypoint
        except ServiceException as e:
            if ros_ok():
                self.node.logwarn("Service call failed: {}".format(e))

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
            elif "traffic_light" in actor.type:
                self._lights_id_list.append(
                    (actor.id, self.get_actor_waypoint(actor.id)))

    def objects_updated(self, objects):
        """
        callback on new objects
        Only used if risk should be avoided.
        """
        self._objects = objects.objects

    def run_step(self):
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
                self._state = AgentState.BLOCKED_BY_VEHICLE
                hazard_detected = True

            # check for the state of the traffic lights
            light_state, traffic_light = self._is_light_red(  # pylint: disable=unused-variable
                self._lights_id_list)
            if light_state:
                self._state = AgentState.BLOCKED_RED_LIGHT
                hazard_detected = True

        if hazard_detected is False:
            self._state = AgentState.NAVIGATING

        return hazard_detected
