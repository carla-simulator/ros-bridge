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

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from derived_object_msgs.msg import ObjectArray
from carla_msgs.msg import CarlaActorList
from agent import Agent, AgentState  # pylint: disable=relative-import
from local_planner import LocalPlanner  # pylint: disable=relative-import
from carla_waypoint_types.srv import GetActorWaypoint


class BasicAgent(Agent):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects traffic lights and other vehicles.
    """

    def __init__(self, role_name, ego_vehicle_id, avoid_risk=True):
        """
        """
        super(BasicAgent, self).__init__(role_name, ego_vehicle_id, avoid_risk)

        self._avoid_risk = avoid_risk
        self._current_speed = 0.0  # Km/h
        self._current_pose = Pose()
        self._proximity_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING
        args_lateral_dict = {
            'K_P': 0.9,
            'K_D': 0.0,
            'K_I': 0.1}
        self._local_planner = LocalPlanner(opt_dict={'lateral_control_dict': args_lateral_dict})

        if self._avoid_risk:
            self._vehicle_id_list = []
            self._lights_id_list = []
            self._actors_subscriber = rospy.Subscriber(
                "/carla/actor_list", CarlaActorList, self.actors_updated)
            self._objects = []
            self._objects_subscriber = rospy.Subscriber(
                "/carla/{}/objects".format(role_name), ObjectArray, self.objects_updated)
            self._get_actor_waypoint_client = rospy.ServiceProxy(
                '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(role_name),
                GetActorWaypoint)

        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)

    def get_actor_waypoint(self, actor_id):
        """
        helper method to get waypoint for actor via ros service
        Only used if risk should be avoided.
        """
        try:
            response = self._get_actor_waypoint_client(actor_id)
            return response.waypoint
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            if not rospy.is_shutdown:
                rospy.logwarn("Service call failed: {}".format(e))

    def odometry_updated(self, odo):
        """
        callback on new odometry
        """
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6
        self._current_pose = odo.pose.pose
        super(BasicAgent, self).odometry_updated(odo)

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
                #rospy.loginfo('=== Vehicle blocking ahead [{}])'.format(vehicle))
                self._state = AgentState.BLOCKED_BY_VEHICLE
                hazard_detected = True

            # check for the state of the traffic lights
            light_state, traffic_light = self._is_light_red(  # pylint: disable=unused-variable
                self._lights_id_list)
            if light_state:
                #rospy.loginfo('=== Red Light ahead [{}])'.format(traffic_light))

                self._state = AgentState.BLOCKED_RED_LIGHT
                hazard_detected = True

        if hazard_detected:
            control = self.emergency_stop()
        else:
            self._state = AgentState.NAVIGATING
            # standard local planner behavior
            control, finished = self._local_planner.run_step(
                target_speed, self._current_speed, self._current_pose)

        return control, finished
