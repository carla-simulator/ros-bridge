#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned or move, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed point is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'

Additionally, services are provided to interface CARLA waypoints.
"""
import math
import sys
import threading

import rospy
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetWaypointResponse, GetWaypoint
from carla_waypoint_types.srv import GetActorWaypointResponse, GetActorWaypoint

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


class CarlaToRosWaypointConverter(object):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self, carla_world):
        """
        Constructor
        """
        self.world = carla_world
        self.map = carla_world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self.waypoint_publisher = rospy.Publisher(
            '/carla/{}/waypoints'.format(self.role_name), Path, queue_size=1, latch=True)

        # initialize ros services
        self.getWaypointService = rospy.Service(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(self.role_name),
            GetWaypoint, self.get_waypoint)
        self.getActorWaypointService = rospy.Service(
            '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(self.role_name),
            GetActorWaypoint, self.get_actor_waypoint)

        # set initial goal
        self.goal = self.world.get_map().get_spawn_points()[0]

        self.current_route = None
        self.goal_subscriber = rospy.Subscriber(
            "/carla/{}/goal".format(self.role_name), PoseStamped, self.on_goal)

        self._update_lock = threading.Lock()

        # use callback to wait for ego vehicle
        rospy.loginfo("Waiting for ego vehicle...")
        self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)

    def destroy(self):
        """
        Destructor
        """
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)

    def get_waypoint(self, req):
        """
        Get the waypoint for a location
        """
        carla_position = carla.Location()
        carla_position.x = req.location.x
        carla_position.y = -req.location.y
        carla_position.z = req.location.z

        carla_waypoint = self.map.get_waypoint(carla_position)

        response = GetWaypointResponse()
        response.waypoint.pose.position.x = carla_waypoint.transform.location.x
        response.waypoint.pose.position.y = -carla_waypoint.transform.location.y
        response.waypoint.pose.position.z = carla_waypoint.transform.location.z
        response.waypoint.is_junction = carla_waypoint.is_junction
        response.waypoint.road_id = carla_waypoint.road_id
        response.waypoint.section_id = carla_waypoint.section_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        #rospy.logwarn("Get waypoint {}".format(response.waypoint.pose.position))
        return response

    def get_actor_waypoint(self, req):
        """
        Convenience method to get the waypoint for an actor
        """
        rospy.loginfo("get_actor_waypoint(): Get waypoint of actor {}".format(req.id))
        actor = self.world.get_actors().find(req.id)

        response = GetActorWaypointResponse()
        if actor:
            carla_waypoint = self.map.get_waypoint(actor.get_location())
            response.waypoint.pose.position.x = carla_waypoint.transform.location.x
            response.waypoint.pose.position.y = -carla_waypoint.transform.location.y
            response.waypoint.pose.position.z = carla_waypoint.transform.location.z
            response.waypoint.is_junction = carla_waypoint.is_junction
            response.waypoint.road_id = carla_waypoint.road_id
            response.waypoint.section_id = carla_waypoint.section_id
            response.waypoint.lane_id = carla_waypoint.lane_id
        else:
            rospy.logwarn("get_actor_waypoint(): Actor {} not valid.".format(req.id))
        return response

    def on_goal(self, goal):
        """
        Callback for /move_base_simple/goal

        Receiving a goal (e.g. from RVIZ '2D Nav Goal') triggers a new route calculation.

        :return:
        """
        rospy.loginfo("Received goal, trigger rerouting...")
        carla_goal = carla.Transform()
        carla_goal.location.x = goal.pose.position.x
        carla_goal.location.y = -goal.pose.position.y
        carla_goal.location.z = goal.pose.position.z + 2  # 2m above ground
        quaternion = (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        carla_goal.rotation.yaw = -math.degrees(yaw)

        self.goal = carla_goal
        self.reroute()

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None or self.goal is None:
            # no ego vehicle, remove route if published
            self.current_route = None
            self.publish_waypoints()
        else:
            self.current_route = self.calculate_route(self.goal)
        self.publish_waypoints()

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        with self._update_lock:
            hero = None
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    hero = actor
                    break

            ego_vehicle_changed = False
            if hero is None and self.ego_vehicle is not None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
                ego_vehicle_changed = True

            if not ego_vehicle_changed and hero is not None and \
                    self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
                ego_vehicle_changed = True

            if ego_vehicle_changed:
                rospy.loginfo("Ego vehicle changed.")
                self.ego_vehicle = hero
                self.reroute()
            elif self.ego_vehicle:
                current_location = self.ego_vehicle.get_location()
                if self.ego_vehicle_location:
                    dx = self.ego_vehicle_location.x - current_location.x
                    dy = self.ego_vehicle_location.y - current_location.y
                    distance = math.sqrt(dx * dx + dy * dy)
                    if distance > self.WAYPOINT_DISTANCE:
                        rospy.loginfo("Ego vehicle was repositioned.")
                        self.reroute()
                self.ego_vehicle_location = current_location

    def calculate_route(self, goal):
        """
        Calculate a route from the current location to 'goal'
        """
        rospy.loginfo("Calculating route to x={}, y={}, z={}".format(
            goal.location.x,
            goal.location.y,
            goal.location.z))

        dao = GlobalRoutePlannerDAO(self.world.get_map(), sampling_resolution=1)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        route = grp.trace_route(self.ego_vehicle.get_location(),
                                carla.Location(goal.location.x,
                                               goal.location.y,
                                               goal.location.z))

        return route

    def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                quaternion = tf.transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))


def main():
    """
    main function
    """
    try:
        rospy.init_node("carla_waypoint_publisher", anonymous=True)
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Error while waiting for world info!")
        sys.exit(1)

    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)
    timeout = rospy.get_param("/carla/timeout", 10)

    rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        waypointConverter = CarlaToRosWaypointConverter(carla_world)

        rospy.spin()
        waypointConverter.destroy()
        del waypointConverter
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
