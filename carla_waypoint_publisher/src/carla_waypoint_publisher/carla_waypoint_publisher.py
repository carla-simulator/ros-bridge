#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed spawnpoint is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'
"""
import math
import threading

import rospy
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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
        self.world = carla_world
        self.map = carla_world.get_map()
        self.ego_vehicle = None
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self.waypoint_publisher = rospy.Publisher(
            '/carla/{}/waypoints'.format(self.role_name), Path, queue_size=1, latch=True)

        # set initial goal
        self.goal = self.world.get_map().get_spawn_points()[0]

        self.current_route = None
        self.goal_subscriber = rospy.Subscriber(
            "/carla/{}/goal".format(self.role_name), PoseStamped, self.on_goal)

        self._update_lock = threading.Lock()

        # use callback to wait for ego vehicle
        rospy.loginfo("Waiting for ego vehicle...")
        self.world.on_tick(self.find_ego_vehicle_actor)

    def on_goal(self, goal):
        """
        Callback for /move_base_simple/goal

        Receiving a goal (e.g. from RVIZ '2D Nav Goal') triggers a new route calculation.

        :return:
        """
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

    def calculate_route(self, goal):
        """
        Calculate a route from the current location to 'goal'
        """
        rospy.loginfo("Calculating route to x={}, y={}, z={}".format(
            goal.location.x,
            goal.location.y,
            goal.location.z))

        dao = GlobalRoutePlannerDAO(self.world.get_map())
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
    rospy.init_node("carla_waypoint_publisher", anonymous=True)

    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        waypointConverter = CarlaToRosWaypointConverter(carla_world)

        rospy.spin()
        del waypointConverter
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
