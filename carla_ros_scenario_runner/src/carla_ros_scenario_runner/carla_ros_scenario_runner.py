#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Execute scenarios via ros service

Internally, the CARLA scenario runner is executed
"""
import sys
import os
try:
    import queue
except ImportError:
    import Queue as queue
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from carla_ros_scenario_runner_types.srv import ExecuteScenario, ExecuteScenarioResponse
from carla_ros_scenario_runner_types.msg import CarlaScenarioRunnerStatus
from application_runner import ApplicationStatus
from scenario_runner_runner import ScenarioRunnerRunner

# Check Python dependencies of scenario runner
try:
    import carla  # pylint: disable=unused-import
except ImportError:
    print "ERROR: CARLA Python Egg not available. Please add \
        <CARLA_DIR>/PythonAPI/carla/dist/carla-<CARLA_VERSION>-\
        py<PYTHON_VERSION>-linux-x86_64.egg to your PYTHONPATH."
    sys.exit(1)

try:
    from agents.navigation.local_planner import LocalPlanner  # pylint: disable=unused-import
except ImportError:
    print "ERROR: CARLA Python Agents not available. \
        Please add <CARLA_DIR>/PythonAPI/carla to your PYTHONPATH."
    sys.exit(1)


class CarlaRosScenarioRunner(object):
    """
    Execute scenarios via ros service
    """

    def __init__(self, role_name, host, scenario_runner_path, publish_waypoints, publish_goal):
        """
        Constructor
        """
        self._goal_publisher = None
        if publish_goal:
            self._goal_publisher = rospy.Publisher(
                "/carla/{}/goal".format(role_name), PoseStamped, queue_size=1, latch=True)
        self._target_speed_publisher = rospy.Publisher(
            "/carla/{}/target_speed".format(role_name), Float64, queue_size=1, latch=True)

        self._path_publisher = None
        if publish_waypoints:
            self._path_publisher = rospy.Publisher(
                "/carla/{}/waypoints".format(role_name), Path, queue_size=1, latch=True)

        self._status_publisher = rospy.Publisher(
            "/scenario_runner/status", CarlaScenarioRunnerStatus, queue_size=1, latch=True)
        self.scenario_runner_status_updated(ApplicationStatus.STOPPED)
        self._scenario_runner = ScenarioRunnerRunner(
            scenario_runner_path,
            host,
            self.scenario_runner_status_updated,
            self.scenario_runner_log)
        self._request_queue = queue.Queue()
        self._execute_scenario_service = rospy.Service(
            '/scenario_runner/execute_scenario', ExecuteScenario, self.execute_scenario)

    def scenario_runner_log(self, log):  # pylint: disable=no-self-use
        """
        Callback for application logs
        """
        rospy.logwarn("[SC]{}".format(log))

    def scenario_runner_status_updated(self, status):
        """
        Executed from application runner whenever the status changed
        """
        rospy.loginfo("Status updated to {}".format(status))
        val = CarlaScenarioRunnerStatus.STOPPED
        if status == ApplicationStatus.STOPPED:
            val = CarlaScenarioRunnerStatus.STOPPED
        elif status == ApplicationStatus.STARTING:
            val = CarlaScenarioRunnerStatus.STARTING
        elif status == ApplicationStatus.RUNNING:
            val = CarlaScenarioRunnerStatus.RUNNING
        elif status == ApplicationStatus.SHUTTINGDOWN:
            val = CarlaScenarioRunnerStatus.SHUTTINGDOWN
        else:
            val = CarlaScenarioRunnerStatus.ERROR
        status = CarlaScenarioRunnerStatus()
        status.status = val
        self._status_publisher.publish(status)

    def execute_scenario(self, req):
        """
        Execute a scenario
        """
        rospy.loginfo("Scenario Execution requested...")

        response = ExecuteScenarioResponse()
        response.result = True
        if not os.path.isfile(req.scenario.scenario_file):
            rospy.logwarn("Requested scenario file not existing {}".format(
                req.scenario.scenario_file))
            response.result = False
        else:
            self._request_queue.put(req.scenario)
        return response

    def run(self):
        """
        Control loop

        :return:
        """
        current_req = None
        while not rospy.is_shutdown():
            if current_req:
                if self._scenario_runner.is_running():
                    rospy.loginfo("Scenario Runner currently running. Shutting down.")
                    self._scenario_runner.shutdown()
                    rospy.loginfo("Scenario Runner stopped.")
                rospy.loginfo("Executing scenario {}...".format(current_req.name))

                # execute scenario
                scenario_executed = self._scenario_runner.execute_scenario(
                    current_req.scenario_file)
                if scenario_executed:
                    # publish target speed
                    self._target_speed_publisher.publish(Float64(data=current_req.target_speed))

                    # publish last pose of route as goal
                    # (can be used in conjunction with carla_waypoint_publisher)
                    if self._goal_publisher:
                        goal = PoseStamped()
                        if current_req.waypoints:
                            goal.pose = current_req.waypoints[-1]
                        goal.header.stamp = rospy.get_rostime()
                        goal.header.frame_id = "map"
                        self._goal_publisher.publish(goal)

                    # publish the waypoints (can directly be used within carla_ad_agent)
                    if self._path_publisher:
                        path = Path()
                        path.header.stamp = rospy.get_rostime()
                        path.header.frame_id = "map"
                        for pose in current_req.waypoints:
                            path.poses.append(PoseStamped(pose=pose))
                        self._path_publisher.publish(path)
                else:
                    rospy.logwarn("Unable to execute scenario.")
                current_req = None
            else:
                try:
                    current_req = self._request_queue.get(block=True, timeout=0.5)
                except queue.Empty:
                    # no new request
                    pass
        if self._scenario_runner.is_running():
            rospy.loginfo("Scenario Runner currently running. Shutting down.")
            self._scenario_runner.shutdown()


def main():
    """

    main function

    :return:
    """
    rospy.init_node('carla_ros_scenario_runner', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    scenario_runner_path = rospy.get_param("~scenario_runner_path", "")
    host = rospy.get_param("~host", "localhost")
    publish_waypoints = rospy.get_param("~publish_waypoints", False)
    publish_goal = rospy.get_param("~publish_goal", True)
    scenario_runner = CarlaRosScenarioRunner(
        role_name, host, scenario_runner_path, publish_waypoints, publish_goal)
    try:
        scenario_runner.run()
    finally:
        del scenario_runner
        rospy.loginfo("Done.")


if __name__ == "__main__":
    main()
