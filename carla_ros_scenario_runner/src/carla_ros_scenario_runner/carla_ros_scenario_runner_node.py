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
import logging
try:
    import queue
except ImportError:
    import Queue as queue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from carla_ros_scenario_runner_types.srv import ExecuteScenario
from carla_ros_scenario_runner_types.msg import CarlaScenarioRunnerStatus
from carla_ros_scenario_runner.application_runner import ApplicationStatus  # pylint: disable=relative-import
from carla_ros_scenario_runner.scenario_runner_runner import ScenarioRunnerRunner  # pylint: disable=relative-import

from ros_compatibility import CompatibleNode, QoSProfile, ros_ok

# Check Python dependencies of scenario runner
try:
    import carla  # pylint: disable=unused-import
except ImportError:
    print("ERROR: CARLA Python Egg not available. Please add \
        <CARLA_DIR>/PythonAPI/carla/dist/carla-<CARLA_VERSION>-\
        py<PYTHON_VERSION>-linux-x86_64.egg to your PYTHONPATH.")
    sys.exit(1)

try:
    from agents.navigation.local_planner import LocalPlanner  # pylint: disable=unused-import
except ImportError:
    print("ERROR: CARLA Python Agents not available. \
        Please add <CARLA_DIR>/PythonAPI/carla to your PYTHONPATH.")
    sys.exit(1)

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    from carla_ros_scenario_runner_types.srv import ExecuteScenarioResponse
elif ROS_VERSION == 2:
    import rclpy
    import threading


class CarlaRosScenarioRunner(CompatibleNode):
    """
    Execute scenarios via ros service
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaRosScenarioRunner, self).__init__('carla_ros_scenario_runner')

        role_name = self.get_param("role_name", "ego_vehicle")
        scenario_runner_path = self.get_param("scenario_runner_path", "")
        wait_for_ego = self.get_param("wait_for_ego", "True")
        host = self.get_param("host", "localhost")
        port = self.get_param("port", 2000)

        self._status_publisher = self.new_publisher(
            CarlaScenarioRunnerStatus, "/scenario_runner/status",
            qos_profile=QoSProfile(depth=1, durability=1))
        self.scenario_runner_status_updated(ApplicationStatus.STOPPED)
        self._scenario_runner = ScenarioRunnerRunner(
            scenario_runner_path,
            host,
            port,
            wait_for_ego,
            self.scenario_runner_status_updated,
            self.scenario_runner_log)
        self._request_queue = queue.Queue()
        self._execute_scenario_service = self.new_service(
            ExecuteScenario, '/scenario_runner/execute_scenario', self.execute_scenario)

    def scenario_runner_log(self, log):  # pylint: disable=no-self-use
        """
        Callback for application logs
        """
        self.logwarn("[SC]{}".format(log))

    def scenario_runner_status_updated(self, status):
        """
        Executed from application runner whenever the status changed
        """
        self.loginfo("Status updated to {}".format(status))
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

    def execute_scenario(self, req, response=None):
        """
        Execute a scenario
        """
        self.loginfo("Scenario Execution requested...")

        if ROS_VERSION == 1:
            response = ExecuteScenarioResponse()
        response.result = True
        if not os.path.isfile(req.scenario.scenario_file):
            self.logwarn("Requested scenario file not existing {}".format(
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
        while ros_ok():
            if current_req:
                if self._scenario_runner.is_running():
                    self.loginfo("Scenario Runner currently running. Shutting down.")
                    self._scenario_runner.shutdown()
                    self.loginfo("Scenario Runner stopped.")
                self.loginfo("Executing scenario {}...".format(current_req.name))

                # execute scenario
                scenario_executed = self._scenario_runner.execute_scenario(
                    current_req.scenario_file)
                if not scenario_executed:
                    self.logwarn("Unable to execute scenario.")
                current_req = None
            else:
                try:
                    current_req = self._request_queue.get(block=True, timeout=0.5)
                except queue.Empty:
                    # no new request
                    pass

        if self._scenario_runner.is_running():
            self.loginfo("Scenario Runner currently running. Shutting down.")
            self._scenario_runner.shutdown()


def main():
    """

    main function

    :return:
    """
    if ROS_VERSION == 2:
        rclpy.init()

    scenario_runner = CarlaRosScenarioRunner()

    if ROS_VERSION == 2:
        spin_thread = threading.Thread(target=scenario_runner.spin, daemon=True)
        spin_thread.start()

    try:
        scenario_runner.run()
    finally:
        if scenario_runner._scenario_runner.is_running():
            scenario_runner.loginfo("Scenario Runner still running. Shutting down.")
            scenario_runner._scenario_runner.shutdown()
        del scenario_runner
        logging.info("Done.")


if __name__ == "__main__":
    main()
