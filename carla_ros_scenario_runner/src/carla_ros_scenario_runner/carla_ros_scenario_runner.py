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
import json
import os
import sys
try:
    import queue
except ImportError:
    import Queue as queue
import rospy
from carla_ros_scenario_runner_types.srv import ExecuteScenario, ExecuteScenarioResponse
from carla_ros_scenario_runner_types.msg import (CarlaScenario,
                                                 CarlaScenarioList,
                                                 CarlaScenarioRunnerStatus)
from application_runner import ApplicationStatus  # pylint: disable=relative-import
from scenario_runner_runner import ScenarioRunnerRunner  # pylint: disable=relative-import

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


class CarlaRosScenarioRunner(object):
    """
    Execute scenarios via ros service
    """

    def __init__(self,
                 role_name,
                 host,
                 port,
                 scenario_runner_path,
                 available_scenarios_dir,
                 available_scenarios_json,
                 wait_for_ego):
        """
        Constructor
        """
        self._status_publisher = rospy.Publisher(
            "/scenario_runner/status", CarlaScenarioRunnerStatus, queue_size=1, latch=True)
        self.scenario_runner_status_updated(ApplicationStatus.STOPPED)

        self._scenario_publisher = rospy.Publisher(
            "/carla/available_scenarios", CarlaScenarioList, queue_size=1, latch=True)
        self.add_available_scenarios(available_scenarios_dir, available_scenarios_json)
        self._available_scenarios_dir = available_scenarios_dir

        self._scenario_runner = ScenarioRunnerRunner(
            scenario_runner_path,
            host,
            port,
            wait_for_ego,
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

    def add_available_scenarios(self, directory, json_file):
        """
        Add available scenarios
        """
        if not os.path.exists(json_file):
            rospy.logerr('Failed to load available_scenarios_json from: {}'.format(json_file))
            return
        rospy.loginfo("Adding available scenarios...")
        msg = CarlaScenarioList()
        json_scenarios = None
        with open(json_file) as handle:
            json_scenarios = json.loads(handle.read())
        for json_scenario in json_scenarios:
            scenario = CarlaScenario()
            scenario.name = json_scenario.get('name', '')
            scenario.scenario_file = json_scenario.get('scenario_file', '')
            msg.scenarios.append(scenario)
        self._scenario_publisher.publish(msg)

    def execute_scenario(self, req):
        """
        Execute a scenario
        """
        rospy.loginfo("Scenario Execution requested...")

        response = ExecuteScenarioResponse()
        response.result = True
        openscenario = os.path.join(self._available_scenarios_dir, req.scenario.scenario_file)
        if not os.path.isfile(openscenario):
            rospy.logwarn("Requested OpenSCENARIO file not existing {}".format(openscenario))
            response.result = False
        else:
            req.scenario.scenario_file = openscenario
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
                if not scenario_executed:
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
    available_scenarios_dir = rospy.get_param("~available_scenarios_dir", "")
    available_scenarios_json = rospy.get_param("~available_scenarios_json", "")
    wait_for_ego = rospy.get_param("~wait_for_ego", "True")
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 2000)

    scenario_runner = CarlaRosScenarioRunner(
        role_name,
        host,
        port,
        scenario_runner_path,
        available_scenarios_dir,
        available_scenarios_json,
        wait_for_ego)

    try:
        scenario_runner.run()
    finally:
        del scenario_runner
        rospy.loginfo("Done.")


if __name__ == "__main__":
    main()
