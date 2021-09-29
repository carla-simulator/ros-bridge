#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
"""
Executes scenario runner
"""
import os

import ros_compatibility as roscomp

from carla_ros_scenario_runner.application_runner import ApplicationRunner  # pylint: disable=relative-import

ROS_VERSION = roscomp.get_ros_version()


class ScenarioRunnerRunner(ApplicationRunner):
    """
    Executes scenario runner
    """

    def __init__(self, path, host, port, wait_for_ego, status_updated_fct, log_fct):  # pylint: disable=too-many-arguments
        self._path = path
        self._host = host
        self._port = port
        self._wait_for_ego = wait_for_ego
        super(ScenarioRunnerRunner, self).__init__(
            status_updated_fct,
            log_fct,
            "ScenarioManager: Running scenario ")

    def execute_scenario(self, scenario_file):
        """
        Executes scenario
        """
        if ROS_VERSION == 1:
            python_path = "/usr/bin/python"
        elif ROS_VERSION == 2:
            python_path = "/usr/bin/python3"
        cmdline = [python_path, "{}/scenario_runner.py".format(self._path),
                   "--openscenario", "{}".format(scenario_file),
                   "--timeout", "1000000",
                   "--host", self._host,
                   "--port", str(self._port)]
        if self._wait_for_ego:
            cmdline.append("--waitForEgo")
        return self.execute(cmdline, env=os.environ)
