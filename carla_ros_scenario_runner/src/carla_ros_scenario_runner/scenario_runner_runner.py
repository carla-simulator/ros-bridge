#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
"""
Executes scenario runner
"""
import os
from application_runner import ApplicationRunner  # pylint: disable=relative-import


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
            "ScenarioManager: Running scenario OpenScenario")

    def execute_scenario(self, scenario_file):
        """
        Executes scenario
        """
        cmdline = ["/usr/bin/python", "{}/scenario_runner.py".format(self._path),
                   "--openscenario", "{}".format(scenario_file),
                   "--timeout", "1000000",
                   "--host", self._host,
                   "--port", str(self._port)]
        if self._wait_for_ego:
            cmdline.append("--waitForEgo")
        return self.execute(cmdline, env=os.environ)
