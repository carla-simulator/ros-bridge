#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
"""
Executes scenario runner
"""
import os
from application_runner import ApplicationRunner


class ScenarioRunnerRunner(ApplicationRunner):
    """
    Executes scenario runner
    """

    def __init__(self, path, host, status_updated_fct, log_fct):
        self._path = path
        self._host = host
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
                   "--waitForEgo",
                   "--timeout", "1000000",
                   "--host", self._host]
        return self.execute(cmdline, env=os.environ)
