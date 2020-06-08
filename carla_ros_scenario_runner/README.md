# ROS Scenario Runner

This is a wrapper to execute OpenScenarios with the CARLA [scenario runner](https://github.com/carla-simulator/scenario_runner) via ROS.

It is best used from within the [rviz_carla_plugin](../rviz_carla_plugin).

Currently it is not supported to switch the Carla Town. Therefore the scenario needs to use the currently active Town.

## Setup

Please follow the scenario runner [Getting Started](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md) and verify, that scenario_runner is working, if started manually.

Additionally, please execute:

    sudo apt install python-pexpect

## Startup

The environment variables are forwarded to scenario_runner, therefore set them to:

    export PYTHONPATH=$PYTHONPATH:<path_to_carla>/PythonAPI/carla-<carla_version_and_arch>.egg:<path_to_carla>/PythonAPI/carla/

To run the ROS node:

    roslaunch carla_ros_scenario_runner carla_ros_scenario_runner.launch scenario_runner_path:=<path_to_scenario_runner>

To run a scenario from commandline:

    rosservice call /scenario_runner/execute_scenario "{ 'scenario': { 'scenario_file': '<full_path_to_openscenario_file>' } }"


## Available services

| Service                                                     | Description | Type                                                         |
| ----------------------------------------------------------- | ----------- | -------------------------------------------------------------------- |
| `/scenario_runner/execute_scenario` | Execute a scenario. If another scenario is currently running, it gets stopped. | [carla_ros_scenario_runner_types.ExecuteScenario](../carla_ros_scenario_runner_types/srv/ExecuteScenario.srv) |


## Available topics


| Topic                                 | Description | Type                                                                 |
| ------------------------------------- | ----------- | -------------------------------------------------------------------- |
| `/scenario_runner/status`     | The current status of the scenario runner execution (e.g. used by the [rviz_carla_plugin](../rviz_carla_plugin)) | [carla_ros_scenario_runner_types.CarlaScenarioRunnerStatus](../carla_ros_scenario_runner_types/msg/CarlaScenarioRunnerStatus.msg) |

