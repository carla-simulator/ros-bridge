# ROS Scenario Runner

This is a wrapper to execute OpenScenarios with the CARLA [scenario runner](https://github.com/carla-simulator/scenario_runner) via ROS.

It is best used from within the [rviz_carla_plugin](../rviz_carla_plugin).

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

    rosservice call /scenario_runner/execute_scenario "{ 'scenario': { 'scenario_file': '<full_path_to_openscenario_file>', 'target_speed': 10.0, 'destination': { 'position': { 'x': 10, 'y': 10, 'z':0 } } } }"


## Available services

| Service                                                     | Description | Type                                                         |
| ----------------------------------------------------------- | ----------- | -------------------------------------------------------------------- |
| `/scenario_runner/execute_scenario` | Execute a scenario. If another scenario is currently running, it gets stopped. | [carla_ros_scenario_runner_types.ExecuteScenario](../carla_ros_scenario_runner_types/srv/ExecuteScenario.srv) |


## Available topics


| Topic                                 | Description | Type                                                                 |
| ------------------------------------- | ----------- | -------------------------------------------------------------------- |
| `/carla/<ROLE_NAME>/target_speed`     | The ego vehicle target speed | [std_msgs.Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) |
| `/carla/<ROLE_NAME>/goal`     | The ego vehicle destination (e.g. used by [carla_waypoint_publisher](../carla_waypoint_publisher)) | [geometry_msgs.PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) |
| `/scenario_runner/status`     | The current status of the scenario runner execution (e.g. used by the [rviz_carla_plugin](../rviz_carla_plugin)) | [carla_ros_scenario_runner_types.CarlaScenarioRunnerStatus](../carla_ros_scenario_runner_types/msg/CarlaScenarioRunnerStatus.msg) |

