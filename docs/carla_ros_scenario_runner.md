# Carla ROS Scenario Runner

The [CARLA ROS Scenario Runner package](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ros_scenario_runner) is a wrapper to execute [OpenScenarios](https://www.asam.net/standards/detail/openscenario/) with the CARLA [Scenario Runner](https://github.com/carla-simulator/scenario_runner) via ROS.

- [__Before you begin__](#before-you-begin)
- [__Using ROS Scenario Runner__](#using-ros-scenario-runner)
- [__Run ROS Scenario Runner__](#run-ros-scenario-runner)
- [__ROS API__](#ros-api)
    - [Services](#services)
    - [Publications](#publications)

---

## Before you begin

- Follow the Scenario Runner ["Getting started'](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md) tutorial to install Scenario Runner.
- Install the Python module __Pexpect__:

```shell
sudo apt install python-pexpect
```
---

## Using ROS Scenario Runner

The ROS Scenario Runner is best used from within the [`rviz_carla_plugin`](rviz_plugin.md).

!!! Note
    It is currently not supported to change the map. Each scenario will need to use the currently active map.

An example scenario is found [here](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_demo/config/FollowLeadingVehicle.xosc). Of particular importance is the setup of the [ROS controller](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_demo/config/FollowLeadingVehicle.xosc#L78):

```xml
<Controller name="EgoVehicleAgent">
    <Properties>
        <Property name="module" value="carla_ros_scenario_runner.ros_vehicle_control" />
        <Property name="launch" value="carla_ad_agent.launch"/>
        <Property name="launch-package" value="carla_ad_agent"/>
        <Property name="path_topic_name" value="waypoints"/>
    </Properties>
</Controller>
```

The above code example shows an instance of [`carla_ad_agent`](carla_ad_agent.md) being launched. Any additional `<Property>` should be appended as a ROS parameter (name:=value).

---

## Run ROS Scenario Runner

__1.__ Run the ROS Scenario Runner package:

```sh
# ROS 1
roslaunch carla_ros_scenario_runner carla_ros_scenario_runner.launch scenario_runner_path:=<path_to_scenario_runner>

# ROS 2
ros2 launch carla_ros_scenario_runner carla_ros_scenario_runner.launch.py scenario_runner_path:=<path_to_scenario_runner>
```

__2.__ Run a scenario:

```sh
# ROS 1
rosservice call /scenario_runner/execute_scenario "{ 'scenario': { 'scenario_file': '<full_path_to_openscenario_file>' } }"

# ROS 2
ros2 service call /scenario_runner/execute_scenario carla_ros_scenario_runner_types/srv/ExecuteScenario "{ 'scenario': { 'scenario_file': '<full_path_to_openscenario_file>' } }"
```

---

## ROS API

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/scenario_runner/execute_scenario` | [`carla_ros_scenario_runner_types.ExecuteScenario`](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ros_scenario_runner_types/srv/ExecuteScenario.srv) | Execute a scenario. If another scenario is currently running, it gets stopped. |

<br>

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/scenario_runner/status` | [`carla_ros_scenario_runner_types.CarlaScenarioRunnerStatus`](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ros_scenario_runner_types/msg/CarlaScenarioRunnerStatus.msg) | The current status of the scenario runner execution (used by the [rviz_carla_plugin](rviz_plugin.md)) |


The controller `ros_vehicle_control` provides the following topics:
| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<ROLE NAME>/waypoints` | [`nav_msgs.Path`](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | the path defined within the scenario. Note: The topic name can be changed by modifying the parameter `path_topic_name` |
| `/carla/<ROLE NAME>/target_speed` | [`std_msgs.Float64`](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | the target speed as defined within the scenario |

<br>
