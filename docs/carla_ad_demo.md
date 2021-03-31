# CARLA AD Demo

The [AD demo](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_demo) is an example package that provides everything needed to launch a CARLA ROS environment with an AD vehicle. 

- [__Before you begin__](#before-you-begin)
- [__Run the demo__](#run-the-demo)
    - [Random route](#random-route)
    - [Scenario execution](#scenario-execution)
---

## Before you begin

Install [Scenario Runner](https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/) and follow the Scenario Runner ["Getting Started" tutorial](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md) to verify that it's working. 

Set an environment variable to find the Scenario Runner installation:

```sh
export SCENARIO_RUNNER_PATH=<path_to_scenario_runner>
```

---

## Run the demo


#### Random route

To start a demo where the ego vehicle follows a randomly generated route, run the following command after you have started a CARLA server:

```sh
# ROS 1
roslaunch carla_ad_demo carla_ad_demo.launch

# ROS 2
ros2 launch carla_ad_demo carla_ad_demo.launch.py
```

You can also spawn additional vehicles or pedestrians by executing the following command in another terminal:

```sh
cd <CARLA_PATH>/PythonAPI/examples/

python3 spawn_npc.py
```

#### Scenario execution

To execute the demo with a predefined scenario, run the following command after you have started a CARLA server:

```sh
# ROS1
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch

# ROS2
ros2 launch carla_ad_demo carla_ad_demo_with_scenario.launch.py
```

Select the example scenario "FollowLeadingVehicle" within the RVIZ Carla Plugin and press "Execute". The ego vehicle gets repositioned and the scenario is processed. 

You can specify your own scenarios by publishing to `/carla/available_scenarios`. The [launchfile](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch) shows an example of how to do this:

```launch
  <node pkg="rostopic" type="rostopic" name="publish_scenarios"
    args="pub /carla/available_scenarios carla_ros_scenario_runner_types/CarlaScenarioList '{ 'scenarios':  
      [
        {
          'name': 'FollowLeadingVehicle',
          'scenario_file': '$(find carla_ad_demo)/config/FollowLeadingVehicle.xosc'
        }
      ]
    }' -l"/>
```

---



