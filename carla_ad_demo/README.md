# CARLA AD Demo

This meta package provides everything to launch a CARLA ROS environment with an AD vehicle.


![CARLA AD Demo](../docs/images/rviz_carla_plugin.png "rviz carla plugin")

The Node setup is visualized [here](../docs/images/ad_demo.png "AD Demo Node Setup")

## Startup

    export PYTHONPATH=$PYTHONPATH:<path_to_carla>/PythonAPI/carla-<carla_version_and_arch>.egg:<path_to_carla>/PythonAPI/carla/
    export SCENARIO_RUNNER_PATH=<path_to_scenario_runner>
    roslaunch carla_ad_demo carla_ad_demo.launch

### Modes

#### Following A Random Route

On startup, an ego vehicle is spawned and follows a random route to a goal.

You might want to spawn additional vehicles (or pedestrians) by manually executing:

    <CARLA_PATH>/PythonAPI/examples/spawn_npc.py

You can modify start position and goal within the [launch file](launch/carla_ad_demo.launch). The route is currently randomly (regarding the left/right turns) calculated.

#### Scenario Execution

If you prefer to execute a predefined scenario, launch:

    roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch
    
Select to example scenario "FollowLeadingVehicle" within the RVIZ Carla Plugin and press "Execute". The ego vehicle gets repositioned and the scenario is processed.

You can specify your own scenarios by publishing to `/carla/available_scenarios`. See the [launch file](launch/carla_ad_demo_with_scenario.launch) for an example.


##### Troubleshooting

If the example scenario fails, please analyze the ros log and follow the scenario runner [Getting Started](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md) to verify that it's working standalone.
