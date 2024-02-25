# CARLA Automatic Emergency Braking Feature Demo

The [Automatic Emergency Braking Feature Demo](https://github.com/ttgamage/carla-ros-bridge/tree/AutoBrakeFeature) is a CARLA simulation used to evaluate the behavior competency of Automatic Emergency Braking Feature of automated vehicles.

- [__Before you begin__](#before-you-begin)
- [__Run the demo__](#run-the-demo)
---

## Before you begin

- Follow the instructions found in [Installing ROS bridge for ROS 2](ros_installation_ros2.md) and make sure the CARLA Simulator, Scenario Runner, and ROS 2 is setup properly. 
- Check and ensure that the SCENARIO_RUNNER_ROOT environmental variable is set.
  ```sh
    echo $SCENARIO_RUNNER_ROOT
  ```
  The output of the above command should list your Scenario Runner directory.
- Ensure scenario runner is executed in passive mode (i.e. `--sync` is not set). This is the default behavior, unless explicitly changed.
- __!!! BUG NOTICE:__ In the latest version of Scenario Runner (v.0.9.15), there doesn't appear to be a condition to `wait_for_tick()` when running in passive mode. Because of this, every other sensor frame get's skipped, affecting the accuracy and the performance of the simulation. To fix the issue, modify the code as follows:
  ```python
      185 if self._sync_mode and self._running and self._watchdog.get_status():
      186   CarlaDataProvider.get_world().tick()
      187 else:
      188   CarlaDataProvider.get_world().wait_for_tick()
  ```
---

## Integrate with ns-3

1. Clone the feature/ROS2 branch of the UCEFwithNS3 repository at https://github.com/tpr1/UCEFwithNS3/tree/feature/ROS2

2. Install the ns-3 requirements:
```
  sudo apt install cmake
  sudo apt install build-essential
  sudo apt install libsqlite3-dev
```

3. Compile the ns-3 code from the UCEFwithNS3/av-ns3 directory:
```
  ./ns3 configure --enable-examples
  ./ns3 build
```

## Run the demo

You will require 4 Linux Terminals to Run this demo

1. Terminal 01: Start CARLA Server
  ```sh
    sh $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen
  ```

2. Terminal 02: Launch the ROS2 Nodes
  ```sh
    ros2 launch carla_abf_demo carla_abf_demo.launch.py
  ```

3. Terminal 03: Run ns-3
  ```sh
    cd UCEFwithNS3/av-ns3
    ./ns3 run automated-vehicles
  ```
  The ns-3 code must run after ns3_ros_bridge completes initialization. This is launched during the carla_abf_demo with the other ROS2 nodes. In general, it will be initialized by the time the rviz GUI loads.

4. Terminal 04: Publish the Target Speed
  ```sh
    ros2 topic pub --once /carla/hero/target_speed std_msgs/msg/Float64 "{data: 21.0}"
  ```
