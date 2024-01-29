# CARLA Automatic Breaking Feature Demo

The [Automatic Breaking Feature Demo](https://github.com/ttgamage/carla-ros-bridge/tree/AutoBrakeFeature) is a CARLA simulation used to evaluate the behavior competency of Automatic Breaking Feature of automated vehicles.

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
      185 if self._sync_mode and self._running and self._watchdog.get_status():0
      186   CarlaDataProvider.get_world().tick()
      187 else:
      188   CarlaDataProvider.get_world().wait_for_tick()
  ```
---

## Run the demo

You will require 2 Linux Terminals to Run this demo

1. Terminal 01: Start CARLA Server
  ```sh
    sh $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen
  ```
2. Terminal 02: Launch the Demo
  ```sh
    ros2 launch carla_abf_demo carla_abf_demo.launch.py
  ```
  The demo can triggered by entering the target speed value and either hitting "Enter" or by clicking the "Go" button.

3. (optional) Terminal 03: Launch Spectator View
  ```sh
    ros2 launch carla_manual_control carla_manual_control.launch.py
  ```
It is also possible to start the Demo by directly setting the target Speed from the Terminal
  ```sh
    ros2 topic pub --once /carla/hero/target_speed std_msgs/msg/Float64 "{data: 21.0}" 
  ```

