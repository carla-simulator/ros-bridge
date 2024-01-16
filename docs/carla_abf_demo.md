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

---

## Run the demo

You will require 3 Linux Terminals to Run this demo

1. Terminal 01: Start CARLA Server
  ```sh
    sh $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen
  ```
2. Terminal 02: Launch the Demo
  ```sh
    ros2 launch carla_abf_demo carla_abf_demo.launch.py
  ```
3. Terminal 03: Start the Demo (by setting Ego Vehicle's Target Speed)
  ```sh
    ros2 topic pub --once /carla/hero/target_speed std_msgs/msg/Float64 "{data: 21.0}" 
  ```
4. (optional) Terminal 04: Launch Spectator View
  ```sh
    ros2 launch carla_manual_control carla_manual_control.launch.py
  ```
