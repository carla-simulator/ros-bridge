# Carla Manual Control

The [CARLA manual control package](https://github.com/carla-simulator/ros-bridge/tree/master/carla_manual_control) is a ROS only version of the [`manual_control.py`][manualcontrol] script that comes packaged with CARLA. All data is received via ROS topics. 

[manualcontrol]: https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/manual_control.py

- [__Requirements__](#requirements)
- [__Run the package__](#run-the-package)
---

## Requirements

To be able to use `carla_manual_control`, some specific sensors need to be attached to the ego vehicle (see [Carla Spawn Objects](carla_spawn_objects.md) for information on how to attach sensors to vehicles):

- __to display an image__: a camera with role-name `rgb_view` and resolution 800x600.
- __to display the current position__: a GNSS sensor with role-name `gnss` and an odometry pseudo-sensor with role-name `odometry`.
- __to get a notification on lane invasions__: a lane invasion sensor with role-name `lane_invasion`.
- __to get a notification on collisons__: a collision sensor with role-name `collision`.

---

## Run the package

To run the package:
 
__ 1.__ Make sure you have CARLA runing. Start the ROS bridge:

```sh
        # ROS 1
        roslaunch carla_ros_bridge carla_ros_bridge.launch

        # ROS 2
        ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

__2.__ Spawn objects:

```sh
        # ROS 1
        roslaunch carla_spawn_objects carla_spawn_objects.launch

        # ROS 2
        ros2 launch carla_spawn_objects carla_spawn_objects.launch.py
```

__3.__ Launch the `carla_manual_control` node:

```sh
        # ROS 1
        roslaunch carla_manual_control carla_manual_control.launch

        # ROS 2
        ros2 launch carla_manual_control carla_manual_control.launch.py
```

__4.__ To steer the vehicle manually, press 'B'. Press 'H' to see instructions.

Alternatively, all of the above commands are combined into a separate, single launchfile and can be run at the same time by executing the following:

```sh
        # ROS 1
        roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

        # ROS 2
        ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```
---
