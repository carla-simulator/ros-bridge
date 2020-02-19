# RVIZ CARLA Control

A [RVIZ](http://wiki.ros.org/rviz) plugin to visualize/control CARLA.

![CARLA AD Demo](../docs/images/rviz_carla_plugin.png "rviz carla plugin")

## Features

### Provide the RVIZ view pose to other nodes

In combination with [carla_spectator_camera](../carla_spectator_camera), this allows visually moving around in the CARLA world.

Currently, it is limited to a camera attached to the ego-vehicle. Please set the target frame of the "Current View" to `ego_vehicle`.

### Visualize the current ego vehicle state

The current vehicle state is visualized:

- Vehicle Control
- Position

### Allows manually overriding the ego vehicle vehicle control

By using the drive-widget from the [RVIZ Visualization Tutorials](https://github.com/ros-visualization/visualization_tutorials) and a [node to convert from twist to vehicle control](../carla_twist_to_control) it is possible to steer the ego vehicle by mouse.

### Execute a scenario

By using [carla_ros_scenario_runner](../carla_ros_scenario_runner), it is possible to trigger scenarios from within RVIZ.


### Play/Pause the simulation (if started in synchronous mode)

Similar to the [rqt CARLA plugin](../rqt_carla_plugin), it's possible to control the CARLA world, if synchronous mode is active.