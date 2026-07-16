# Derived Objects Visualizer

The [CARLA derived objects visualizer package](https://github.com/carla-simulator/ros-bridge/tree/master/derived_objects_visualizer) is a ros node providing a marker_array topic for derived object with covariance topics e.g. from CARLA.

- [__Requirements__](#requirements)
- [__ROS API__](#ros-api)
    - [__derived objects visualizer Node__](#derived-objects-visualizer-node)
        - [Parameters](#parameters)
- [__Run the package__](#run-the-package)

---

## Requirements

There is no special requirements on running the node besides starting CARLA with native ROS2 support enabled.
---

## ROS API

### derived objects visualizer Node

#### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `objects_topic` | string (default: `/carla/objects_with_covariance`) | CARLA topic of the objects to be visualized |


## Run the package

To run the package:
 
__ 1.__ Make sure you have CARLA runing. Start the package:

```sh
        ros2 launch derived_objects_visualizer derived_objects_visualizer.launch.py
```
... and add the marker topic to your rviz to see the results.
