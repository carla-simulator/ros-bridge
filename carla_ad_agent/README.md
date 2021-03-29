# CARLA AD Agent

An AD agent that can follow a given route.

It avoids crashs with other vehicles and respects the state of the traffic lights by accessing the ground truth data.

For a more comprehensive solution, have a look at [Autoware](https://www.autoware.ai/).

## Subscriptions

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/waypoints` | [nav_msgs.Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html) | Route to follow |
| `/carla/<ROLE NAME>/target_speed` | [std_msgs.Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) | Target speed |
| `/carla/<ROLE NAME>/odometry` | [nav_msgs.Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | localization of ego vehicle |

For risk avoidance, more subscriptions are required:

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs.CarlaEgoVehicleInfo](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleInfo.msg) |  Identify the carla actor id of the ego vehicle |
| `/carla/<ROLE NAME>/objects` | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) | Information about other actors |
| `/carla/actor_list` | [carla_msgs.CarlaActorList](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaActorList.msg) | Actor list |
| `/carla/world_info` | [carla_msgs.CarlaWorldInfo](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaWorldInfo.msg) | Selects mode for traffic lights (US- or Europe-style) |
| `/carla/traffic_lights/status` | [carla_msgs.CarlaTrafficLightStatusList](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaTrafficLightStatusList.msg) | Get the current state of the traffic lights |

## Publications

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs.CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg) | Vehicle control command |

## Local Planner Node

Internally, the CARLA AD Agent uses a separate node for [local planning](src/carla_ad_agent/local_planner.py).

This is currently optimized for `vehicle.tesla.model3`, as it does not have any gear shift delays.

The PID parameters were gathered by [Ziegler-Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method).
