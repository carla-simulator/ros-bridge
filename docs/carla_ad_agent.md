# CARLA AD Agent

The [CARLA AD agent](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_agent) is an AD agent that can follow a given route, avoids crashes with other vehicles and respects the state of traffic lights by accessing ground truth data. It is used by the [CARLA AD demo](carla_ad_demo.md) to provide an example of how the ROS bridge can be used.

- [__Local Planner Node__](#local-planner-node)
- [__ROS API__](#ros-api)
    - [Subscriptions](#subscriptions)
    - [Publications](#publications)

---

## Local Planner Node

Internally the CARLA AD Agent uses a separate node for [local planning](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_agent/src/carla_ad_agent/local_planner.py). This node has been optimized for the `vehicle.tesla.model3`, as it does not have any gear shift delays.

The PID parameters were gathered by [Ziegler-Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method).

## ROS API 

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<ROLE NAME>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | Route to follow |
| `/carla/<ROLE NAME>/target_speed` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Target speed |
| `/carla/<ROLE NAME>/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | Localization of ego vehicle |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs/CarlaEgoVehicleInfo](ros_msgs.md#carlaegovehicleinfomsg) | Identify the carla actor id of the ego vehicle |
| `/carla/<ROLE NAME>/objects` | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | Information about other actors |
| `/carla/actor_list` | [carla_msgs/CarlaActorList](ros_msgs.md#carlaactorlistmsg) | Actor list |
| `/carla/world_info` | [carla_msgs/CarlaWorldInfo](ros_msgs.md#carlaworldinfomsg) | Selects mode for traffic lights (US- or European-style) |
| `/carla/traffic_lights/status` | [carla_msgs/CarlaTrafficLightStatusList](ros_msgs.md#carlatrafficlightstatuslistmsg) | Get the current state of the traffic lights |

<br>

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs/CarlaEgoVehicleControl](ros_msgs.md#carlaegovehiclecontrolmsg) | Vehicle control command |

<br>
