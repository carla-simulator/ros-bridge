# CARLA AD Agent

The [CARLA AD agent](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_agent) is an AD agent that can follow a given route, avoids crashes with other vehicles and respects the state of traffic lights by accessing ground truth data. It is used by the [CARLA AD demo](carla_ad_demo.md) to provide an example of how the ROS bridge can be used.

- [__Requirements__](#requirements)
- [__ROS API__](#ros-api)
    - [__AD Agent Node__](#ad-agent-node)
        - [Parameters](#parameters)
        - [Subscriptions](#subscriptions)
        - [Publications](#publications)
    - [__Local Planner Node__](#local-planner-node)
        - [Parameters](#parameters)
        - [Subscriptions](#subscriptions)
        - [Publications](#publications)

Internally the CARLA AD Agent uses a separate node for [local planning](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_agent/src/carla_ad_agent/local_planner.py). This node has been optimized for the `vehicle.tesla.model3`, as it does not have any gear shift delays.

The PID parameters were gathered by [Ziegler-Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method).

---


## ROS API 

### AD Agent Node

#### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `role_name` | string (default: `ego_vehicle`) | CARLA role name of the ego vehicle |
| `avoid_risk` | bool (default: `true`) | If True, avoids crashes with other vehicles and respects traffic lights  |

<br>

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/vehicles/<ROLE NAME>/target_speed` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Target speed of the ego vehicle |
| `/carla/vehicles/<ROLE NAME>/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | Odometry of the ego vehicle |
| `/carla/vehicles/<ROLE NAME>/vehicle_info` | [carla_msgs/CarlaVehicleInfo](ros_msgs.md#carlavehicleinfomsg) | Identify the CARLA actor id of the ego vehicle |
| `/carla/vehicles/<ROLE NAME>/objects` | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | Information about other actors |
| `/carla/traffic_lights/status` | [carla_msgs/CarlaTrafficLightStatusList](ros_msgs.md#carlatrafficlightstatuslistmsg) | Get the current state of the traffic lights |
| `/carla/traffic_lights/info` | [carla_msgs/CarlaTrafficLightInfoList](ros_msgs.md#carlatrafficlightinfolistmsg) | Get info about traffic lights |

<br>

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/vehicles/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Target speed |

<br>

### Local Planner Node

#### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `role_name` | string (default: `ego_vehicle`) | CARLA role name of the ego vehicle |
| `control_time_step` | float (default: `0.05`) | Control loop rate |
| `Kp_lateral` | float (default `0.9`) | Proportional term lateral PID controller |
| `Ki_lateral` | float (default `0.0`) | Integral term lateral PID controller |
| `Kd_lateral` | float (default `0.0`) | Derivative term lateral PID controller |
| `Kp_longitudinal` | float (default `0.206`) | Proportional term longitudinal PID controller |
| `Ki_longitudinal` | float (default `0.0206`) | Integral term longitudinal PID controller |
| `Kd_longitudinal` | float (default `0.515`) | Derivative term longitudinal PID controller |

<br>

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/vehicles/<ROLE NAME>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | Route to follow |
| `/carla/vehicles/<ROLE NAME>/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | Odometry of the ego vehicle |
| `/carla/vehicles/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | Target speed |

<br>

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/vehicles/<ROLE NAME>/next_target` | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | Next target pose marker |
| `/carla/vehicles/<ROLE NAME>/control/vehicle_control_cmd` | [carla_msgs/CarlaVehicleControl](ros_msgs.md#carlavehiclecontrolmsg) | Vehicle control command |

<br>
