# Carla Twist to Control

The [`carla_twist_to_control` package](https://github.com/carla-simulator/ros-bridge/tree/master/carla_twist_to_control) converts a [geometry_msgs.Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) to [carla_msgs.CarlaVehicleControl](ros_msgs.md#carlavehiclecontrolmsg).

---
## ROS API

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/vehicles/<ROLE NAME>/vehicle_info` | [`carla_msgs.CarlaVehicleInfo`](ros_msgs.md#carlavehicleinfomsg) | Ego vehicle info, to receive max steering angle. |
| `/carla/vehicles/<ROLE NAME>/twist` | [`geometry_msgs.Twist`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | Twist to convert. |

<br>

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/vehicles/<ROLE NAME>/control/vehicle_control_cmd` | [`carla_msgs.CarlaVehicleControl`](ros_msgs.md#carlavehiclecontrolmsg) | Converted vehicle control command. |

<br>
