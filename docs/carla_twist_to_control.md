# Carla Twist to Control

The [`carla_twist_to_control` package](https://github.com/carla-simulator/ros-bridge/tree/master/carla_twist_to_control) converts a [geometry_msgs.Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) to [carla_msgs.CarlaEgoVehicleControl](ros_msgs.md#carlaegovehiclecontrolmsg).

---
## ROS API

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<ROLE NAME>/vehicle_info` | [`carla_msgs.CarlaEgoVehicleInfo`](ros_msgs.md#carlaegovehicleinfomsg) | Ego vehicle info, to receive max steering angle. |
| `/carla/<ROLE NAME>/twist` | [`geometry_msgs.Twist`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | Twist to convert. |

<br>

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [`carla_msgs.CarlaEgoVehicleControl`](ros_msgs.md#carlaegovehiclecontrolmsg) | Converted vehicle control command. |

<br>
