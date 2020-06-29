# Twist to Vehicle Control conversion

This node converts a [geometry_msgs.Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) to [carla_msgs.CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg)


## Subscriptions

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs.CarlaEgoVehicleInfo](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleInfo.msg) | Ego vehicle info, to receive max steering angle |
| `/carla/<ROLE NAME>/twist` | [geometry_msgs.Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) | Twist to convert |

## Publications

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs.CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg) | Converted vehicle control command |
