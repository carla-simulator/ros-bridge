# CARLA AD Agent

An AD agent that can follow a given route.

It avoids crashs with other vehicles and respects the state of the traffic lights.

For a more comprehensive solution, have a look at [Autoware](https://www.autoware.ai/).

## Publications

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs.CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg) | Vehicle control command |
