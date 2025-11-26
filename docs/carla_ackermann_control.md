# Carla Ackermann Control

[carlavehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#carlavehiclecontrolmsg

- [__Configuration__](#configuration)
- [__Testing control messages__](#testing-control-messages)
- [__ROS API__](#ros-api)
    - [Subscriptions](#subscriptions)
    - [Publications](#publications)

---

### Configuration

Parameters can be set both initially in a [configuration file][ackermanconfig] when using both ROS 1 and ROS 2 and during runtime via ROS [dynamic reconfigure][rosdynamicreconfig] in ROS 1. 

[ackermanconfig]: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/config/settings.yaml
[rosdynamicreconfig]: https://wiki.ros.org/dynamic_reconfigure

---

### Testing control messages

Test the setup by sending commands to the car via the topic `/carla/vehicles/<ROLE NAME>/ackermann_cmd`. For example, move an ego vehicle with the role name of `ego_vehicle` forward at a speed of 10 meters/sec by running this command:

```bash

# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

```

Or make the vehicle move forward while turning at an angle of 1.22 radians:

```bash

# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

```

---

### ROS API

#### Subscriptions

|Topic|Type|Description|
|--|--|--|
|`/carla/vehicles/<ROLE NAME>/ackermann_cmd` | [ackermann_msgs.AckermannDrive][ackermanncontrolmsg] | __Subscriber__ for steering commands |

<br>

#### Publications

|Topic|Type|Description|
|--|--|--|
| `/carla/vehicles/<ROLE NAME>/ackermann_control/control_info` | [carla_ackermann_control.EgoVehicleControlInfo][egovehiclecontrolmsg] | The current values used within the controller (useful for debugging) |

[egovehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#egovehiclecontrolinfomsg

<br>
