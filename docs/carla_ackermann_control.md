# Carla Ackermann Control

[carlavehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#carlavehiclecontrolmsg

- [__Configuration__](#configuration)
- [__Testing control messages__](#testing-control-messages)
- [__ROS API__](#ros-api)
    - [Subscriptions](#subscriptions)
    - [Publications](#publications)

---

### Configuration

Ackermann control messages are supported as control input from CARLA natively.

---

### Testing control messages

Test the setup by sending commands to the car via the topic `/carla/<ROLE NAME>/ackermann_cmd`. For example, move an ego vehicle with the role name of `ego_vehicle` forward at a speed of 10 meters/sec by running this command:

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
|`/carla/<ROLE NAME>/ackermann_cmd` | [ackermann_msgs.AckermannDrive][ackermanncontrolmsg] | __Subscriber__ for steering commands |

<br>

#### Publications

|Topic|Type|Description|
|--|--|--|
| `/carla/<ROLE NAME>/vehicle_status` | [carla_msgs.CarlaEgoVehicleStatus][carlaegovehiclestatusmsg] | Contains the current values used within the controller (useful for debugging) |

[carlaegovehiclestatusmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#carlaegovehiclestatusmsg

<br>
