# Carla Ackermann Control

ROS Node to convert [AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) messages to [CarlaEgoVehicleControl](carla_ros_bridge/msg/CarlaEgoVehicleControl.msg).

-   A PID controller is used to control the acceleration/velocity.
-   Reads the Vehicle Info, required for controlling from Carla (via carla ros bridge)

## Prerequisites

    #install python simple-pid
    pip install --user simple-pid

### Configuration

Initial parameters can be set via [configuration file](config/settings.yaml).

It is possible to modify the parameters during runtime via ROS dynamic reconfigure.

## Available Topics

| Topic                                               | Type                                                                                                 | Description                                                   |
| --------------------------------------------------- | ---------------------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| `/carla/<ROLE NAME>/ackermann_cmd` (subscriber)     | [ackermann_msgs.AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) | Subscriber for stearing commands                              |
| `/carla/<ROLE NAME>/ackermann_control/control_info` | [carla_ackermann_control.EgoVehicleControlInfo](msg/EgoVehicleControlInfo.msg)                       | The current values used within the controller (for debugging) |

The role name is specified within the configuration.

### Test control messages

You can send command to the car using the topic `/carla/<ROLE NAME>/ackermann_cmd`.

Examples for a ego vehicle with role_name 'ego_vehicle':

Forward movements, speed in in meters/sec.

     rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10

Forward with steering

     rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10

Info: the steering_angle is the driving angle (in radians) not the wheel angle.
