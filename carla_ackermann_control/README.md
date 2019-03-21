# Carla Control

ROS Node to send [AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) messages to Carla.

* A PID controller is used to control the acceleration/velocity.
* Where possible, it reads the Vehicle Info from Carla (via carla ros bridge)

# Prerequisites

    #install python simple-pid
    pip install --user simple-pid


## Test control messages
You can send command to the car using the topic ```/carla/ego_vehicle/ackermann_cmd```.

Example of forward movements, speed in in meters/sec.

     rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10


Example of forward with steering

     rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10

  Info: the steering_angle is the driving angle (in radians) not the wheel angle.

