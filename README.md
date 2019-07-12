
# ROS bridge for CARLA simulator

This ROS package aims at providing a simple ROS bridge for CARLA simulator.

__Important Note:__
This documentation is for CARLA versions *newer* than 0.9.4.

![rviz setup](./docs/images/rviz_carla_default.png "rviz")
![depthcloud](./docs/images/depth_cloud_and_lidar.png "depthcloud")

![short video](https://youtu.be/S_NoN2GBtdY)


# Features

- [x] Cameras (depth, segmentation, rgb) support
- [x] Transform publications
- [x] Manual control using ackermann msg
- [x] Handle ROS dependencies
- [x] Marker/bounding box messages for cars/pedestrian
- [x] Lidar sensor support
- [x] Support CARLA synchronous mode
- [ ] Add traffic light support

# Setup

## Create a catkin workspace and install carla_ros_bridge package

    #setup folder structure
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
    cd ~/carla-ros-bridge
    git clone https://github.com/carla-simulator/ros-bridge.git
    cd catkin_ws/src
    ln -s ../../ros-bridge
    source /opt/ros/kinetic/setup.bash
    cd ..

    #install required ros-dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src -r

    #build
    catkin_make

For more information about configuring a ROS environment see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

# Start the ROS bridge

First run the simulator (see carla documentation: http://carla.readthedocs.io/en/latest/)

    ./CarlaUE4.sh -windowed -ResX=320 -ResY=240 -benchmark -fps=10


Wait for the message:

    Waiting for the client to connect...

Then start the ros bridge (choose one option):

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash

    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # Option 2: start the ros bridge together with RVIZ
    roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch

    # Option 3: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

# Settings

You can setup the ros bridge configuration [carla_ros_bridge/config/settings.yaml](carla_ros_bridge/config/settings.yaml).

If the rolename is within the list specified by ROS parameter `/carla/ego_vehicle/rolename`, the client is interpreted as an controllable ego vehicle and all relevant ROS topics are created.

## Synchronous Mode

In default mode (`/carla/synchronous_mode: false`) data is published:

 - on every `world.on_tick()` callback
 - on every `sensor.listen()` callback

In synchronous mode, the bridge waits for all sensor data that is expected within the current frame. This might slow down the overall simulation but ensures reproducible results.

Additionally it is possible to control the simulation execution:

- Pause/Play
- Execute single step

The following topic allows to control the stepping.

|Topic                          | Type |
|-------------------------------|------|
| `/carla/control` | [carla_msgs.CarlaControl](carla_msgs/msg/CarlaControl.msg) |

A [CARLA Control rqt plugin](rqt_carla_control/README.md) is available to publish to the topic.

# Available ROS Topics

## Ego Vehicle

### Odometry

|Topic                          | Type |
|-------------------------------|------|
| `/carla/<ROLE NAME>/odometry` | [nav_msgs.Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) |

### Sensors

The ego vehicle sensors are provided via topics with prefix /carla/ego_vehicle/<sensor_topic>

Currently the following sensors are supported:

#### Camera

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/camera/rgb/<SENSOR ROLE NAME>/image_color` | [sensor_msgs.Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) |
| `/carla/<ROLE NAME>/camera/rgb/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs.CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) |

#### Lidar

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/lidar/<SENSOR ROLE NAME>/point_cloud` | [sensor_msgs.PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) |

#### GNSS

|Topic                                 | Type | Description |
|--------------------------------------|------|-------------|
| `/carla/<ROLE NAME>/gnss/front/gnss` | [sensor_msgs.NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) | publish gnss location |

#### Collision Sensor

|Topic                          | Type | Description |
|-------------------------------|------|-------------|
| `/carla/<ROLE NAME>/collision` | [carla_msgs.CarlaCollisionEvent](carla_msgs/msg/CarlaCollisionEvent.msg) | publish collision events |

#### Lane Invasion Sensor

|Topic                          | Type | Description |
|-------------------------------|------|-------------|
| `/carla/<ROLE NAME>/lane_invasion` | [carla_msgs.CarlaLaneInvasionEvent](carla_msgs/msg/CarlaLaneInvasionEvent.msg) | publish events on lane-invasion |

### Object Sensor

|Topic         | Type | Description |
|--------------|------|-------------|
| `/carla/<ROLE NAME>/objects` | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) | all vehicles and walkers, except the ego vehicle |

### Control

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/vehicle_control_cmd` (subscriber) | [carla_msgs.CarlaEgoVehicleControl](carla_msgs/msg/CarlaEgoVehicleControl.msg) |
| `/carla/<ROLE NAME>/vehicle_control_cmd_manual` (subscriber) | [carla_msgs.CarlaEgoVehicleControl](carla_msgs/msg/CarlaEgoVehicleControl.msg) |
| `/carla/<ROLE NAME>/vehicle_control_manual_override` (subscriber) | [std_msgs.Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html) |
| `/carla/<ROLE NAME>/vehicle_status` | [carla_msgs.CarlaEgoVehicleStatus](carla_msgs/msg/CarlaEgoVehicleStatus.msg) |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs.CarlaEgoVehicleInfo](carla_msgs/msg/CarlaEgoVehicleInfo.msg) |

There are two modes to control the vehicle.

1. Normal Mode (reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd`)
1. Manual Mode (reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd_manual`)

This allows to manually override a Vehicle Control Commands published by a software stack. You can toggle between the two modes by publishing to `/carla/<ROLE NAME>/vehicle_control_manual_override`.

[carla_manual_control](carla_manual_control/) makes use of this feature.


For testing purposes, you can stear the ego vehicle from the commandline by publishing to the topic `/carla/<ROLE NAME>/vehicle_control_cmd`.

Examples for a ego vehicle with role_name 'ego_vehicle':

Max forward throttle:

     rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 0.0}" -r 10


Max forward throttle with max steering to the right:

     rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10


The current status of the vehicle can be received via topic `/carla/<ROLE NAME>/vehicle_status`.
Static information about the vehicle can be received via `/carla/<ROLE NAME>/vehicle_info`

#### Additional way of controlling

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/twist_cmd` (subscriber) | [geometry_msgs.Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) |

CAUTION: This control method does not respect the vehicle constraints. It allows movements impossible in the real world, like flying or rotating.

You can also control the vehicle via publishing linear and angular velocity within a Twist datatype.

Currently this method applies the complete linear vector, but only the yaw from angular vector.

#### Carla Ackermann Control

In certain cases, the [Carla Control Command](carla_msgs/msg/CarlaEgoVehicleControl.msg) is not ideal to connect to an AD stack.
Therefore a ROS-based node ```carla_ackermann_control``` is provided which reads [AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) messages.
You can find further documentation [here](carla_ackermann_control/README.md).


## Other Topics

### Object information of other actors

|Topic         | Type | Description |
|--------------|------|-------------|
| `/carla/objects` | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) | all vehicles and walkers |
| `/carla/marker` | [visualization_msgs.Maker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html) | visualization of vehicles and walkers |
| `/carla/actor_list` | [carla_msgs.CarlaActorList](carla_msgs/msg/CarlaActorList.msg) | list of all carla actors |


### Status of CARLA

|Topic         | Type |
|--------------|------|
| `/carla/status` | [carla_msgs.CarlaStatus](carla_msgs/msg/CarlaStatus.msg) |


## Map

|Topic         | Type | Description |
|--------------|------|-------------|
| `/carla/map` | [std_msgs.String](http://docs.ros.org/api/std_msgs/html/msg/String.html) | OPEN Drive map description |


## Walker

|Topic                                 | Type | Description |
|--------------------------------------|------|-------------|
| `/carla/<ROLE NAME>/walker_control_cmd` (subscriber) | [carla_msgs.CarlaWalkerControl](carla_msgs/msg/CarlaWalkerControl.msg) | Control a walker |


# Carla Ego Vehicle

`carla_ego_vehicle` provides a generic way to spawn an ego vehicle and attach sensors to it. You can find further documentation [here](carla_ego_vehicle/README.md).


# Waypoint calculation

To make use of the Carla waypoint calculation a ROS Node is available to get waypoints. You can find further documentation [here](carla_waypoint_publisher/README.md).


# Troubleshooting

## ImportError: No module named carla

You're missing Carla Python. Please execute:

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>

Please note that you have to put in the complete path to the egg-file including
the egg-file itself. Please use the one, that is supported by your Python version.
Depending on the type of CARLA (pre-build, or build from source), the egg files
are typically located either directly in the PythonAPI folder or in PythonAPI/dist.

Check the installation is successfull by trying to import carla from python:

    python -c 'import carla;print("Success")'

You should see the Success message without any errors.
