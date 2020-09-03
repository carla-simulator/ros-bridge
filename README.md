# ROS bridge for CARLA simulator

This ROS package aims at providing a simple ROS bridge for CARLA simulator.

![rviz setup](./docs/images/rviz_carla_default.png "rviz")

**This version requires CARLA 0.9.9.5**

## Features

- Provide Sensor Data (Lidar, Cameras (depth, segmentation, rgb), GNSS, Radar, IMU)
- Provide Object Data (Transforms (via [tf](http://wiki.ros.org/tf)), Traffic light status, Visualization markers, Collision, Lane invasion)
- Control AD Agents (Steer/Throttle/Brake)
- Control CARLA (Support synchronous mode, Play/pause simulation, Set simulation parameters)

### Additional Functionality

Beside the bridging functionality, there are many more features provided in separate packages.

| Name                              | Description                                                                                             |
| --------------------------------- | ------------------------------------------------------------------------------------------------------- |
| [Carla Ego Vehicle](carla_ego_vehicle/README.md) | Provides a generic way to spawn an ego vehicle and attach sensors to it. |
| [Carla Manual Control](carla_manual_control/README.md) | A ROS-based visualization and control tool for an ego vehicle (similar to carla_manual_control.py provided by CARLA) |
| [Carla Infrastructure](carla_infrastructure/README.md) | Provides a generic way to spawn a set of infrastructure sensors defined in a config file. |
| [Carla Waypoint Publisher](carla_waypoint_publisher/README.md) | Provide routes and access to the Carla waypoint API |
| [Carla ROS Scenario Runner](carla_ros_scenario_runner/README.md) | ROS node that wraps the functionality of the CARLA [scenario runner](https://github.com/carla-simulator/scenario_runner) to execute scenarios. |
| [Carla Ackermann Control](carla_ackermann_control/README.md) | A controller to convert ackermann commands to steer/throttle/brake|
| [Carla AD Agent](carla_ad_agent/README.md) | A basic AD agent, that follows a route, avoids collisions with other vehicles and stops on red traffic lights. |
| [Carla Walker Agent](carla_walker_agent/README.md) | A basic walker agent, that follows a route. |
| [RVIZ Carla Plugin](rviz_carla_plugin/README.md) | A [RVIZ](http://wiki.ros.org/rviz) plugin to visualize/control CARLA. |
| [RQT Carla Plugin](rqt_carla_plugin/README.md) | A [RQT](http://wiki.ros.org/rqt) plugin to control CARLA. |

For a quick overview, after following the [Setup section](#setup), please run the [CARLA AD Demo](carla_ad_demo/README.md). It provides a ready-to-use demonstrator of many of the features.


## Setup

### For Users

First add the apt repository:

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
    sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"

Then simply install the ROS bridge:

    sudo apt-get update
    sudo apt-get install carla-ros-bridge

This will install carla-ros-bridge in /opt/carla-ros-bridge

### For Developers

    Create a catkin workspace and install carla_ros_bridge package

    #setup folder structure
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
    cd ~/carla-ros-bridge
    git clone https://github.com/carla-simulator/ros-bridge.git
    cd ros-bridge
    git submodule update --init
    cd ../catkin_ws/src
    ln -s ../../ros-bridge
    source /opt/ros/<kinetic or melodic>/setup.bash
    cd ..

    #install required ros-dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src -r

    #build
    catkin_make

For more information about configuring a ROS environment see
<http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>

## Start the ROS bridge

First run the simulator (see carla documentation: <http://carla.readthedocs.io/en/latest/>)

    # run carla in background
    SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

Wait a few seconds

    export PYTHONPATH=$PYTHONPATH:<path-to-carla>/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg

##### For Users

    source /opt/carla-ros-bridge/<melodic or kinetic>/setup.bash

##### For Developers

    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash

Start the ros bridge (choose one option):

    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # Option 2: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

## Configuration

### Settings file

You can modify the ros bridge configuration by editing [carla_ros_bridge/config/settings.yaml](carla_ros_bridge/config/settings.yaml).

If the rolename is within the list specified by ROS parameter `/carla/ego_vehicle/rolename`, the client is interpreted as an controllable ego vehicle and all relevant ROS topics are created.

### Launch file

Certain parameters can be set within the launch file [carla_ros_bridge.launch](carla_ros_bridge/launch/carla_ros_bridge.launch).

#### Map

The bridge is able to load a CARLA map by setting the launch-file parameter ```town```. Either specify an available CARLA Town (e.g. 'Town01') or a OpenDRIVE file (with ending '.xodr').

#### Mode

##### Default Mode

In default mode (`synchronous_mode: false`) data is published:

-   on every `world.on_tick()` callback
-   on every `sensor.listen()` callback

##### Synchronous Mode

CAUTION: In synchronous mode, only the ros-bridge is allowed to tick. Other CARLA clients must passively wait.

In synchronous mode (`synchronous_mode: true`), the bridge waits for all sensor data that is expected within the current frame. This might slow down the overall simulation but ensures reproducible results.

Additionally you might set `synchronous_mode_wait_for_vehicle_control_command` to `true` to wait for a vehicle control command before executing the next tick.

###### Control Synchronous Mode

It is possible to control the simulation execution:

-   Pause/Play
-   Execute single step

The following topic allows to control the stepping.

| Topic            | Type                                                       |
| ---------------- | ---------------------------------------------------------- |
| `/carla/control` | [carla_msgs.CarlaControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaControl.msg) |

A [CARLA Control rqt plugin](rqt_carla_control/README.md) is available to publish to the topic.

## Available ROS Topics

### Ego Vehicle

#### Sensors

The ego vehicle sensors are provided via topics with prefix /carla/ego_vehicle/&lt;sensor_topic>

Currently the following sensors are supported:

##### Camera

| Topic                                                          | Type                                                                                   |
| -------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `/carla/<ROLE NAME>/camera/rgb/<SENSOR ROLE NAME>/image_color` | [sensor_msgs.Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)           |
| `/carla/<ROLE NAME>/camera/rgb/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs.CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) |

##### Lidar

| Topic                                                     | Type                                                                                     |
| --------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `/carla/<ROLE NAME>/lidar/<SENSOR ROLE NAME>/point_cloud` | [sensor_msgs.PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) |

##### Radar

| Topic                                               | Type                                                                                                                                          |
| --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `/carla/<ROLE NAME>/radar/<SENSOR ROLE NAME>/radar` | [carla_msgs.CarlaRadarMeasurement](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaRadarMeasurement.msg) |

##### IMU

| Topic                    | Type                                                                              |
| ------------------------ | --------------------------------------------------------------------------------- |
| `/carla/<ROLE NAME>/imu` | [sensor_msgs.Imu](https://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) |

##### GNSS

| Topic                                            | Type                                                                                 | Description           |
| ------------------------------------------------ | ------------------------------------------------------------------------------------ | --------------------- |
| `/carla/<ROLE NAME>/gnss/<SENSOR ROLE NAME>/fix` | [sensor_msgs.NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) | publish gnss location |

##### Collision Sensor

| Topic                          | Type                                                                     | Description              |
| ------------------------------ | ------------------------------------------------------------------------ | ------------------------ |
| `/carla/<ROLE NAME>/collision` | [carla_msgs.CarlaCollisionEvent](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaCollisionEvent.msg) | publish collision events |

##### Lane Invasion Sensor

| Topic                              | Type                                                                           | Description                     |
| ---------------------------------- | ------------------------------------------------------------------------------ | ------------------------------- |
| `/carla/<ROLE NAME>/lane_invasion` | [carla_msgs.CarlaLaneInvasionEvent](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaLaneInvasionEvent.msg) | publish events on lane-invasion |

#### Object Sensor

| Topic                        | Type                                                                                                     | Description                                      |
| ---------------------------- | -------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `/carla/<ROLE NAME>/objects` | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) | all vehicles and walkers, except the ego vehicle |

#### Control

| Topic                                                             | Type                                                                           |
| ----------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| `/carla/<ROLE NAME>/vehicle_control_cmd` (subscriber)             | [carla_msgs.CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg) |
| `/carla/<ROLE NAME>/vehicle_control_cmd_manual` (subscriber)      | [carla_msgs.CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg) |
| `/carla/<ROLE NAME>/vehicle_control_manual_override` (subscriber) | [std_msgs.Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html)           |
| `/carla/<ROLE NAME>/vehicle_status`                               | [carla_msgs.CarlaEgoVehicleStatus](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleStatus.msg)   |
| `/carla/<ROLE NAME>/vehicle_info`                                 | [carla_msgs.CarlaEgoVehicleInfo](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleInfo.msg)       |

There are two modes to control the vehicle.

1.  Normal Mode (reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd`)
2.  Manual Mode (reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd_manual`)

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

##### Additional way of controlling

| Topic                                       | Type                                                                             |
| ------------------------------------------- | -------------------------------------------------------------------------------- |
| `/carla/<ROLE NAME>/twist_cmd` (subscriber) | [geometry_msgs.Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) |

CAUTION: This control method does not respect the vehicle constraints. It allows movements impossible in the real world, like flying or rotating.

You can also control the vehicle via publishing linear and angular velocity within a Twist datatype.

Currently this method applies the complete linear vector, but only the yaw from angular vector.

##### Carla Ackermann Control

In certain cases, the [Carla Control Command](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaEgoVehicleControl.msg) is not ideal to connect to an AD stack.
Therefore a ROS-based node `carla_ackermann_control` is provided which reads [AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) messages.
You can find further documentation [here](carla_ackermann_control/README.md).

### Other Topics

#### Object information of other actors

| Topic               | Type                                                                                                     | Description                           |
| ------------------- | -------------------------------------------------------------------------------------------------------- | ------------------------------------- |
| `/carla/objects`    | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) | all vehicles and walkers              |
| `/carla/marker`     | [visualization_msgs.Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)             | visualization of vehicles and walkers |
| `/carla/actor_list` | [carla_msgs.CarlaActorList](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaActorList.msg)                                           | list of all carla actors              |
| `/carla/traffic_lights` | [carla_msgs.CarlaTrafficLightStatusList](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaTrafficLightStatusList.msg)             | list of all traffic lights with their status |
| `/carla/traffic_lights_info` | [carla_msgs.CarlaTrafficLightInfoList](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaTrafficLightInfoList.msg)             | static information for all traffic lights (e.g. position)|
| `/carla/weather_control` | [carla_msgs.CarlaWeatherParameters](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaWeatherParameters.msg)             | set the CARLA weather parameters|

#### Status of CARLA

| Topic               | Type                                                           | Description                                            |
| ------------------- | -------------------------------------------------------------- | ------------------------------------------------------ |
| `/carla/status`     | [carla_msgs.CarlaStatus](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaStatus.msg)       |                                                        |
| `/carla/world_info` | [carla_msgs.CarlaWorldInfo](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaWorldInfo.msg) | Info about the CARLA world/level (e.g. OPEN Drive map) |

### Walker

| Topic                                                | Type                                                                         | Description        |
| ---------------------------------------------------- | ---------------------------------------------------------------------------- | ------------------ |
| `/carla/walker/<ID>/walker_control_cmd` (subscriber) | [carla_msgs.CarlaWalkerControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaWalkerControl.msg)       | Control a walker   |
| `/carla/walker/<ID>/odometry`                        | [nav_msgs.Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) | odometry of walker |

### Other Vehicles

| Topic                          | Type                                                                         | Description         |
| ------------------------------ | ---------------------------------------------------------------------------- | ------------------- |
| `/carla/vehicle/<ID>/odometry` | [nav_msgs.Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) | odometry of vehicle |

### TF

The tf data is published for all traffic participants and sensors. 

#### TF for traffic participants

The `child_frame_id` corresponds with the CARLA actor id.
If a role name is specified, an additional (static) transform with role name as child_frame_id is published.

#### TF for sensors

Sensors publish the transform, when the measurement is done. The `child_frame_id` corresponds with the prefix of the sensor topics.

### Debug Marker

It is possible to draw markers in CARLA.

Caution: Markers might affect the data published by sensors.

The following markers are supported in 'map'-frame:

-   Arrow (specified by two points)
-   Points
-   Cube
-   Line Strip

| Topic                              | Type                                                                                                   | Description                 |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------ | --------------------------- |
| `/carla/debug_marker` (subscriber) | [visualization_msgs.MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html) | draw markers in CARLA world |

## Troubleshooting

### ImportError: No module named carla

You're missing Carla Python. Please execute:

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/carla/dist/<your_egg_file>

Please note that you have to put in the complete path to the egg-file including
the egg-file itself. Please use the one, that is supported by your Python version.
Depending on the type of CARLA (pre-build, or build from source), the egg files
are typically located either directly in the PythonAPI folder or in PythonAPI/dist.

Check the installation is successfull by trying to import carla from python:

    python -c 'import carla;print("Success")'

You should see the Success message without any errors.
