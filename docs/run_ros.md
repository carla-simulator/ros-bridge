# The ROS Bridge package

The `carla_ros_bridge` package is the main package needed to run the basic ROS bridge functionality. In this section you will learn how to prepare the ROS environment, run the ROS bridge, how to configure the settings, usage of synchronous mode, controlling the ego vehicle and a summary of the subscriptions, publications and services available.

- [__Setting the ROS environment__](#setting-the-ros-environment)
    - [Prepare ROS 1 environment](#prepare-ros-1-environment)
    - [Prepare ROS 2 environment](#prepare-ros-2-environment)
- [__Running the ROS bridge__](#running-the-ros-bridge)
- [__Configuring CARLA settings__](#configuring-carla-settings)
- [__Using the ROS bridge in synchronous mode__](#using-the-ros-bridge-in-synchronous-mode)
- [__Ego vehicle control__](#ego-vehicle-control)
- [__ROS API__](#ros-api)
    - [Subscriptions](#subscriptions)
    - [Publications](#publications)
    - [Services](#services)
---

## Setting the ROS environment

The ROS bridge supports both ROS 1 and ROS 2 using separate implementations with a common interface. When you want to run the ROS bridge you will have to set your ROS environment according to your ROS version in every terminal that you use:

#### Prepare ROS 1 environment:

The command to run depends on whether you installed the ROS bridge via the Debian package or via the source build. You will also need to change the ROS version in the path for the Debian option:

```sh
    # For debian installation of ROS bridge. Change the command according to your installed version of ROS.
    source /opt/carla-ros-bridge/<melodic/noetic>/setup.bash

    # For GitHub repository installation of ROS bridge
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

#### Prepare ROS 2 environment:

```sh
    source ./install/setup.bash
```

## Running the ROS bridge

Once you have set your ROS environment and have a CARLA server running, you will need to start the `carla_ros_bridge` package before being able to use any of the other packages. To do that, run the following command:

```sh
    # ROS 1
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # ROS 2
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

There are other launchfiles that combine the above functionality of starting the ROS bridge at the same time as starting other packges or plugins:

- `carla_ros_bridge_with_example_ego_vehicle.launch` (ROS 1) and `carla_ros_bridge_with_example_ego_vehicle.launch.py` (ROS 2) start the ROS bridge along with the [`carla_spawn_objects`](carla_spawn_objects.md) and [`carla_manual_control`](carla_manual_control.md) packages.

---

## Configuring CARLA settings

Configurations should be set either within the launchfile or passed as an argument when running the file from the command line, for example:


```sh
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py passive:=True
```

The following launch arguments are available for `carla_ros_bridge.launch` and `carla_ros_bridge.launch.py`:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `True` | Use simulation time instead of system time. This synchronizes ROS with the [`/clock`][ros_clock] topic. |
| `host` | `carla-server` | IP address or hostname of the CARLA server. |
| `port` | `2000` | TCP port of the CARLA server. |
| `timeout` | `5000` | Time to wait for a successful connection to the CARLA server. |
| `passive` | `False` | Let another client tick the world. This is only valid in synchronous mode; another client must tick CARLA or the simulation will freeze. If the bridge falls behind, it skips frames. |
| `publish_clock` | `True` | Publish ROS `/clock` messages. Disable this only if another component publishes the clock. |
| `synchronous_mode` | `True` | Enable synchronous mode. If enabled, the ROS bridge waits for the expected sensor data before the next tick. |
| `synchronous_mode_wait_for_vehicle_control_command` | `False` | In synchronous mode, pause the tick until a vehicle control command has been received. |
| `fixed_delta_seconds` | `0.05` | Simulation time step in seconds. It must be lower than `0.1`; see the [CARLA documentation](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/). |
| `start_unix_time_stamp` | `0` | Start timestamp offset for ROS simulation time. Values below `0` use the current Unix time. |
| `town` | empty | Load a CARLA town, such as `Town01`, or an OpenDRIVE file ending in `.xodr`. If empty, keep the currently loaded CARLA world. |
| `rt_factor` | `1.0` | Desired real-time factor. Values above `0.0` pace synchronous ticks against a monotonic schedule, catching up after delays of up to 1 s without skipping ticks; a longer overload or pause is not caught up. Values at or below `0.0` disable throttling. |
| `register_all_sensors` | `True` | Register all sensors already present in the simulation. If `False`, only sensors spawned by the bridge are registered. |
| `native_interface` | `True` | Enable CARLA native DDS interfaces. The bridge will not create bridge-side sensor publishers or control subscribers for native sensors. |
| `ego_vehicle_role_name` | `['hero', 'ego_vehicle', 'hero0', 'hero1', 'hero2', 'hero3']` | Role names used to identify ego vehicles. Relevant ROS topics are created for matching vehicles. |
| `publish_static_vehicles` | `True` | Include static vehicles in object list outputs. |
| `publish_compressed_images` | `True` | Publish compressed image topics in addition to raw image topics. |
| `ignore_altitude` | `False` | Disable altitude information in TF, odometry, GNSS, and object-list outputs where supported. |
| `ignore_tilt` | `True` | Disable pitch and roll in pseudo TF, odometry, IMU, and object-list pose output. Yaw is preserved. |
| `georeference_substitution` | empty | Replace the OpenDRIVE georeference string before publishing world information. |
| `grid_convergence` | `None` | Apply grid convergence when publishing the map frame transform. Use `True`, `False`, or `None` for automatic handling. |
| `publish_etsi_messages` | `False` | Publish ETSI MAPEM and SPATEM messages for traffic light information. |
| `mapem_timer_period` | `1.0` | Time in seconds between ETSI MAPEM publications. |
| `spatem_timer_period` | `0.1` | Time in seconds between ETSI SPATEM publications. |
| `integrate_all_junctions` | `False` | Include every CARLA junction in ETSI MAPEM output. If `False`, only junctions with traffic lights are included. |
| `waypoints_search_distance` | `1.0` | Waypoint search distance in meters. |
| `lane_waypoints_count` | `10` | Number of waypoints included in ETSI MAPEM ingress and egress lanes. |
| `log_level` | `info` | ROS logging level for the bridge node (`debug`, `info`, `warn`, `error`, or `fatal`). |

The `carla_ros_bridge_with_example_ego_vehicle.launch` and `carla_ros_bridge_with_example_ego_vehicle.launch.py` launch files expose all bridge arguments above and add the following arguments for the spawned example ego vehicle:

| Argument | Default | Description |
|----------|---------|-------------|
| `role_name` | `ego_vehicle` | Role name for the spawned ego vehicle and manual control. |
| `vehicle_filter` | `vehicle.*` | CARLA blueprint filter used to select the spawned vehicle. |
| `spawn_point` | `None` | Spawn transform for the vehicle. If `None`, CARLA chooses a spawn point. |


[ros_clock]: https://wiki.ros.org/Clock

---

## Using the ROS bridge in synchronous mode

The ROS bridge operates in synchronous mode by default. It will wait for all sensor data that is expected within the current frame to ensure reproducible results. 

When running multiple clients in synchronous mode, only one client is allowed to tick the world. The ROS bridge will by default be the only client allowed to tick the world unless passive mode is enabled. Enabling passive mode in [`ros-bridge/carla_ros_bridge/config/settings.yaml`](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/config/settings.yaml) will make the ROS bridge step back and allow another client to tick the world. __Another client must tick the world, otherwise CARLA will freeze.__

If the ROS bridge is not in passive mode (ROS bridge is the one ticking the world), then there are two ways to send step controls to the server:

- Send a message to the topic `/carla/control` with a [`carla_msgs.CarlaControl`](ros_msgs.md#carlacontrolmsg) message.
- Use the [Control rqt plugin](rqt_plugin.md). This plugin launches a new window with a simple interface. It is then used to manage the steps and publish in the `/carla/control` topic. To use it, run the following command with CARLA in synchronous mode:
```sh
    rqt --standalone rqt_carla_control
```

---

## Ego vehicle control

There are two modes to control the ego vehicle:

1. Normal mode - reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd`
2. Manual mode - reading commands from  `/carla/<ROLE NAME>/vehicle_control_cmd_manual`. This allows to manually override Vehicle Control Commands published by a software stack.

You can toggle between the two modes by publishing to `/carla/<ROLE NAME>/vehicle_control_manual_override`. For an example of this being used see [Carla Manual Control](carla_manual_control.md).

To test steering from the command line:

__1.__ Launch the ROS Bridge with an ego vehicle:

```sh
    # ROS 1
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

    # ROS 2
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

__2.__ In another terminal, publish to the topic `/carla/<ROLE NAME>/vehicle_control_cmd`

```sh
    # Max forward throttle with max steering to the right

    # for ros1
    rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

    # for ros2
    ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

```

The current status of the vehicle can be received via topic `/carla/<ROLE NAME>/vehicle_status`. Static information about the vehicle can be received via `/carla/<ROLE NAME>/vehicle_info`.

It is possible to use [AckermannDriveStamped](https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) messages to control the ego vehicles. This can be achieved through the use of the [CARLA Ackermann Control](carla_ackermann_control.md) package.

---

## ROS API

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/debug_marker` | [visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html) | Draws markers in the CARLA world. |
| `/carla/weather_control` | [carla_msgs/CarlaWeatherParameters](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaWeatherParameters.msg) | Set the CARLA weather parameters |
| `/clock` | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html) | Publishes simulated time in ROS. |

<br>

!!! Note
    When using `debug_marker`, be aware that markers may affect the data published by sensors. Supported markers include: arrow (specified by two points), points, cube and line strip.
<br>

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/status` | [carla_msgs/CarlaStatus](ros_msgs.md#carlastatusmsg) | Read the current status of CARLA |
| `/carla/world_info` | [carla_msgs/CarlaWorldInfo](ros_msgs.md#carlaworldinfomsg) | Information about the current CARLA map. |
| `/clock` | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html) | Publishes simulated time in ROS. |
| `/rosout` | [rosgraph_msgs/Log](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Log.html) | ROS logging. |

<br>

#### Services

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/destroy_object` | [carla_msgs/DestroyObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/DestroyObject.srv) | Destroys an object |
| `/carla/get_blueprints` | [carla_msgs/GetBlueprints.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/GetBlueprints.srv) | Gets blueprints |
| `/carla/spawn_object` | [carla_msgs/SpawnObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/SpawnObject.srv) | Spawn an object |

---
