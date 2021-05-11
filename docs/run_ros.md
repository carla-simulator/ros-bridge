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
roslaunch carla_ros_bridge carla_ros_bridge.launch passive:=True
```

The following settings are available:

* __use_sim_time__: This should be set to __True__ to ensure that ROS is using simulation time rather than system time. This parameter will synchronize the ROS [`/clock`][ros_clock] topic with CARLA simulation time.
*  __host and port__: Network settings to connect to CARLA using a Python client.
* __timeout__: Time to wait for a successful connection to the server.
* __passive__: Passive mode is for use in scynchronous mode. When enabled, the ROS bridge will take a backseat and another client __must__ tick the world. ROS bridge will wait for all expected data from all sensors to be received.
*  __synchronous_mode__:
	*  __If false__: Data is published on every `world.on_tick()` and every `sensor.listen()` callback.
	*  __If true (default)__: ROS bridge waits for all the sensor messages expected before the next tick. This might slow down the overall simulation but ensures reproducible results.
*  __synchronous_mode_wait_for_vehicle_control_command__: In synchronous mode, pauses the tick until a vehicle control is completed.
*  __fixed_delta_seconds__: Simulation time (delta seconds) between simulation steps. __It must be lower than 0.1__. Take a look at the [documentation](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/) to learn more about this.
*  __ego_vehicle__: Role names to identify ego vehicles. Relevant topics will be created so these vehicles will be able to be controlled from ROS.
* __town__: Either use an available CARLA town (eg. 'town01') or an OpenDRIVE file (ending in `.xodr`).
*  __register_all_sensors__:
	*  __If false__: Only sensors spawned by the bridge are registered.
	*  __If true (default)__: All the sensors present in the simulation are registered.


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

It is possible to use [AckermannDrive](https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html) messages to control the ego vehicles. This can be achieved through the use of the [CARLA Ackermann Control](carla_ackermann_control.md) package.

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
