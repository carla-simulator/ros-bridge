# OpenADS specific changes

## General

- Support the CARLA ROS 2 native interface via the [`native_interface`](./docs/run_ros.md) launch argument. When enabled, native CARLA sensor publishers are used and bridge-side sensor actors are skipped where possible.
- Bump the supported CARLA version to [1.0](./carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION) using Unreal Engine 5.8 and the corresponding Python API updates.
- Use a [`SingleThreadedExecutor`](./carla_ros_bridge/src/carla_ros_bridge/bridge.py) for the bridge node. The CARLA world tick already runs in a separate thread, so a `MultiThreadedExecutor` can conflict with the tick/update flow.

## Timing Related

- Pace synchronous ticks against a monotonic schedule, catching up after delays of up to 1 s without skipping ticks. A longer overload or pause is not caught up, so the simulation does not run faster than the real-time factor afterwards.
- Support real-time control by decoupling CARLA ticking from ROS callback processing in the bridge.
- Add [`start_unix_time_stamp`](./docs/run_ros.md) to offset ROS simulation timestamps or start them from the current Unix time.
- Allow disabling bridge-side `/clock` publishing when the CARLA native interface provides time synchronization.
- Support [`use_sim_time`](./docs/run_ros.md) consistently across bridge, spawning, and control nodes.
- Support per-sensor [`sensor_tick`](./docs/ros_sensors.md#object-sensor) for object and ideal object pseudo-sensors, skipping object-list construction on unused ticks.
- Process world updates in [`passive`](./docs/run_ros.md) mode in a separate thread instead of CARLA's `on_tick` callback. A blocking call during the update no longer deadlocks the client that ticks a synchronous CARLA world; if the update falls behind, it skips to the latest frame.

## Map Related

- Rename `frame_id` from `map` to `carla_map`
- Publish a static transform from the OpenDRIVE georeferenced UTM frame to `carla_map`; the georeference can also be overridden through [`georeference_substitution`](./docs/run_ros.md).
- Apply grid-convergence correction for projected map frames where needed, with override support through [`grid_convergence`](./docs/run_ros.md).

## Transform Related

- Add [`ignore_altitude`](./docs/run_ros.md) to flatten altitude in TF, odometry, GNSS, and object-list outputs where supported.
- Add [`ignore_tilt`](./docs/run_ros.md) to suppress pitch and roll while preserving yaw in pseudo TF, odometry, IMU, and object-list pose outputs.
- Let [`carla_spawn_objects`](./carla_spawn_objects/README.md) own the [transform](./docs/ros_sensors.md#sensor-transforms) of sensors whose absolute pose misrepresents them: a sensor spawned without a parent actor that sits in a group or at a ground-relative altitude. It is spawned with the CARLA `no_transform` attribute and its static transform is published relative to the enclosing group. Every other sensor keeps the server-side transform. Can be overridden per sensor through `no_transform`.

## Spawning Related

- Add automatic spawn altitude correction for maps with elevation.
- Extend [`carla_spawn_objects`](./carla_spawn_objects/README.md) with `group` and `blueprint` placeholders to spawn a specific vehicle and its sensor equipment in a reproducible setup.
- Add WGS84/global spawn point support for object definitions; spawn points can use `lat`/`lon` in addition to CARLA `x`/`y` coordinates.
- Add [ground-relative spawn altitudes](./docs/ros_sensors.md#ground-relative-spawn-altitude) for sensor and group definitions; spawn points can use `alt_above_ground`/`z_above_ground` to place an object at a height above the terrain instead of at an absolute altitude. The property is inherited by all `children` and keeps the object's transform ground-relative, which stays consistent with `ignore_altitude`. The ground altitude is taken from the OpenDRIVE map and looked up once per group, and `alt_ground`/`z_ground` can state a known ground altitude to replace the lookup altogether. Vehicles and walkers are not supported and are rejected with an error.
- Add an `open_doors` property for vehicle definitions in [`carla_spawn_objects`](./carla_spawn_objects/README.md) that opens the given doors directly after spawning. It accepts `fl`/`front_left`, `fr`/`front_right`, `rl`/`rear_left`, `rr`/`rear_right` and `all`, as a list or a comma-separated string.
- Spawn at a random driving-lane waypoint when a random pose is requested on a map without spawn points, instead of at the map origin.

## Sensor Related

- Add optional compressed image publishing for cameras via [`publish_compressed_images`](./docs/run_ros.md).
- Optionally include static map objects in object-list output. The object sensors query and convert them once instead of in every update.
- Add an [Ideal Object Sensor](./docs/ros_sensors.md#ideal-object-sensor), which detects vehicles and walkers by range, field of view, bounding-box vertices, and optional occlusion checks.
- Keep an attached Ideal Object Sensor's range, field-of-view, and occlusion checks consistent with a dynamically flattened parent transform when `ignore_altitude` is enabled.
- Speed up the object sensors in dense traffic: the object message of a vehicle or walker is created once per frame and shared by all object sensors, and the Ideal Object Sensor checks the center range of a target before it computes the bounding-box corners and transforms the corners into the sensor frame with a matrix computed once per update instead of with tf2 for every corner.
- Add ETSI MAPEM/SPATEM conversion for traffic-light and junction information in the [Traffic Lights Sensor](./docs/etsi_its_conversion.md).
- Prevent similar sensor names and thus duplicated topics
- Add `/carla/weather` publishing and `/carla/weather_control` handling for CARLA weather parameters.

## Scenario Related

- Add [`carla_ros_scenario_runner_with_selection.launch.py`](./carla_ros_scenario_runner/launch/carla_ros_scenario_runner_with_selection.launch.py), which publishes every `.xosc` file below `scenarios_dir` as an available scenario, filtered to the map given in `SCENARIO_MAP`. Scenarios listed in the file given by `INVALID_SCENARIOS_FILE` are hidden.
- Allow scenario execution in the [RViz plugin](./docs/rviz_plugin.md) only while manual control is off and the ego vehicle is at a standstill (below 0.1 m/s).

## Controlling

- Disable automated control commands while manual control is active.
- Add Xbox wireless controller support to [manual control](./carla_manual_control/src/carla_manual_control/carla_manual_control.py).
- Redesign the manual-control HUD for OpenADSim and show the compass heading clockwise from north.
- Size the manual-control window from the camera image by default (capped at 90 % of the desktop) and fit images into explicitly configured window sizes without cropping.
- Use [`AckermannDriveStamped`](./docs/carla_ackermann_control.md) instead of `AckermannDrive` for Ackermann control input.

## Misc

- Disable CARLA hard version checks
- Add [`docker-ros`](github.com/ika-rwth-aachen/docker-ros) CI pipeline support.
