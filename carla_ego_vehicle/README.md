# ROS Ego Vehicle

The reference Carla client `carla_example_ego_vehicle` can be used to spawn an ego vehicle (role-name: "ego_vehicle") with the following sensors attached to it.

- GNSS
- LIDAR
- Cameras (one front-camera + one camera for visualization in carla_ros_manual_control)
- Collision Sensor
- Lane Invasion Sensor

Info: To be able to use `carla_manual_control` a camera with role-name 'view' and resolution of 800x600 is required.

If no specific position is set, the ego vehicle is spawned at a random position.


### Spawning at specific position

It is possible to (re)spawn the ego vehicle at the specific location by publishing to `/initialpose`.

The preferred way of doing that is using RVIZ:

![Autoware Runtime Manager Settings](../docs/images/rviz_set_start_goal.png)

Selecting a Pose with '2D Pose Estimate' will delete the current ego_vehicle and respawn it at the specified position.


### Create your own sensor setup

To setup your own ego vehicle with sensors, follow a similar approach as in `carla_example_ego_vehicle` by subclassing from `CarlaEgoVehicleBase`.

Define sensors with their attributes as described in the Carla Documentation about [Cameras and Sensors](https://github.com/carla-simulator/carla/blob/master/Docs/cameras_and_sensors.md).

The format is a list of dictionaries. One dictionary has the values as follows:

    {
        'type': '<SENSOR-TYPE>',
        'role_name': '<NAME>',
        'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, # pose of the sensor, relative to the vehicle
        <ADDITIONAL-SENSOR-ATTRIBUTES>
    }

