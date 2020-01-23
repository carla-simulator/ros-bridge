# ROS Ego Vehicle

The reference Carla client `carla_ego_vehicle` can be used to spawn an ego vehicle (role-name: "ego_vehicle") with attached sensors.

Info: To be able to use `carla_manual_control` a camera with role-name 'view' and resolution of 800x600 is required.

If no specific position is set, the ego vehicle is spawned at a random position.

## Spawning at specific position

It is possible to (re)spawn the ego vehicle at the specific location by publishing to `/carla/<ROLE NAME>/initialpose`.

The preferred way of doing that is using RVIZ:

![Autoware Runtime Manager Settings](../docs/images/rviz_set_start_goal.png)

Selecting a Pose with '2D Pose Estimate' will delete the current ego_vehicle and respawn it at the specified position.

## Re-use existing vehicle as ego-vehicle

It is possible to re-use an existing vehicle as ego-vehicle, instead of spawning a new vehicle. In this case, the role_name is used to identify the vehicle
among all CARLA actors through the rolename attribute. Upon success, the requested sensors are attached to this actor, and the actor becomes the new ego vehicle.

To make use of this option, set the ROS parameter spawn_ego_vehicle to false.

## Create your own sensor setup

Sensors, attached to the ego vehicle can be defined via a json file. `carla_ego_vehicle` reads it from the file location defined via the private ros parameter `sensor_definition_file`.

The format is defined like that:

    { "sensors" = [
        {
          "type": "<SENSOR-TYPE>",
          "id": "<NAME>",
          "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, # pose of the sensor, relative to the vehicle
          <ADDITIONAL-SENSOR-ATTRIBUTES>
        },
        ...
      ]
    }

Define sensors with their attributes as described in the Carla Documentation about [Cameras and Sensors](https://github.com/carla-simulator/carla/blob/master/Docs/cameras_and_sensors.md).

An example is provided by [carla_example_ego_vehicle.launch](launch/carla_example_ego_vehicle.launch). It uses the sensors from [sensors.json](config/sensors.json)
