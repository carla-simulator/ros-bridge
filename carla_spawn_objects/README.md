# ROS Carla Spawn Objects

`carla_spawn_objects` can be used to spawn actors (vehicles, sensors, walkers) with attached sensors.

Info: To be able to use `carla_manual_control` a camera with role-name 'view' and resolution of 800x600 is required.

If no specific position is set for vehicles, they will be spawned at a random position.

## Spawning at specific position

- It is possible to specify the position at which the vehicle will be spawned, by defining a ros parameter named `spawn_point_<VEHICLE-NAME>`, with `<VEHICLE-NAME>` specified in the `id` field of the vehicle, in the config file.
- It is also possible to specify the initial position directly in the config file. This is also how the initial positions of sensors should be declared.
- The `spawn_point` specified for a sensor attached to a vehicle, will be considered relative to the vehicle.

It is possible to re-spawn a vehicle at a specific location by publishing to `/carla/<ROLE NAME>/<CONTROLLER_ID>/initialpose`, but only if an `actor.pseudo.control` pseudo-actor (with id `<CONTROLLER_ID>`) is attached to the vehicle. The node `set_initial_pose` should also be running to handle the message on the topic. It can be launched using [set_initial_pose.launch](launch/set_initial_pose.launch), and `<CONTROLLER_ID>` should be specified by setting the ros parameter called `controller_id`.

The preferred way to publish the new pose message is to use RVIZ:

![Autoware Runtime Manager Settings](../docs/images/rviz_set_start_goal.png)

Selecting a Pose with '2D Pose Estimate' will delete the current ego_vehicle and respawn it at the specified position.

## Attach sensor to an existing vehicle

It possible to attach sensors to an existing vehicle. To do so, a `sensor.pseudo.actor_list` should also be spawned (define it in the config file) to give access to a list of active actors. The ROS parameter `spawn_sensors_only` should also be set to True. `carla_spawn_objects` will then check if an actor with same id and type as the one specified in its config file is already active, and if yes attach the sensors to this actor.

## Create your own sensor setup

Sensors, attached to vehicles or not, can be defined via a json file. `carla_spawn_objects` reads it from the file location defined via the private ros parameter `objects_definition_file`.

The format is defined like that:

```json
{ "actors" = [
                {
                "type": "<SENSOR-TYPE>",
                "id": "<NAME>",
                "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                <ADDITIONAL-SENSOR-ATTRIBUTES>
                },
                {
                "type": "<VEHICLE-TYPE>",
                "id": "<VEHICLE-NAME>",
                "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                "sensors":
                    [
                        <SENSORS-TO-ATTACH-TO-VEHICLE>
                    ]
                }
    ...
                ]
}
```

Define sensors with their attributes as described in the Carla Documentation about [Cameras and Sensors](https://github.com/carla-simulator/carla/blob/master/Docs/cameras_and_sensors.md).

An example is provided by [carla_example_ego_vehicle.launch](launch/carla_example_ego_vehicle.launch). It uses the sensors from [objects.json](config/objects.json)
