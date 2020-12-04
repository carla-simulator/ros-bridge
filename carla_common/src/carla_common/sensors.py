#!/usr/bin/env python

#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions for carla sensors
"""

import carla
import rospy


def create_carla_sensor_transform_from_spec(sensor_spec):
    """
    Create the sensor transform from a given sensor spec
    :param sensor_spec:
    :return:
    """
    sensor_location = carla.Location(x=sensor_spec.pop("x"),
                                     y=sensor_spec.pop("y"),
                                     z=sensor_spec.pop("z"))
    sensor_rotation = carla.Rotation(
        pitch=sensor_spec.pop('pitch', 0.0),
        roll=sensor_spec.pop('roll', 0.0),
        yaw=sensor_spec.pop('yaw', 0.0))
    return carla.Transform(sensor_location, sensor_rotation)


def setup_sensors(world, sensors, attach_to_actor=None):
    """
    Create the sensors defined by the user and attach them to the specified actor
    :param world: carla world
    :param sensors: list of sensors
    :param attach_to_actor: actors where the sensors should be attached to (None by default)
    :return:
    """
    actors = []
    bp_library = world.get_blueprint_library()
    sensor_names = []
    for sensor_spec in sensors:
        try:
            sensor_type = str(sensor_spec.pop("type"))
            sensor_id = str(sensor_spec.pop("id"))

            sensor_name = sensor_type + "/" + sensor_id
            if sensor_name in sensor_names:
                rospy.logfatal(
                    "Sensor rolename '{}' is only allowed to be used once.".format(
                        sensor_spec['id']))
                raise NameError(
                    "Sensor rolename '{}' is only allowed to be used once.".format(
                        sensor_spec['id']))
            sensor_names.append(sensor_name)

            sensor_transform = create_carla_sensor_transform_from_spec(sensor_spec)

            bp = bp_library.find(sensor_type)
            bp.set_attribute('role_name', sensor_id)
            for attribute, value in sensor_spec.items():
                bp.set_attribute(str(attribute), str(value))

        except KeyError as e:
            rospy.logfatal(
                "Sensor will not be spawned, the mandatory attribute {} is missing"
                .format(e))
            raise e

        except IndexError as e:
            rospy.logfatal(
                "Sensor {} will not be spawned, {}".format(sensor_name, e))
            raise e

        # create sensor
        actors.append(world.spawn_actor(bp, sensor_transform,
                                        attach_to=attach_to_actor))
    return actors
