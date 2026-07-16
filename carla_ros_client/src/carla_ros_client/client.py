#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosclient class:

Class that handle communication between CARLA and ROS
"""

import carla

import ros_compatibility as roscomp

from carla_ros_client.actor_factory import ActorFactory


def main(args=None):
    """
    main function for carla simulator ROS client
    maintaining the communication client and the CarlaClient object
    """
    roscomp.init("client", args=args)

    actor_factory = None
    carla_world = None
    carla_client = None
    executor = None
    parameters = {}

    executor = roscomp.executors.MultiThreadedExecutor()
    actor_factory = ActorFactory()
    executor.add_node(actor_factory)

    roscomp.on_shutdown(actor_factory.destroy)

    parameters['host'] = actor_factory.get_param('host', 'localhost')
    parameters['port'] = actor_factory.get_param('port', 2000)
    parameters['timeout'] = actor_factory.get_param('timeout', 2)
    parameters['fixed_delta_seconds'] = actor_factory.get_param('fixed_delta_seconds', 0.0)

    actor_factory.loginfo("Trying to connect to {host}:{port}".format(
        host=parameters['host'], port=parameters['port']))

    try:
        carla_client = carla.Client(
            host=parameters['host'],
            port=parameters['port'])
        carla_client.set_timeout(parameters['timeout'])

        carla_world = carla_client.get_world()
        carla_settings = carla_world.get_settings()
        carla_settings.fixed_delta_seconds = parameters['fixed_delta_seconds']

        if ( carla_settings.fixed_delta_seconds is not None and carla_settings.fixed_delta_seconds > 0.0 ):
            if not carla_settings.synchronous_mode:
                carla_settings.synchronous_mode = True
                actor_factory.loginfo("Enabling synchronous mode for fixed delta seconds...")
            actor_factory.loginfo("Applying fixed delta seconds: {}".format(carla_settings.fixed_delta_seconds))
            carla_world.apply_settings(carla_settings)
            actor_factory.loginfo("Settings applied...")

        actor_factory.start(carla_client)

        actor_factory.spin()

    except (IOError, RuntimeError) as e:
        actor_factory.logerr("Error: {}".format(e))
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
        del carla_world
        del carla_client


if __name__ == "__main__":
    main()
