#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Entry point for carla simulator ROS bridge
"""

import importlib
import sys
import os

import carla

from carla_ros_bridge.binding.ros_binding import RosBinding
from carla_ros_bridge.bridge import CarlaRosBridge


def main():
    """
    main function for carla simulator ROS bridge
    maintaining the communication client and the CarlaRosBridge object
    """

    # get binding by using a plain parser (as e.g. ros adds additional
    # parameters to working with default argparse module)
    binding = None
    args = sys.argv[1:]
    found = False
    binding_name = None
    for arg in args:
        if arg == '--binding':
            found = True
        elif found is True:
            binding_name = arg

    if binding_name:
        module_name = os.path.basename(binding_name).split('.')[0]
        sys.path.insert(0, os.path.dirname(binding_name))
        module_agent = importlib.import_module(module_name)
        binding = getattr(module_agent, module_agent.__name__)()
    else:
        #default is ros
        binding = RosBinding()

    print("Trying to connect to {host}:{port}".format(
        host=binding.get_parameters()['host'], port=binding.get_parameters()['port']))

    carla_ros_bridge = None
    try:
        carla_client = carla.Client(
            host=binding.get_parameters()['host'],
            port=binding.get_parameters()['port'])
        carla_client.set_timeout(2000)

        carla_world = carla_client.get_world()

        carla_ros_bridge = CarlaRosBridge(carla_client.get_world(), binding)
        carla_ros_bridge.run()
    finally:
        del carla_world
        del carla_client


if __name__ == "__main__":
    main()
