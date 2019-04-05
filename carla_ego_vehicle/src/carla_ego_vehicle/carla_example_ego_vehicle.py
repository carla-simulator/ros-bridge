#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Example Carla Ego Vehicle

"""

from carla_ego_vehicle.carla_ego_vehicle_base import CarlaEgoVehicleBase


class CarlaExampleEgoVehicle(CarlaEgoVehicleBase):
    """
    Example Carla Ego Vehicle
    """

    def sensors(self):
        """
        define all sensors attached to your ego vehicle
        """
        return [
            {
                'type': 'sensor.camera.rgb',
                'role_name': 'front',
                'x': 2.0, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.rgb',
                'role_name': 'view',
                'x': -4.5, 'y': 0.0, 'z': 2.8, 'roll': 0.0, 'pitch': -20.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.lidar.ray_cast',
                'role_name': 'lidar1',
                'x': 0.0, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 5000,
                'channels': 32,
                'points_per_second': 320000,
                'upper_fov': 2.0,
                'lower_fov': -26.8,
                'rotation_frequency': 20
            },
            {
                'type': 'sensor.other.gnss',
                'role_name': 'gnss1',
                'x': 1.0, 'y': 0.0, 'z': 2.0
            },
            {
                'type': 'sensor.other.collision',
                'role_name': 'collision1',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            },
            {
                'type': 'sensor.other.lane_invasion',
                'role_name': 'laneinvasion1',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            }
        ]


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    ego_vehicle = CarlaExampleEgoVehicle()
    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    main()
