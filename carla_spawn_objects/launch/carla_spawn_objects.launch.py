import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=os.path.join(get_package_share_directory(
                'carla_spawn_objects'), 'config', 'objects.json')
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),

        launch_ros.actions.Node(
            package='carla_spawn_objects',
            executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
                },
                {
                    'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle')
                },
                {
                    'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
