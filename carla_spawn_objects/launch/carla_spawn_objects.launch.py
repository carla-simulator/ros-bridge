import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),

        launch_ros.actions.Node(
            package='carla_spawn_objects',
            node_executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
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
