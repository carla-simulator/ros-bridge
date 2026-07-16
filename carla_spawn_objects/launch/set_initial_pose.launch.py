import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=''
        ),
        launch_ros.actions.Node(
            package='carla_spawn_objects',
            executable='set_initial_pose',
            name='set_initial_pose',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
